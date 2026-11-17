"""Mô hình vật bằng các hộp + ray-cast + camera depth ảo.

Dùng cho 3 việc:
1. Sinh template "chỉ phần nhìn thấy được" của pallet / rack / dock chữ V.
2. Sinh đám mây điểm tổng hợp có ground-truth cho unit test và benchmark.
3. Sinh world Gazebo (gói camera3d_docking_sim) từ CÙNG mô hình -> template và mô
   phỏng khớp nhau tuyệt đối.
4. Biết tia nào trúng hộp nào -> gán nhãn từng điểm (mặt nạ segmentation "hoàn hảo").

Quy ước frame vật: gốc ở tâm mặt trước, trên sàn (z=0), +x hướng vào trong vật, +z lên.
"""
from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

from .geometry import R_LINK_FROM_OPTICAL, T_from_xyz_rpy, make_T, rot_z


@dataclass(frozen=True)
class Box:
    """Hộp chữ nhật: tâm (x,y,z), kích thước đầy đủ (sx,sy,sz), xoay yaw quanh trục z."""
    center: tuple
    size: tuple
    yaw: float = 0.0


# --------------------------------------------------------------------------- mô hình
def euro_pallet_boxes(depth: float = 1.2, width: float = 0.8, block_h: float = 0.078,
                      board_t: float = 0.022, block_widths=(0.10, 0.145, 0.10),
                      block_depth: float = 0.145,
                      top_widths=(0.145, 0.10, 0.145, 0.10, 0.145)) -> list:
    """Pallet khối kiểu EUR/EPAL (1200x800x144 mm), tiếp cận từ mặt rộng ``width``.

    Lớp (từ dưới lên): 3 ván đáy dọc x | 9 khối | 3 ván ngang (dọc y) | 5 ván mặt dọc x.
    Hốc càng: giữa các hàng khối, cao board_t + block_h (0.1 m), rộng 0.2275 m.
    """
    bw = list(block_widths)
    rows_y = [width / 2 - bw[0] / 2, 0.0, -(width / 2 - bw[2] / 2)]
    cols_x = [block_depth / 2, depth / 2, depth - block_depth / 2]
    z_blk = board_t + block_h / 2
    z_str = board_t + block_h + board_t / 2
    z_top = 2 * board_t + block_h + board_t / 2
    boxes = []
    for r in range(3):  # ván đáy chạy dọc x dưới mỗi hàng khối
        boxes.append(Box((depth / 2, rows_y[r], board_t / 2), (depth, bw[r], board_t)))
    for r in range(3):  # 9 khối
        for c in range(3):
            boxes.append(Box((cols_x[c], rows_y[r], z_blk), (block_depth, bw[r], block_h)))
    for c in range(3):  # ván ngang (stringer) chạy dọc y trên mỗi cột khối
        boxes.append(Box((cols_x[c], 0.0, z_str), (block_depth, width, board_t)))
    tw = list(top_widths)  # 5 ván mặt, khe đều nhau
    gap = (width - sum(tw)) / (len(tw) - 1)
    y = width / 2
    for w in tw:
        boxes.append(Box((depth / 2, y - w / 2, z_top), (depth, w, board_t)))
        y -= w + gap
    return boxes


def rack_boxes(bay_width: float = 1.0, depth: float = 0.9, height: float = 2.0,
               upright=(0.08, 0.08), beam_levels=(0.15, 1.2), beam_h: float = 0.10,
               beam_t: float = 0.05) -> list:
    """Kệ 1 khoang: 4 cột đứng + dầm trước/sau ở mỗi tầng. Mặt trước cột tại x=0."""
    ux, uy = upright
    yc = bay_width / 2 + uy / 2
    boxes = []
    for x in (ux / 2, depth - ux / 2):
        for y in (yc, -yc):
            boxes.append(Box((x, y, height / 2), (ux, uy, height)))
    for z in beam_levels:
        for x in (beam_t / 2, depth - beam_t / 2):
            boxes.append(Box((x, 0.0, z + beam_h / 2), (beam_t, bay_width, beam_h)))
    return boxes


def v_dock_boxes(opening: float = 0.5, depth: float = 0.2, height: float = 0.25,
                 thickness: float = 0.03) -> list:
    """Dock (trạm sạc) có rãnh chữ V mở về phía robot: đỉnh V tại x=depth, miệng tại x=0."""
    half = opening / 2
    length = math.hypot(depth, half)
    ang = math.atan2(half, depth)
    boxes = [
        Box((depth / 2, half / 2, height / 2), (length, thickness, height), -ang),  # cánh trái
        Box((depth / 2, -half / 2, height / 2), (length, thickness, height), ang),  # cánh phải
        Box((depth + 0.05, 0.0, height / 2), (0.10, opening + 0.10, height)),      # thân sau
    ]
    return boxes


def place_boxes(boxes, x: float, y: float, yaw: float) -> list:
    """Đặt mô hình (frame vật) lên sàn world tại (x, y, yaw)."""
    R = rot_z(yaw)
    out = []
    for b in boxes:
        c = R @ np.asarray(b.center, dtype=float) + np.array([x, y, 0.0])
        out.append(Box(tuple(c), tuple(b.size), b.yaw + yaw))
    return out


# --------------------------------------------------------------------------- ray-cast
def raycast_boxes(origin, dirs, boxes, floor_z=None, max_range=np.inf, return_index: bool = False):
    """Ray-cast một gốc tới nhiều tia (N,3) đơn vị; trả (t khoảng cách, pháp tuyến).

    t = inf ở tia không trúng. Phương pháp slab trong hệ cục bộ của từng hộp.
    return_index=True: trả thêm chỉ số hộp mà tia trúng (-1 = sàn, -2 = không trúng).
    """
    o = np.asarray(origin, dtype=float)
    D = np.asarray(dirs, dtype=float).reshape(-1, 3)
    n_rays = len(D)
    t_best = np.full(n_rays, np.inf)
    n_best = np.zeros((n_rays, 3))
    i_best = np.full(n_rays, -2, dtype=np.int32)
    rows = np.arange(n_rays)
    for k, b in enumerate(boxes):
        Rt = rot_z(b.yaw).T                         # world -> cục bộ
        ol = Rt @ (o - np.asarray(b.center, dtype=float))
        dl = D @ Rt.T
        half = np.asarray(b.size, dtype=float) / 2.0
        with np.errstate(divide="ignore", invalid="ignore"):
            inv = 1.0 / dl
            t1 = (-half - ol) * inv
            t2 = (half - ol) * inv
        tmin = np.fmin(t1, t2)
        tmax = np.fmax(t1, t2)
        tmin = np.where(np.isnan(tmin), -np.inf, tmin)
        tmax = np.where(np.isnan(tmax), np.inf, tmax)
        t_in = tmin.max(axis=1)
        t_out = tmax.min(axis=1)
        hit = (t_in <= t_out) & (t_in > 1e-9) & (t_in < t_best)
        if not hit.any():
            continue
        ax = tmin.argmax(axis=1)
        nl = np.zeros((n_rays, 3))
        nl[rows, ax] = -np.sign(dl[rows, ax])
        t_best[hit] = t_in[hit]
        n_best[hit] = nl[hit] @ Rt                   # cục bộ -> world
        i_best[hit] = k
    if floor_z is not None:
        with np.errstate(divide="ignore", invalid="ignore"):
            tf = (floor_z - o[2]) / D[:, 2]
        hit = (D[:, 2] < 0.0) & (tf > 1e-9) & (tf < t_best)
        t_best[hit] = tf[hit]
        n_best[hit] = (0.0, 0.0, 1.0)
        i_best[hit] = -1
    far = t_best > max_range
    t_best[far] = np.inf
    n_best[far] = 0.0
    i_best[far] = -2
    if return_index:
        return t_best, n_best, i_best
    return t_best, n_best


# --------------------------------------------------------------------------- camera ảo
@dataclass
class PinholeCamera:
    width: int
    height: int
    fx: float
    fy: float
    cx: float
    cy: float

    @classmethod
    def from_hfov(cls, width: int, height: int, hfov_deg: float) -> "PinholeCamera":
        fx = (width / 2.0) / math.tan(math.radians(hfov_deg) / 2.0)
        return cls(width, height, fx, fx, (width - 1) / 2.0, (height - 1) / 2.0)

    def ray_dirs(self, stride: int = 1) -> np.ndarray:
        """Hướng tia đơn vị trong frame quang học, mảng (H', W', 3)."""
        uu, vv = np.meshgrid(np.arange(0, self.width, stride, dtype=float),
                             np.arange(0, self.height, stride, dtype=float))
        d = np.stack([(uu - self.cx) / self.fx, (vv - self.cy) / self.fy, np.ones_like(uu)], -1)
        return d / np.linalg.norm(d, axis=-1, keepdims=True)


@dataclass(frozen=True)
class DepthNoise:
    """Nhiễu depth kiểu camera stereo: sigma(z) = sigma_const + sigma_quad * z^2."""
    sigma_const: float = 0.001
    sigma_quad: float = 0.002

    def sigma(self, z):
        return self.sigma_const + self.sigma_quad * np.square(z)


def camera_mount(cam_x: float, cam_height: float, cam_pitch: float, cam_y: float = 0.0,
                 cam_yaw: float = 0.0) -> np.ndarray:
    """T_base_optical cho camera gắn trên robot; cam_pitch > 0 là chúc xuống."""
    T_base_link = T_from_xyz_rpy(cam_x, cam_y, cam_height, 0.0, cam_pitch, cam_yaw)
    return T_base_link @ make_T(R_LINK_FROM_OPTICAL, (0.0, 0.0, 0.0))


def render_cloud(cam: PinholeCamera, T_world_optical, boxes_world, floor_z=0.0,
                 noise: DepthNoise | None = None, rng=None, max_range: float = 6.0,
                 min_range: float = 0.0, stride: int = 1, return_labels: bool = False):
    """Đám mây điểm organized (H', W', 3) float32 trong frame QUANG HỌC; NaN = không đo được.

    return_labels=True: trả thêm (H', W') int32 = chỉ số hộp mà tia trúng (-1 sàn, -2 không đo).
    """
    dirs = cam.ray_dirs(stride)
    H, W = dirs.shape[:2]
    d_opt = dirs.reshape(-1, 3)
    R = np.asarray(T_world_optical)[:3, :3]
    o = np.asarray(T_world_optical)[:3, 3]
    t, _, hit = raycast_boxes(o, d_opt @ R.T, boxes_world, floor_z=floor_z, max_range=np.inf,
                              return_index=True)
    P = d_opt * t[:, None]
    z = P[:, 2]
    valid = np.isfinite(t) & (z <= max_range) & (z >= min_range)
    if noise is not None:
        rng = np.random.default_rng() if rng is None else rng
        zv = z[valid]
        scale = (zv + rng.normal(0.0, 1.0, size=zv.shape) * noise.sigma(zv)) / zv
        P[valid] *= scale[:, None]
    P[~valid] = np.nan
    P = P.reshape(H, W, 3).astype(np.float32)
    if return_labels:
        return P, np.where(valid, hit, -2).reshape(H, W)
    return P
