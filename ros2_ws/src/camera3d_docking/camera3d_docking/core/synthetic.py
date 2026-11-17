"""Dựng cảnh tổng hợp có ground-truth (robot + camera + các vật + nhãn từng điểm) cho test và benchmark."""
from __future__ import annotations

import numpy as np

from .geometry import T_from_xyz_yaw, transform_points
from .shapes import Box, PinholeCamera, camera_mount, place_boxes, render_cloud

# Nhãn pixel/điểm của render_scene (vật thứ k trong `objects` có nhãn k = 0, 1, ...)
LABEL_FLOOR = -1      # sàn
LABEL_CLUTTER = -2    # hộp thêm (extra_boxes): tường, hàng hóa, vật bên cạnh
LABEL_NONE = -3       # pixel không có phép đo (NaN)


def default_camera() -> PinholeCamera:
    """Camera giống RealSense D435 ở chế độ 424x240, HFOV 87°."""
    return PinholeCamera.from_hfov(424, 240, 87.0)


def render_scene(objects, robot_pose=(0.0, 0.0, 0.0), cam_x: float = 0.0,
                 cam_height: float = 0.3, cam_pitch: float = np.radians(10.0),
                 camera: PinholeCamera | None = None, noise=None, rng=None, floor: bool = True,
                 extra_boxes=(), max_range: float = 6.0, min_range: float = 0.2,
                 stride: int = 1) -> dict:
    """Render cảnh: ``objects`` = [(list[Box] trong frame vật, x, y, yaw) trong world].

    Trả dict:
      points_base       (N,3) float32 — điểm hợp lệ trong base_link
      organized_optical (H,W,3) float32 — cloud organized trong frame quang học (có NaN)
      labels            (H,W) int16 — nhãn từng pixel: k = vật thứ k, LABEL_FLOOR/CLUTTER/NONE
      pixel_index       (N,) — chỉ số pixel (phẳng, theo hàng) của từng điểm trong points_base
      point_labels      (N,) int16 — nhãn của từng điểm trong points_base
      T_base_optical, T_world_base, camera_pos_base, camera
    """
    camera = camera or default_camera()
    T_world_base = T_from_xyz_yaw(robot_pose[0], robot_pose[1], 0.0, robot_pose[2])
    T_base_opt = camera_mount(cam_x, cam_height, cam_pitch)
    placed = [place_boxes(bx, x, y, yaw) for (bx, x, y, yaw) in objects]
    extra = list(extra_boxes)
    boxes = [b for group in placed for b in group] + extra
    owner = np.array([k for k, group in enumerate(placed) for _ in group]
                     + [LABEL_CLUTTER] * len(extra), dtype=np.int16)
    org, hit = render_cloud(camera, T_world_base @ T_base_opt, boxes,
                            floor_z=0.0 if floor else None, noise=noise, rng=rng,
                            max_range=max_range, min_range=min_range, stride=stride,
                            return_labels=True)
    labels = np.full(hit.shape, LABEL_NONE, dtype=np.int16)
    labels[hit == -1] = LABEL_FLOOR
    on_box = hit >= 0
    labels[on_box] = owner[hit[on_box]]
    flat = org.reshape(-1, 3)
    pixel_index = np.flatnonzero(np.isfinite(flat).all(axis=1))
    return {
        "points_base": transform_points(T_base_opt, flat[pixel_index]).astype(np.float32),
        "organized_optical": org,
        "labels": labels,
        "pixel_index": pixel_index,
        "point_labels": labels.ravel()[pixel_index],
        "T_base_optical": T_base_opt,
        "T_world_base": T_world_base,
        "camera_pos_base": T_base_opt[:3, 3].copy(),
        "camera": camera,
    }


def warehouse_clutter(kind: str, x: float, y: float, yaw: float, depth: float = 1.2,
                      width: float = 0.8, height: float = 0.144, gap: float = 0.05) -> list:
    """Vật gây nhiễu quanh vật cần dock đặt tại (x, y, yaw) — các tình huống kho hàng hay gặp.

    clean: không có gì.
    wall : tường cách mặt sau của vật ``gap``.
    load : thùng hàng chồng trên pallet, nhô ra trước mặt pallet 2 cm.
    row  : hai chồng hàng hai bên cách ``gap`` (mặt trước thẳng hàng) + tường phía sau.
    depth/width/height là kích thước vật (mặc định pallet EUR 1.2 x 0.8 x 0.144 m).
    """
    wall = Box((depth + gap + 0.05, 0.0, 1.0), (0.10, 4.0, 2.0))
    x0, x1 = -0.02, depth - 0.2
    cargo = Box(((x0 + x1) / 2, 0.0, height + 0.45), (x1 - x0, width - 0.04, 0.90))
    sides = [Box((depth / 2, sgn * (width / 2 + gap + 0.4), 0.6), (depth, 0.8, 1.2))
             for sgn in (1.0, -1.0)]
    table = {"clean": [], "wall": [wall], "load": [cargo], "row": [wall] + sides}
    if kind not in table:
        raise ValueError(f"Cảnh không hỗ trợ: {kind!r} (chọn: {', '.join(table)})")
    return place_boxes(table[kind], x, y, yaw)
