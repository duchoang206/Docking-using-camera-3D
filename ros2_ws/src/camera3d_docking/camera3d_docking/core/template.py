"""Template của vật cần dock: tập điểm + pháp tuyến trong FRAME VẬT.

Frame vật: gốc tại tâm mặt trước, trên sàn (z=0); +x hướng vào trong vật; +z lên.
Chỉ giữ phần bề mặt NHÌN THẤY ĐƯỢC từ phía robot tiến tới (phía x < 0).

3 cách tạo:
- template_from_boxes : ray-cast từ nhiều điểm nhìn ảo (pallet, rack, dock chữ V...)
- template_from_mesh  : lấy mẫu bề mặt CAD (STL/OBJ), bỏ mặt quay lưng
- template_from_points: từ đám mây quét thực tế đã đưa về frame vật
"""
from __future__ import annotations

import json
import math
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np

from . import fileio
from .cloud import estimate_normals, voxel_downsample
from .shapes import raycast_boxes


@dataclass
class Template:
    points: np.ndarray
    normals: np.ndarray
    name: str = ""
    meta: dict = field(default_factory=dict)

    def __post_init__(self):
        self.points = np.ascontiguousarray(self.points, dtype=np.float64).reshape(-1, 3)
        N = np.asarray(self.normals, dtype=np.float64).reshape(-1, 3)
        norm = np.linalg.norm(N, axis=1, keepdims=True)
        self.normals = np.ascontiguousarray(N / np.where(norm > 0, norm, 1.0))
        if len(self.points) != len(self.normals):
            raise ValueError("points và normals phải cùng số phần tử")

    @property
    def bounds(self):
        return self.points.min(axis=0), self.points.max(axis=0)

    def kdtree(self):
        """KD-tree (scipy cKDTree) trên các điểm template, tạo một lần rồi dùng lại."""
        if getattr(self, "_kdtree", None) is None:
            from scipy.spatial import cKDTree

            self._kdtree = cKDTree(self.points)
        return self._kdtree

    def facing_mask(self, cam_pos_obj, min_cos: float = 0.0) -> np.ndarray:
        """Điểm có pháp tuyến quay về phía camera (frame vật) với cos(góc tới) > min_cos.

        min_cos > 0 loại các mặt nhìn gần song song (camera lấy mẫu rất thưa ở đó).
        """
        v = np.asarray(cam_pos_obj, dtype=float) - self.points
        dist = np.linalg.norm(v, axis=1)
        return (v * self.normals).sum(axis=1) > min_cos * dist

    def save(self, path) -> None:
        path = Path(path)
        if path.suffix.lower() == ".pcd":
            fileio.write_pcd(path, self.points, self.normals, binary=True)
            return
        np.savez_compressed(path, points=self.points, normals=self.normals,
                            name=np.array(self.name), meta=np.array(json.dumps(self.meta)))

    @classmethod
    def load(cls, path) -> "Template":
        path = Path(path)
        if path.suffix.lower() == ".pcd":
            d = fileio.read_pcd(path)
            P = np.c_[d["x"], d["y"], d["z"]].astype(np.float64)
            if "normal_x" in d:
                N = np.c_[d["normal_x"], d["normal_y"], d["normal_z"]]
            else:
                N = estimate_normals(P, k=12, viewpoint=(-2.0, 0.0, 0.5))
            return cls(P, N, name=path.stem, meta={"source": "pcd", "file": path.name})
        with np.load(path, allow_pickle=False) as data:
            return cls(data["points"], data["normals"], name=str(data["name"]),
                       meta=json.loads(str(data["meta"])))


def _merge(points, normals, spacing):
    """Voxel hóa điểm; pháp tuyến lấy từ điểm gốc gần nhất (giữ pháp tuyến chính xác)."""
    from scipy.spatial import cKDTree

    P = np.vstack(points)
    N = np.vstack(normals)
    Q, _ = voxel_downsample(P, spacing)
    _, idx = cKDTree(P).query(Q)
    return Q, N[idx]


def _look_at(pos, target):
    """R_obj_optical cho camera tại pos nhìn về target (trục z quang học hướng tới target)."""
    z = np.asarray(target, float) - np.asarray(pos, float)
    z /= np.linalg.norm(z)
    x = np.cross(z, [0.0, 0.0, 1.0])
    x /= np.linalg.norm(x)
    y = np.cross(z, x)
    return np.column_stack([x, y, z])


def template_from_boxes(boxes, name: str, spacing: float = 0.01, cam_heights=(0.25, 0.6),
                        distances=(1.2, 2.0), angles_deg=(-40, -20, 0, 20, 40),
                        meta: dict | None = None) -> Template:
    """Ray-cast mô hình hộp từ nhiều điểm nhìn phía trước, chỉ giữ điểm nhìn thấy được."""
    lo = np.min([np.asarray(b.center) - np.asarray(b.size) / 2 for b in boxes], axis=0)
    hi = np.max([np.asarray(b.center) + np.asarray(b.size) / 2 for b in boxes], axis=0)
    corners = np.array([[x, y, z] for x in (lo[0], hi[0]) for y in (lo[1], hi[1])
                        for z in (lo[2], hi[2])])
    target = np.array([(lo[0] + hi[0]) / 2, 0.0, (lo[2] + hi[2]) / 2])
    pts, nrm = [], []
    for h in cam_heights:
        for r in distances:
            for a in np.radians(angles_deg):
                pos = np.array([-r * math.cos(a), r * math.sin(a), h])
                R = _look_at(pos, target)
                c_opt = (corners - pos) @ R          # góc hộp bao trong frame quang học
                c_opt = c_opt[c_opt[:, 2] > 1e-3]
                if len(c_opt) == 0:
                    continue
                u = c_opt[:, 0] / c_opt[:, 2]
                v = c_opt[:, 1] / c_opt[:, 2]
                step = spacing / (2.0 * np.linalg.norm(target - pos))
                uu, vv = np.meshgrid(np.arange(u.min(), u.max() + step, step),
                                     np.arange(v.min(), v.max() + step, step))
                d_opt = np.stack([uu.ravel(), vv.ravel(), np.ones(uu.size)], axis=1)
                d = d_opt @ R.T
                d /= np.linalg.norm(d, axis=1, keepdims=True)
                t, n = raycast_boxes(pos, d, boxes, floor_z=None)
                hit = np.isfinite(t)
                pts.append(pos + d[hit] * t[hit, None])
                nrm.append(n[hit])
    if not pts or sum(len(p) for p in pts) == 0:
        raise ValueError("Không có điểm nào nhìn thấy được — kiểm tra mô hình hộp")
    P, N = _merge(pts, nrm, spacing)
    info = {"source": "boxes", "spacing": spacing, "n_boxes": len(boxes)}
    info.update(meta or {})
    return Template(P, N, name=name, meta=info)


def template_from_points(points_obj, viewpoint_obj, voxel: float = 0.01, k_normals: int = 12,
                         name: str = "", meta: dict | None = None) -> Template:
    """Template từ đám mây thực tế đã ở frame vật; pháp tuyến quay về viewpoint."""
    P, _ = voxel_downsample(np.asarray(points_obj, dtype=np.float64), voxel)
    N = estimate_normals(P, k=k_normals, viewpoint=viewpoint_obj)
    info = {"source": "points", "voxel": voxel}
    info.update(meta or {})
    return Template(P, N, name=name, meta=info)


def template_from_mesh(V, F, spacing: float = 0.01, approach_dir=(1.0, 0.0, 0.0), name: str = "",
                       meta: dict | None = None, seed: int = 0) -> Template:
    """Lấy mẫu bề mặt mesh theo diện tích; bỏ mặt quay lưng và mặt đáy (không nhìn thấy).

    Hạn chế: không xét che khuất (đủ dùng cho vật lồi; vật lõm nên quét thực tế).
    """
    V = np.asarray(V, dtype=np.float64)
    F = np.asarray(F, dtype=np.int64)
    a, b, c = V[F[:, 0]], V[F[:, 1]], V[F[:, 2]]
    cr = np.cross(b - a, c - a)
    area2 = np.linalg.norm(cr, axis=1)
    ok = area2 > 1e-12
    n = np.zeros_like(cr)
    n[ok] = cr[ok] / area2[ok, None]
    keep = ok & (n @ np.asarray(approach_dir, float) <= 1e-6) & (n[:, 2] >= -0.5)
    if not keep.any():
        raise ValueError("Không còn mặt nào nhìn thấy được từ hướng tiếp cận")
    a, b, c, n, area = a[keep], b[keep], c[keep], n[keep], area2[keep] / 2.0
    counts = np.maximum(1, np.ceil(4.0 * area / spacing**2)).astype(int)
    rng = np.random.default_rng(seed)
    tri = np.repeat(np.arange(len(area)), counts)
    r1 = np.sqrt(rng.random(len(tri)))
    r2 = rng.random(len(tri))
    P = (1 - r1)[:, None] * a[tri] + (r1 * (1 - r2))[:, None] * b[tri] + (r1 * r2)[:, None] * c[tri]
    Q, N = _merge([P], [n[tri]], spacing)
    info = {"source": "mesh", "spacing": spacing}
    info.update(meta or {})
    return Template(Q, N, name=name, meta=info)
