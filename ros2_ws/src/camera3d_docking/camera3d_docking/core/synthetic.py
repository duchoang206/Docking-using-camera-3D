"""Dựng cảnh tổng hợp có ground-truth (robot + camera + các vật) cho test và benchmark."""
from __future__ import annotations

import numpy as np

from .geometry import T_from_xyz_yaw, transform_points
from .shapes import PinholeCamera, camera_mount, place_boxes, render_cloud


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
      T_base_optical, T_world_base, camera_pos_base, camera
    """
    camera = camera or default_camera()
    T_world_base = T_from_xyz_yaw(robot_pose[0], robot_pose[1], 0.0, robot_pose[2])
    T_base_opt = camera_mount(cam_x, cam_height, cam_pitch)
    boxes = [b for (bx, x, y, yaw) in objects for b in place_boxes(bx, x, y, yaw)]
    boxes += list(extra_boxes)
    org = render_cloud(camera, T_world_base @ T_base_opt, boxes,
                       floor_z=0.0 if floor else None, noise=noise, rng=rng,
                       max_range=max_range, min_range=min_range, stride=stride)
    pts = org.reshape(-1, 3)
    pts = pts[np.isfinite(pts).all(axis=1)]
    return {
        "points_base": transform_points(T_base_opt, pts).astype(np.float32),
        "organized_optical": org,
        "T_base_optical": T_base_opt,
        "T_world_base": T_world_base,
        "camera_pos_base": T_base_opt[:3, 3].copy(),
        "camera": camera,
    }
