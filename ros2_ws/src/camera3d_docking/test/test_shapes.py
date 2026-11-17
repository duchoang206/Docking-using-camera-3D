"""Task 3 — core/shapes.py (mô hình hộp, ray-cast, camera ảo)"""
import numpy as np
import pytest

from camera3d_docking.core import geometry as g
from camera3d_docking.core import shapes as s


def test_raycast_hits_box_front_face():
    box = s.Box(center=(2.0, 0.0, 0.5), size=(1.0, 1.0, 1.0))
    t, n = s.raycast_boxes((0.0, 0.0, 0.5), np.array([[1.0, 0.0, 0.0]]), [box])
    assert t[0] == pytest.approx(1.5)
    assert np.allclose(n[0], [-1.0, 0.0, 0.0])


def test_raycast_rotated_box():
    box = s.Box(center=(2.0, 0.0, 0.5), size=(1.0, 0.2, 1.0), yaw=np.pi / 2)
    t, n = s.raycast_boxes((0.0, 0.0, 0.5), np.array([[1.0, 0.0, 0.0]]), [box])
    assert t[0] == pytest.approx(1.9)
    assert np.allclose(n[0], [-1.0, 0.0, 0.0], atol=1e-9)


def test_raycast_floor_and_miss():
    d = np.array([[np.cos(np.pi / 4), 0.0, -np.sin(np.pi / 4)], [1.0, 0.0, 0.0]])
    t, n = s.raycast_boxes((0.0, 0.0, 1.0), d, [], floor_z=0.0)
    assert t[0] == pytest.approx(np.sqrt(2.0))
    assert np.allclose(n[0], [0.0, 0.0, 1.0])
    assert np.isinf(t[1])


def test_camera_mount_optical_axis():
    T_base_opt = s.camera_mount(cam_x=0.2, cam_height=0.3, cam_pitch=np.radians(10.0))
    z_axis = T_base_opt[:3, :3] @ [0.0, 0.0, 1.0]
    assert np.allclose(z_axis, [np.cos(np.radians(10)), 0.0, -np.sin(np.radians(10))])
    assert np.allclose(T_base_opt[:3, 3], [0.2, 0.0, 0.3])


def _on_some_surface(P_world, boxes, tol=1e-6):
    ok = np.abs(P_world[:, 2]) < tol  # sàn
    for b in boxes:
        L = (P_world - np.asarray(b.center)) @ g.rot_z(b.yaw)  # sang hệ cục bộ của hộp
        half = np.asarray(b.size) / 2.0
        inside = (np.abs(L) <= half + tol).all(axis=1)
        on_face = (np.abs(np.abs(L) - half) < tol).any(axis=1)
        ok |= inside & on_face
    return ok


def test_render_points_lie_on_surfaces():
    boxes = s.place_boxes(s.euro_pallet_boxes(), 1.5, 0.1, np.radians(15.0))
    cam = s.PinholeCamera.from_hfov(160, 90, 87.0)
    T_world_opt = s.camera_mount(0.0, 0.3, np.radians(10.0))
    P = s.render_cloud(cam, T_world_opt, boxes, floor_z=0.0, noise=None)
    assert P.shape == (90, 160, 3) and P.dtype == np.float32
    pts = P.reshape(-1, 3)
    pts = pts[np.isfinite(pts).all(axis=1)].astype(np.float64)
    assert len(pts) > 1000
    Pw = g.transform_points(T_world_opt, pts)
    assert _on_some_surface(Pw, boxes, tol=1e-4).all()


def test_render_noise_is_along_ray():
    box = [s.Box(center=(2.0, 0.0, 0.5), size=(0.2, 3.0, 3.0))]
    cam = s.PinholeCamera.from_hfov(40, 30, 60.0)
    T = s.camera_mount(0.0, 0.5, 0.0)
    P = s.render_cloud(cam, T, box, floor_z=None, noise=s.DepthNoise(0.0, 0.01),
                       rng=np.random.default_rng(0))
    z = P[..., 2][np.isfinite(P[..., 2])]
    assert 0.01 < z.std() < 0.08          # có nhiễu, σ ≈ 0.01*1.9² ≈ 0.036
    assert abs(z.mean() - 1.9) < 0.02


def test_euro_pallet_geometry():
    boxes = s.euro_pallet_boxes()
    lo = np.min([np.asarray(b.center) - np.asarray(b.size) / 2 for b in boxes], axis=0)
    hi = np.max([np.asarray(b.center) + np.asarray(b.size) / 2 for b in boxes], axis=0)
    assert np.allclose(lo, [0.0, -0.4, 0.0]) and np.allclose(hi, [1.2, 0.4, 0.144])
    ys = np.linspace(-0.45, 0.45, 9001)
    d = np.tile([1.0, 0.0, 0.0], (len(ys), 1))
    t = np.array([s.raycast_boxes((-1.0, y, 0.06), d[:1], boxes)[0][0] for y in ys])
    gap = (t > 1.01) & (np.abs(ys) < 0.4)
    edges = np.flatnonzero(np.diff(gap.astype(int)))
    widths = ys[edges[1::2]] - ys[edges[0::2]]
    centers = (ys[edges[1::2]] + ys[edges[0::2]]) / 2
    assert np.allclose(widths, [0.2275, 0.2275], atol=2e-4)
    assert np.allclose(np.sort(centers), [-0.18625, 0.18625], atol=2e-4)
