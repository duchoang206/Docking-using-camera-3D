"""Task 1 — core/geometry.py"""
import numpy as np
import pytest

from camera3d_docking.core import geometry as g


def test_inv_T_roundtrip():
    T = g.T_from_xyz_rpy(0.3, -1.2, 0.5, 0.1, -0.2, 2.5)
    assert np.allclose(g.inv_T(T) @ T, np.eye(4), atol=1e-12)


def test_yaw_of_rot_z():
    assert g.yaw_of(g.rot_z(0.3)) == pytest.approx(0.3)
    assert g.yaw_of(g.T_from_xyz_yaw(1.0, 2.0, 3.0, -2.0)) == pytest.approx(-2.0)


def test_quat_roundtrip():
    rng = np.random.default_rng(0)
    for _ in range(50):
        R = g.so3_exp(rng.normal(size=3))
        q = g.quat_from_R(R)
        assert np.linalg.norm(q) == pytest.approx(1.0)
        assert np.allclose(g.R_from_quat(q), R, atol=1e-9)


def test_quat_convention_xyzw():
    q = g.quat_from_R(g.rot_z(np.pi / 2))
    assert np.allclose(q, [0.0, 0.0, np.sin(np.pi / 4), np.cos(np.pi / 4)])


def test_wrap_angle():
    assert g.wrap_angle(3 * np.pi / 2) == pytest.approx(-np.pi / 2)
    assert g.wrap_angle(-3 * np.pi / 2) == pytest.approx(np.pi / 2)
    assert np.allclose(g.wrap_angle(np.array([0.0, 2 * np.pi + 0.1])), [0.0, 0.1])


def test_so3_exp_matches_rot_z():
    assert np.allclose(g.so3_exp(np.array([0.0, 0.0, 0.7])), g.rot_z(0.7))
    assert np.allclose(g.so3_exp(np.zeros(3)), np.eye(3))


def test_optical_convention():
    R = g.R_LINK_FROM_OPTICAL
    assert np.allclose(R @ [0, 0, 1], [1, 0, 0])   # optical z (tới trước) -> link x
    assert np.allclose(R @ [1, 0, 0], [0, -1, 0])  # optical x (phải) -> link -y
    assert np.allclose(R @ [0, 1, 0], [0, 0, -1])  # optical y (xuống) -> link -z


def test_transform_points_and_pose2d():
    T = g.T_from_xyz_yaw(1.0, 2.0, 0.5, np.pi / 2)
    assert np.allclose(g.transform_points(T, np.array([[1.0, 0.0, 0.0]])), [[1.0, 3.0, 0.5]])
    x, y, yaw = g.pose2d(T)
    assert (x, y) == pytest.approx((1.0, 2.0))
    assert yaw == pytest.approx(np.pi / 2)


def test_transform_points_keeps_float32():
    P = np.ones((5, 3), dtype=np.float32)
    assert g.transform_points(g.T_from_xyz_yaw(1, 0, 0, 0.2), P).dtype == np.float32


def test_rpy_roundtrip():
    T = g.T_from_xyz_rpy(0.0, 0.0, 0.0, 0.1, -0.2, 0.3)
    assert np.allclose(g.rpy_from_R(T[:3, :3]), [0.1, -0.2, 0.3])
