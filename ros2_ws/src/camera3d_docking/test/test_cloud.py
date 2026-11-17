"""Task 2 — core/cloud.py"""
import numpy as np

from camera3d_docking.core import cloud as c
from camera3d_docking.core import geometry as g


def _grid(n=100, z=0.0):
    xs, ys = np.meshgrid(np.linspace(0.0, 1.0, n), np.linspace(0.0, 1.0, n))
    return np.c_[xs.ravel(), ys.ravel(), np.full(xs.size, z)]


def test_finite_points_drops_nan_inf():
    P = np.array([[0, 0, 0], [np.nan, 1, 1], [1, np.inf, 0], [2, 2, 2]], float)
    assert c.finite_points(P).shape == (2, 3)


def test_crop_box_mask():
    P = np.array([[0.5, 0, 0], [2, 0, 0], [0.5, 0, 3]], float)
    assert c.crop_box_mask(P, (0, -1, -1), (1, 1, 1)).tolist() == [True, False, False]


def test_voxel_downsample_reduces_and_preserves_centroid():
    P = _grid(100)
    Q, counts = c.voxel_downsample(P, 0.05)
    assert len(Q) < len(P) / 10
    assert counts.sum() == len(P)
    weighted = (Q * counts[:, None]).sum(0) / counts.sum()
    assert np.allclose(weighted, P.mean(0), atol=1e-9)


def test_voxel_handles_negative_coordinates():
    P = np.array([[-0.01, -0.01, -0.01], [0.01, 0.01, 0.01]])
    Q, counts = c.voxel_downsample(P, 0.05)
    assert len(Q) == 2 and counts.tolist() == [1, 1]


def test_voxel_min_points_removes_isolated():
    dense = np.random.default_rng(0).uniform(0.0, 0.04, size=(50, 3))
    Q, counts = c.voxel_downsample(np.vstack([dense, [[1.0, 1.0, 1.0]]]), 0.05, min_points=2)
    assert len(Q) == 1 and counts[0] == 50


def test_voxel_empty_input():
    Q, counts = c.voxel_downsample(np.zeros((0, 3)), 0.05)
    assert Q.shape == (0, 3) and counts.shape == (0,)


def test_estimate_normals_plane_oriented_to_viewpoint():
    N = c.estimate_normals(_grid(30), k=10, viewpoint=(0.5, 0.5, 1.0))
    ang = np.degrees(np.arccos(np.clip(N @ [0.0, 0.0, 1.0], -1.0, 1.0)))
    assert ang.max() < 2.0


def test_ransac_plane_with_outliers():
    rng = np.random.default_rng(1)
    R = g.rot_x(np.radians(5.0))
    plane = (_grid(40) - [0.5, 0.5, 0.0]) @ R.T
    outliers = rng.uniform(-1.0, 1.0, size=(int(len(plane) * 0.43), 3))  # ~30% tổng số điểm
    fit = c.fit_plane_ransac(np.vstack([plane, outliers]), threshold=0.01, iterations=200, rng=rng)
    true_n = R @ [0.0, 0.0, 1.0]
    assert np.degrees(np.arccos(abs(fit.normal @ true_n))) < 0.5
    assert fit.normal[2] > 0
    assert fit.inliers.sum() >= len(plane)


def test_ransac_returns_none_for_too_few_points():
    assert c.fit_plane_ransac(np.zeros((2, 3))) is None


def test_leveling_transform_flattens_tilted_floor():
    R = g.rot_y(np.radians(3.0))
    floor = (_grid(40) - [0.5, 0.5, 0.0]) @ R.T + [0.0, 0.0, 0.05]
    fit = c.fit_plane_ransac(floor, threshold=0.005, rng=np.random.default_rng(0))
    T = c.leveling_transform(fit.normal, fit.d, floor_z=0.0)
    assert np.abs(g.transform_points(T, floor)[:, 2]).max() < 0.002
    assert abs(g.yaw_of(T)) < 1e-9          # không đổi yaw
    assert np.allclose(T[:2, 3], 0.0)       # không đổi x, y
