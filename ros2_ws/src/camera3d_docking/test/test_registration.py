"""Task 5 — core/registration.py (+ core/synthetic.py dùng chung)"""
import numpy as np
import pytest

from camera3d_docking.core import cloud as c
from camera3d_docking.core import geometry as g
from camera3d_docking.core import registration as reg
from camera3d_docking.core import shapes as s
from camera3d_docking.core import synthetic as syn
from camera3d_docking.core import template as tp


@pytest.fixture(scope="module")
def pallet_tpl():
    return tp.template_from_boxes(s.euro_pallet_boxes(), "euro_pallet")


@pytest.fixture(scope="module")
def pallet_grid(pallet_tpl):
    return reg.NearestTemplateGrid(pallet_tpl.points, resolution=0.01, margin=0.25)


def _prep(points):
    """Tiền xử lý như estimator: bỏ sàn (z > 3 cm) + voxel 2 cm."""
    P = points[points[:, 2] > 0.03]
    return c.voxel_downsample(P, 0.02)[0]


def _scene(x, y, yaw_deg, noise=True, seed=0, boxes_fn=s.euro_pallet_boxes):
    return syn.render_scene([(boxes_fn(), x, y, np.radians(yaw_deg))],
                            noise=s.DepthNoise() if noise else None,
                            rng=np.random.default_rng(seed))


def _err(T, x, y, yaw_deg):
    ex, ey, eyaw = g.pose2d(T)
    return np.hypot(ex - x, ey - y), abs(np.degrees(g.wrap_angle(eyaw - np.radians(yaw_deg))))


def test_synthetic_scene_points_in_base_frame():
    sc = syn.render_scene([], noise=None)                  # chỉ có sàn
    P = sc["points_base"]
    assert len(P) > 1000 and np.abs(P[:, 2]).max() < 1e-4
    assert np.allclose(sc["camera_pos_base"], [0.0, 0.0, 0.3])


def test_nn_grid_matches_kdtree_within_resolution(pallet_tpl, pallet_grid):
    lo, hi = pallet_tpl.bounds
    Q = np.random.default_rng(0).uniform(lo - 0.15, hi + 0.15, size=(5000, 3))
    idx = pallet_grid.query(Q)
    assert (idx >= 0).all()
    d_grid = np.linalg.norm(Q - pallet_tpl.points[idx], axis=1)
    d_true, _ = pallet_tpl.kdtree().query(Q)
    assert (d_grid <= d_true + 0.87 * pallet_grid.res + 1e-9).all()


def test_nn_grid_outside_returns_minus_one(pallet_grid):
    assert pallet_grid.query(np.array([[10.0, 10.0, 10.0]]))[0] == -1


@pytest.mark.parametrize("use_grid", [False, True])
def test_icp_recovers_pallet_pose(pallet_tpl, pallet_grid, use_grid):
    sc = _scene(1.5, 0.2, 10.0)
    # Lệch ban đầu ±5 cm / ±5°: rộng hơn sai số của tìm kiếm toàn cục (lưới 3 cm, bước 3°)
    T0 = g.T_from_xyz_yaw(1.5 - 0.05, 0.2 + 0.05, 0.0, np.radians(10.0 - 5.0))
    res = reg.icp_point_to_plane(_prep(sc["points_base"]), pallet_tpl, T0,
                                 nn=pallet_grid if use_grid else None,
                                 camera_pos_base=sc["camera_pos_base"])
    dpos, dyaw = _err(res.T_base_obj, 1.5, 0.2, 10.0)
    assert res.converged
    assert dpos < 0.005 and dyaw < 0.3
    assert res.coverage > 0.5 and res.inlier_ratio > 0.8 and not res.degenerate


def test_icp_lateral_offset_recovered(pallet_tpl, pallet_grid):
    sc = _scene(1.4, 0.0, 0.0)
    T0 = g.T_from_xyz_yaw(1.4, 0.06, 0.0, 0.0)
    res = reg.icp_point_to_plane(_prep(sc["points_base"]), pallet_tpl, T0, nn=pallet_grid,
                                 camera_pos_base=sc["camera_pos_base"])
    assert abs(res.T_base_obj[1, 3]) < 0.01
    assert not res.degenerate


def test_icp_flags_degenerate_for_flat_wall():
    plate = tp.template_from_boxes([s.Box((0.01, 0.0, 0.3), (0.02, 1.0, 0.6))], "plate")
    wall = [s.Box((1.6, 0.0, 1.0), (0.02, 8.0, 2.0))]       # tường rộng hơn FOV camera
    sc = syn.render_scene([], extra_boxes=wall, noise=None)
    T0 = g.T_from_xyz_yaw(1.59, 0.1, 0.0, 0.0)
    res = reg.icp_point_to_plane(_prep(sc["points_base"]), plate, T0,
                                 camera_pos_base=sc["camera_pos_base"])
    assert res.degenerate


def test_icp_isolated_plate_is_weakly_observable():
    """Tấm phẳng cô lập: chỉ 2 mép cho thông tin ngang -> sai số ≤ nửa voxel và bị gắn cờ
    suy biến theo mặc định (thận trọng; spec khuyến nghị dock chữ V)."""
    plate_boxes = [s.Box((0.01, 0.0, 0.3), (0.02, 1.0, 0.6))]
    plate = tp.template_from_boxes(plate_boxes, "plate")
    sc = syn.render_scene([(plate_boxes, 1.6, 0.0, 0.0)], noise=None)
    iso = reg.icp_point_to_plane(_prep(sc["points_base"]), plate, g.T_from_xyz_yaw(1.6, 0.02, 0, 0),
                                 camera_pos_base=sc["camera_pos_base"])
    assert abs(iso.T_base_obj[1, 3]) <= 0.012
    assert iso.degenerate


def test_icp_v_dock_is_observable_and_accurate():
    v = s.v_dock_boxes()
    tpl = tp.template_from_boxes(v, "v_dock")
    sc = syn.render_scene([(v, 1.2, 0.05, np.radians(5.0))], noise=s.DepthNoise(),
                          rng=np.random.default_rng(3))
    res = reg.icp_point_to_plane(_prep(sc["points_base"]), tpl,
                                 g.T_from_xyz_yaw(1.17, 0.08, 0.0, np.radians(2.0)),
                                 camera_pos_base=sc["camera_pos_base"])
    dpos, dyaw = _err(res.T_base_obj, 1.2, 0.05, 5.0)
    assert not res.degenerate
    assert dpos < 0.005 and dyaw < 0.5


@pytest.mark.parametrize("off", [(-0.05, 0.05, -5), (0.05, -0.05, 5), (-0.07, 0.0, 0),
                                 (0.0, 0.08, 0), (0.0, 0.0, 6)])
def test_icp_convergence_basin(pallet_tpl, pallet_grid, off):
    sc = _scene(1.5, 0.2, 10.0, seed=1)
    dx, dy, dyaw = off
    T0 = g.T_from_xyz_yaw(1.5 + dx, 0.2 + dy, 0.0, np.radians(10.0 + dyaw))
    res = reg.icp_point_to_plane(_prep(sc["points_base"]), pallet_tpl, T0, nn=pallet_grid,
                                 camera_pos_base=sc["camera_pos_base"])
    dpos, dyaw_err = _err(res.T_base_obj, 1.5, 0.2, 10.0)
    assert dpos < 0.005 and dyaw_err < 0.3


def test_icp_quality_low_for_wrong_object(pallet_tpl):
    sc = _scene(1.5, 0.0, 0.0, boxes_fn=s.rack_boxes)
    res = reg.icp_point_to_plane(_prep(sc["points_base"]), pallet_tpl,
                                 g.T_from_xyz_yaw(1.5, 0.0, 0.0, 0.0),
                                 camera_pos_base=sc["camera_pos_base"])
    assert res.coverage < 0.35 or res.inlier_ratio < 0.5


def test_icp_covariance_positive_definite(pallet_tpl):
    sc = _scene(1.5, 0.0, 5.0)
    res = reg.icp_point_to_plane(_prep(sc["points_base"]), pallet_tpl,
                                 g.T_from_xyz_yaw(1.48, 0.02, 0.0, np.radians(4.0)),
                                 camera_pos_base=sc["camera_pos_base"])
    assert res.cov_xyyaw.shape == (3, 3)
    assert np.all(np.linalg.eigvalsh(res.cov_xyyaw) > 0)
    assert 0.0 < res.sigma_xy < 0.05 and 0.0 < res.sigma_yaw < np.radians(3.0)


def test_icp_coverage_ignores_grazing_surfaces(pallet_tpl, pallet_grid):
    """Mặt trên pallet nhìn gần song song từ camera thấp (0.3 m) -> mẫu rất thưa; không được
    tính vào phần 'phải thấy' của coverage (lỗi phát hiện trong mô phỏng Gazebo)."""
    sc = syn.render_scene([(s.euro_pallet_boxes(), 2.5, 0.0, 0.0)], noise=s.DepthNoise(),
                          rng=np.random.default_rng(5), stride=2)
    res = reg.icp_point_to_plane(_prep(sc["points_base"]), pallet_tpl,
                                 g.T_from_xyz_yaw(2.5, 0.0, 0.0, 0.0), nn=pallet_grid,
                                 camera_pos_base=sc["camera_pos_base"])
    assert res.coverage >= 0.5, res.coverage


def test_icp_handles_too_few_points(pallet_tpl):
    res = reg.icp_point_to_plane(np.zeros((5, 3)), pallet_tpl, g.T_from_xyz_yaw(1.0, 0, 0, 0))
    assert not res.converged and res.score == 0.0
