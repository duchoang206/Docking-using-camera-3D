"""Task 7 — core/estimator.py (+ tools/benchmark.py)"""
import numpy as np
import pytest

from camera3d_docking.core import estimator as e
from camera3d_docking.core import geometry as g
from camera3d_docking.core import shapes as s
from camera3d_docking.core import synthetic as syn
from camera3d_docking.core import template as tp

PALLET = s.euro_pallet_boxes()


@pytest.fixture(scope="module")
def tpl():
    return tp.template_from_boxes(PALLET, "euro_pallet")


@pytest.fixture(scope="module")
def est(tpl):
    return e.DockPoseEstimator(tpl, e.EstimatorParams())


def _render(objects, extra=(), seed=0, pitch_assumed_deg=None):
    sc = syn.render_scene(objects, extra_boxes=extra, noise=s.DepthNoise(),
                          rng=np.random.default_rng(seed))
    if pitch_assumed_deg is not None:  # ngoại tham số sai: robot tưởng camera chúc khác thực tế
        org = sc["organized_optical"].reshape(-1, 3)
        org = org[np.isfinite(org).all(axis=1)]
        T_wrong = s.camera_mount(0.0, 0.3, np.radians(pitch_assumed_deg))
        sc["points_base"] = g.transform_points(T_wrong, org).astype(np.float32)
    return sc


def _err(det, x, y, yaw_deg):
    ex, ey, eyaw = g.pose2d(det.T_base_obj)
    return np.hypot(ex - x, ey - y), abs(np.degrees(g.wrap_angle(eyaw - np.radians(yaw_deg))))


def test_estimator_detects_pallet_from_scratch(est):
    est.reset()
    sc = _render([(PALLET, 1.8, 0.25, np.radians(-12.0))])
    det = est.process(sc["points_base"], camera_pos_base=sc["camera_pos_base"])
    assert det.status == e.Status.OK, det.message
    dpos, dyaw = _err(det, 1.8, 0.25, -12.0)
    assert dpos < 0.015 and dyaw < 1.0
    assert det.mode == "search" and det.cov_xyyaw.shape == (3, 3)


def test_estimator_tracking_with_prior(est):
    est.reset()
    sc = _render([(PALLET, 1.5, 0.0, np.radians(5.0))], seed=1)
    prior = g.T_from_xyz_yaw(1.52, 0.02, 0.0, np.radians(6.0))
    det = est.process(sc["points_base"], sc["camera_pos_base"], prior_T_base_obj=prior)
    assert det.status == e.Status.OK and det.mode == "track"
    dpos, dyaw = _err(det, 1.5, 0.0, 5.0)
    assert dpos < 0.01 and dyaw < 0.5
    det_s = est.process(sc["points_base"], sc["camera_pos_base"])
    assert det.timings["total"] < det_s.timings["total"]


def test_estimator_track_falls_back_to_search_on_bad_prior(est):
    est.reset()
    sc = _render([(PALLET, 1.6, -0.2, 0.0)], seed=2)
    bad_prior = g.T_from_xyz_yaw(1.6, 0.6, 0.0, np.radians(20.0))  # lệch 0.8 m
    det = est.process(sc["points_base"], sc["camera_pos_base"], prior_T_base_obj=bad_prior)
    assert det.status == e.Status.OK and det.mode == "search"
    assert _err(det, 1.6, -0.2, 0.0)[0] < 0.015


def test_estimator_empty_and_nan_cloud(est):
    assert est.process(np.zeros((0, 3))).status == e.Status.NO_POINTS
    assert est.process(np.full((100, 3), np.nan)).status == e.Status.NO_POINTS


def test_estimator_not_found_on_wall_only(est):
    est.reset()
    sc = _render([], extra=[s.Box((2.0, 0.0, 1.0), (0.1, 6.0, 2.0))])
    det = est.process(sc["points_base"], sc["camera_pos_base"])
    assert det.status != e.Status.OK


def test_estimator_floor_leveling_corrects_pitch_error(tpl):
    sc = _render([(PALLET, 2.0, 0.1, 0.0)], seed=3, pitch_assumed_deg=12.0)  # thực tế 10°
    det = e.DockPoseEstimator(tpl, e.EstimatorParams(floor_leveling=True)).process(
        sc["points_base"], sc["camera_pos_base"])
    assert det.status == e.Status.OK, det.message
    assert _err(det, 2.0, 0.1, 0.0)[0] < 0.02


def test_estimator_ambiguous_two_pallets(tpl, est):
    est.reset()
    sc = _render([(PALLET, 2.0, 0.65, 0.0), (PALLET, 2.0, -0.65, 0.0)], seed=4)
    det = est.process(sc["points_base"], sc["camera_pos_base"])
    assert det.status == e.Status.AMBIGUOUS, det.message
    left_only = e.DockPoseEstimator(tpl, e.EstimatorParams(roi_min=(0.25, 0.0, 0.03)))
    det2 = left_only.process(sc["points_base"], sc["camera_pos_base"])
    assert det2.status == e.Status.OK, det2.message
    assert abs(det2.T_base_obj[1, 3] - 0.65) < 0.02


def test_benchmark_quick_runs():
    from camera3d_docking.tools import benchmark

    assert benchmark.main(["--quick"]) == 0
