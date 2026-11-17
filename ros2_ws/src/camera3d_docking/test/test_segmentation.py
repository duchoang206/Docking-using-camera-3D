"""Segmentation (mặt nạ 2D của vật) -> chỉ dùng điểm của vật khi tìm kiếm / ICP.

Cảnh tổng hợp biết chính xác pixel nào thuộc vật nào, nên tạo được mặt nạ "hoàn hảo"
(oracle — trần trên của mọi mạng segmentation) và các mặt nạ lỗi giống mạng thật:
lem ra nền, hụt mép, chỉ có bbox, lệch giữa ảnh màu và depth.
"""
import numpy as np
import pytest

from camera3d_docking.core import estimator as e
from camera3d_docking.core import geometry as g
from camera3d_docking.core import segmentation as seg
from camera3d_docking.core import shapes as s
from camera3d_docking.core import synthetic as syn
from camera3d_docking.core import template as tp

PALLET = s.euro_pallet_boxes()


@pytest.fixture(scope="module")
def tpl():
    return tp.template_from_boxes(PALLET, "euro_pallet")


def _scene(kind="clean", x=1.8, y=0.0, yaw_deg=0.0, seed=0):
    yaw = np.radians(yaw_deg)
    return syn.render_scene([(PALLET, x, y, yaw)], extra_boxes=syn.warehouse_clutter(kind, x, y, yaw),
                            noise=s.DepthNoise(), rng=np.random.default_rng(seed))


def _pallet_mask(sc):
    """Mặt nạ hoàn hảo của pallet (vật số 0), theo đúng thứ tự points_base."""
    return (sc["labels"] == 0).ravel()[sc["pixel_index"]]


# ------------------------------------------------------------------ nhãn của cảnh tổng hợp
def test_raycast_reports_index_of_hit_box():
    boxes = [s.Box((2.0, 0.0, 0.5), (0.2, 1.0, 1.0)), s.Box((3.0, 0.0, 0.5), (0.2, 3.0, 1.0))]
    D = np.array([[1.0, 0.0, 0.0], [1.0, 0.4, 0.0], [1.0, 0.0, -1.0], [0.0, 0.0, 1.0]])
    D /= np.linalg.norm(D, axis=1, keepdims=True)
    t, _, idx = s.raycast_boxes((0.0, 0.0, 0.5), D, boxes, floor_z=0.0, return_index=True)
    assert idx.tolist() == [0, 1, -1, -2]               # hộp 0 | hộp 1 | sàn | không trúng
    assert np.isfinite(t[:3]).all() and np.isinf(t[3])


def test_render_scene_labels_every_point():
    sc = syn.render_scene([(PALLET, 2.0, 0.0, 0.0)],
                          extra_boxes=syn.warehouse_clutter("load", 2.0, 0.0, 0.0))
    lab, idx, pts = sc["labels"], sc["pixel_index"], sc["points_base"]
    assert lab.shape == sc["organized_optical"].shape[:2] and len(idx) == len(pts)
    org = sc["organized_optical"].reshape(-1, 3)[idx]
    np.testing.assert_allclose(g.transform_points(sc["T_base_optical"], org), pts, atol=1e-5)
    pl = sc["point_labels"]
    np.testing.assert_array_equal(pl, lab.ravel()[idx])
    pallet = pts[pl == 0]
    assert len(pallet) > 300 and pallet[:, 2].max() < 0.15 and pallet[:, 0].min() > 1.99
    assert np.abs(pts[pl == syn.LABEL_FLOOR, 2]).max() < 1e-3
    assert pts[pl == syn.LABEL_CLUTTER, 2].min() > 0.14        # thùng hàng nằm trên pallet
    invalid = ~np.isfinite(sc["organized_optical"]).all(axis=-1)
    assert invalid.any() and (lab[invalid] == syn.LABEL_NONE).all()


def test_warehouse_clutter_kinds():
    assert syn.warehouse_clutter("clean", 2.0, 0.0, 0.0) == []
    for kind in ("wall", "load", "row"):
        assert len(syn.warehouse_clutter(kind, 2.0, 0.0, 0.3)) > 0
    with pytest.raises(ValueError):
        syn.warehouse_clutter("forest", 0.0, 0.0, 0.0)


# ------------------------------------------------------------------ mặt nạ lỗi
def test_degrade_mask_models_segmentation_errors():
    m = np.zeros((20, 30), bool)
    m[5:15, 10:20] = True                                  # vật 10x10 px
    assert np.array_equal(seg.degrade_mask(m, "oracle"), m)
    d = seg.degrade_mask(m, "dilate", 2)                   # lem ra nền 2 px
    assert d[m].all() and d[3, 15] and not d[2, 15]
    er = seg.degrade_mask(m, "erode", 2)                   # hụt mép 2 px
    assert m[er].all() and er.sum() == 36
    sh = seg.degrade_mask(m, "shift", 3)                   # ảnh màu lệch depth 3 px
    assert sh[5:15, 13:23].all() and sh.sum() == m.sum()
    L = np.zeros((20, 30), bool)
    L[5:15, 10:12] = True
    L[13:15, 10:20] = True                                 # vật hình chữ L
    bb = seg.degrade_mask(L, "bbox", 1)                    # detector chỉ cho bbox (+1 px)
    assert bb.sum() == 12 * 12 and bb[4:16, 9:21].all()
    with pytest.raises(ValueError):
        seg.degrade_mask(m, "blur")


def test_mask_to_grid_matches_cloud_resolution():
    m = np.zeros((480, 848), bool)
    m[100:200, 300:500] = True
    grid = seg.mask_to_grid(m, 240, 424)                   # mặt nạ gấp đôi độ phân giải cloud
    assert grid.shape == (240, 424) and grid[75, 200] and not grid[10, 10]
    assert grid.sum() == m.sum() // 4
    assert np.array_equal(seg.mask_to_grid(m, 480, 848), m)
    with pytest.raises(ValueError):
        seg.mask_to_grid(np.zeros((480, 640), bool), 240, 424)   # 4:3 khác 16:9


# ------------------------------------------------------------------ estimator dùng mặt nạ
def test_estimator_restricts_to_masked_points(tpl):
    sc = _scene("load", seed=5)
    est = e.DockPoseEstimator(tpl, e.EstimatorParams())
    det_all = est.process(sc["points_base"], sc["camera_pos_base"])
    est.reset()
    det = est.process(sc["points_base"], sc["camera_pos_base"], object_mask=_pallet_mask(sc))
    assert det_all.mask == "none" and det.mask == "used"
    assert det.status == e.Status.OK, det.message
    assert det.n_points < 0.5 * det_all.n_points            # thùng hàng đã bị loại
    x, y, yaw = g.pose2d(det.T_base_obj)
    assert np.hypot(x - 1.8, y) < 0.015 and abs(np.degrees(yaw)) < 1.0


def test_estimator_falls_back_to_geometry_when_mask_is_empty(tpl):
    sc = _scene(seed=6)
    est = e.DockPoseEstimator(tpl, e.EstimatorParams())
    det = est.process(sc["points_base"], sc["camera_pos_base"],
                      object_mask=np.zeros(len(sc["points_base"]), bool))
    assert det.mask == "too_few" and det.status == e.Status.OK, det.message


def test_estimator_rejects_mask_of_wrong_length(tpl):
    sc = _scene(seed=7)
    est = e.DockPoseEstimator(tpl, e.EstimatorParams())
    with pytest.raises(ValueError):
        est.process(sc["points_base"], object_mask=np.ones(10, bool))


def test_mask_mode_search_uses_mask_only_for_global_search(tpl):
    sc = _scene("load", seed=8)
    m = _pallet_mask(sc)
    soft = e.DockPoseEstimator(tpl, e.EstimatorParams(mask_mode="search", floor_leveling=False))
    d_all = soft.process(sc["points_base"], sc["camera_pos_base"])
    d_soft = soft.process(sc["points_base"], sc["camera_pos_base"], object_mask=m)
    hard = e.DockPoseEstimator(tpl, e.EstimatorParams(mask_mode="hard", floor_leveling=False))
    d_hard = hard.process(sc["points_base"], sc["camera_pos_base"], object_mask=m)
    assert d_soft.mask == "used" and d_soft.n_points == d_all.n_points   # ICP trên mọi điểm
    assert d_hard.n_points < d_soft.n_points                              # ICP chỉ trên pallet
    with pytest.raises(ValueError):
        e.DockPoseEstimator(tpl, e.EstimatorParams(mask_mode="soft"))


def test_mask_used_in_tracking_mode(tpl):
    sc = _scene("load", seed=9)
    est = e.DockPoseEstimator(tpl, e.EstimatorParams())
    prior = g.T_from_xyz_yaw(1.82, -0.02, 0.0, np.radians(1.0))
    det = est.process(sc["points_base"], sc["camera_pos_base"], prior_T_base_obj=prior,
                      object_mask=_pallet_mask(sc))
    assert det.status == e.Status.OK and det.mode == "track" and det.mask == "used"


def test_seg_study_quick_runs(capsys):
    from camera3d_docking.tools import benchmark

    assert benchmark.main(["--seg-study", "--quick", "--scenes", "clean,load",
                           "--seg", "none,oracle,erode3@search"]) == 0
    out = capsys.readouterr().out
    for word in ("clean", "load", "none", "oracle", "erode3@search"):
        assert word in out
