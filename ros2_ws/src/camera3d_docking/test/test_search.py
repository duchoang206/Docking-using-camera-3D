"""Task 6 — core/search.py (tìm kiếm tương quan 2.5D đa dải + phạt vùng-phải-trống)"""
import numpy as np
import pytest

from camera3d_docking.core import geometry as g
from camera3d_docking.core import search as se
from camera3d_docking.core import shapes as s
from camera3d_docking.core import synthetic as syn
from camera3d_docking.core import template as tp

ROI_LO, ROI_HI = (0.3, -1.5), (3.5, 1.5)


@pytest.fixture(scope="module")
def searcher():
    tpl = tp.template_from_boxes(s.euro_pallet_boxes(), "euro_pallet")
    return se.CorrelativeSearch(tpl, se.SearchParams(z_min=0.03))


def _scene_pts(objects, extra=(), seed=0):
    sc = syn.render_scene(objects, extra_boxes=extra, noise=s.DepthNoise(),
                          rng=np.random.default_rng(seed))
    P = sc["points_base"]
    return P[P[:, 2] > 0.03]          # bỏ sàn như estimator


def test_search_finds_pallet(searcher):
    wall = [s.Box((3.6, 0.0, 1.0), (0.1, 6.0, 2.0))]
    clutter = [s.Box((1.2, 0.9, 0.25), (0.4, 0.4, 0.5))]
    P = _scene_pts([(s.euro_pallet_boxes(), 2.0, -0.3, np.radians(12.0))], extra=wall + clutter)
    hyps = searcher.search(P, ROI_LO, ROI_HI)
    assert len(hyps) >= 1
    best = hyps[0]
    assert np.hypot(best.x - 2.0, best.y + 0.3) < 0.06
    assert abs(np.degrees(g.wrap_angle(best.yaw - np.radians(12.0)))) < 4.0
    assert all(hyps[i].score >= hyps[i + 1].score for i in range(len(hyps) - 1))


def test_search_wall_scores_lower_than_pallet(searcher):
    wall_only = _scene_pts([], extra=[s.Box((2.05, 0.0, 1.0), (0.1, 6.0, 2.0))])
    with_pallet = _scene_pts([(s.euro_pallet_boxes(), 2.0, 0.0, 0.0)])
    s_wall = searcher.search(wall_only, ROI_LO, ROI_HI)
    s_pal = searcher.search(with_pallet, ROI_LO, ROI_HI)
    best_wall = s_wall[0].score if s_wall else 0.0
    assert best_wall < 0.6 * s_pal[0].score


def test_search_hypotheses_are_distinct(searcher):
    P = _scene_pts([(s.euro_pallet_boxes(), 1.8, 0.0, 0.0)],
                   extra=[s.Box((1.0, -0.8, 0.07), (0.6, 0.6, 0.14))])
    hyps = searcher.search(P, ROI_LO, ROI_HI)
    p = searcher.params
    for i in range(len(hyps)):
        for j in range(i + 1, len(hyps)):
            far = np.hypot(hyps[i].x - hyps[j].x, hyps[i].y - hyps[j].y) >= p.nms_dist - 1e-9
            turned = abs(np.degrees(hyps[i].yaw - hyps[j].yaw)) >= p.nms_yaw_deg - 1e-9
            assert far or turned


def test_search_empty_scene_returns_empty(searcher):
    assert searcher.search(np.zeros((0, 3)), ROI_LO, ROI_HI) == []
