"""Pipeline ước lượng pose vật cần dock từ đám mây điểm (thuần numpy/scipy, không ROS).

Mỗi khung:  cân bằng sàn (RANSAC) -> cắt ROI + xóa sàn -> voxel
            -> [bám: ICP từ pose dự đoán]  hoặc  [mất dấu: tìm kiếm tương quan -> ICP top-k]
            -> cổng chất lượng (coverage, inlier, rmse, sigma, suy biến, mơ hồ)
Nguyên tắc: thà trả trạng thái "không chắc chắn" còn hơn trả pose sai.
"""
from __future__ import annotations

import math
import time
from dataclasses import dataclass, field
from enum import Enum

import numpy as np

from .cloud import crop_box_mask, finite_points, fit_plane_ransac, leveling_transform, voxel_downsample
from .geometry import T_from_xyz_yaw, make_T, pose2d, so3_exp, transform_points, wrap_angle
from .registration import IcpParams, NearestTemplateGrid, icp_point_to_plane
from .search import CorrelativeSearch, SearchParams


class Status(str, Enum):
    OK = "OK"
    NO_POINTS = "NO_POINTS"
    NOT_FOUND = "NOT_FOUND"
    LOW_QUALITY = "LOW_QUALITY"
    DEGENERATE = "DEGENERATE"
    AMBIGUOUS = "AMBIGUOUS"


@dataclass
class QualityParams:
    min_coverage: float = 0.35
    min_inlier_ratio: float = 0.5
    max_rmse: float = 0.015
    max_sigma_xy: float = 0.02
    max_sigma_yaw_deg: float = 1.5
    ambiguity_ratio: float = 0.9
    ambiguity_dist: float = 0.05
    ambiguity_yaw_deg: float = 3.0
    track_jump_xy: float = 0.25
    track_jump_yaw_deg: float = 15.0


@dataclass
class EstimatorParams:
    roi_min: tuple = (0.25, -1.5, 0.03)    # (x, y, z TRÊN SÀN) trong base_link
    roi_max: tuple = (3.5, 1.5, 2.0)
    floor_z: float = 0.0                   # độ cao sàn trong base frame
    voxel_size: float = 0.02
    min_points_per_voxel: int = 1
    min_scene_points: int = 50
    floor_leveling: bool = True
    floor_band: float = 0.15
    floor_ransac_thresh: float = 0.015
    floor_max_tilt_deg: float = 10.0
    floor_max_dz: float = 0.10
    floor_ema: float = 0.3
    floor_min_inliers: int = 200
    floor_update_every: int = 1            # cập nhật cân bằng sàn mỗi N khung
    nn_grid: bool = True                   # lưới tra cứu tương ứng (nhanh trên CPU yếu)
    nn_resolution: float = 0.01
    search: SearchParams = field(default_factory=SearchParams)
    icp: IcpParams = field(default_factory=IcpParams)
    quality: QualityParams = field(default_factory=QualityParams)


@dataclass
class Detection:
    status: Status
    T_base_obj: np.ndarray | None = None
    cov_xyyaw: np.ndarray | None = None
    quality: dict = field(default_factory=dict)
    timings: dict = field(default_factory=dict)
    n_points: int = 0
    message: str = ""
    mode: str = ""
    floor: dict = field(default_factory=dict)


class DockPoseEstimator:
    def __init__(self, template, params: EstimatorParams | None = None):
        self.template = template
        self.params = p = params or EstimatorParams()
        z_cut = p.floor_z + p.roi_min[2]          # phần template dưới mức cắt sàn không thể thấy
        p.search.z_min = max(p.search.z_min, z_cut)
        p.icp.coverage_min_z = max(p.icp.coverage_min_z, z_cut)
        self.searcher = CorrelativeSearch(template, p.search)
        self.nn = (NearestTemplateGrid(template.points, p.nn_resolution,
                                       margin=max(p.icp.max_corr_dists) + 0.05)
                   if p.nn_grid else None)
        self._rng = np.random.default_rng(0)
        self.last_scene = None
        self.reset()

    # ------------------------------------------------------------------ cân bằng sàn
    def reset(self) -> None:
        self._lev = np.zeros(3)          # (rx, ry, dz): phép quay nghiêng nhỏ nhất + dịch z
        self._lev_valid = False
        self._frame = 0

    @property
    def leveling_T(self) -> np.ndarray:
        return make_T(so3_exp((self._lev[0], self._lev[1], 0.0)), (0.0, 0.0, self._lev[2]))

    def floor_info(self) -> dict:
        tilt = math.degrees(float(np.linalg.norm(self._lev[:2])))
        return {"valid": self._lev_valid, "tilt_deg": tilt, "dz": float(self._lev[2])}

    def _update_leveling(self, P) -> bool:
        p = self.params
        ylim = max(abs(p.roi_min[1]), abs(p.roi_max[1]))
        m = ((np.abs(P[:, 2] - p.floor_z) < p.floor_band) & (P[:, 0] > p.roi_min[0])
             & (P[:, 0] < p.roi_max[0]) & (np.abs(P[:, 1]) < ylim))
        band = P[m]
        if len(band) < p.floor_min_inliers:
            return False
        if len(band) > 4000:
            band = band[self._rng.choice(len(band), 4000, replace=False)]
        fit = fit_plane_ransac(band, p.floor_ransac_thresh, iterations=60, rng=self._rng)
        if fit is None or int(fit.inliers.sum()) < p.floor_min_inliers:
            return False
        n = fit.normal
        tilt = math.degrees(math.acos(min(1.0, max(-1.0, float(n[2])))))
        if tilt > p.floor_max_tilt_deg or abs(-fit.d - p.floor_z) > p.floor_max_dz:
            return False
        axis = np.cross(n, [0.0, 0.0, 1.0])
        s = float(np.linalg.norm(axis))
        rv = axis / s * math.atan2(s, float(n[2])) if s > 1e-12 else np.zeros(3)
        dz = leveling_transform(n, fit.d, p.floor_z)[2, 3]
        new = np.array([rv[0], rv[1], dz])
        a = p.floor_ema
        self._lev = new if not self._lev_valid else (1.0 - a) * self._lev + a * new
        self._lev_valid = True
        return True

    def preprocess(self, points_base) -> np.ndarray:
        """Cân bằng sàn + cắt ROI (xóa sàn) + voxel. Trả (M,3) float64."""
        p = self.params
        P = finite_points(np.asarray(points_base, dtype=np.float32).reshape(-1, 3))
        if len(P) == 0:
            return np.zeros((0, 3))
        if p.floor_leveling and self._frame % max(1, p.floor_update_every) == 0:
            self._update_leveling(P)
        self._frame += 1
        if self._lev_valid:
            P = transform_points(self.leveling_T, P)
        lo = (p.roi_min[0], p.roi_min[1], p.floor_z + p.roi_min[2])
        hi = (p.roi_max[0], p.roi_max[1], p.floor_z + p.roi_max[2])
        Q, _ = voxel_downsample(P[crop_box_mask(P, lo, hi)], p.voxel_size, p.min_points_per_voxel)
        return Q.astype(np.float64)

    # ------------------------------------------------------------------ đánh giá
    def _judge(self, r):
        q = self.params.quality
        if r.coverage < q.min_coverage or r.inlier_ratio < q.min_inlier_ratio:
            return Status.LOW_QUALITY, (f"Khớp kém: coverage={r.coverage:.2f}, "
                                        f"inlier={r.inlier_ratio:.2f}")
        if r.rmse > q.max_rmse:
            return Status.LOW_QUALITY, f"RMSE lớn: {100 * r.rmse:.1f} cm"
        if r.degenerate:
            return Status.DEGENERATE, (f"Suy biến (observability={r.observability:.4f}): "
                                       "vật thiếu đặc trưng ngang/hướng")
        if r.sigma_xy > q.max_sigma_xy or r.sigma_yaw > math.radians(q.max_sigma_yaw_deg):
            return Status.LOW_QUALITY, (f"Bất định lớn: σxy={100 * r.sigma_xy:.1f} cm, "
                                        f"σyaw={math.degrees(r.sigma_yaw):.2f}°")
        return Status.OK, ""

    def _distinct(self, Ta, Tb) -> bool:
        q = self.params.quality
        xa, ya, ta = pose2d(Ta)
        xb, yb, tb = pose2d(Tb)
        return (math.hypot(xa - xb, ya - yb) > q.ambiguity_dist
                or abs(math.degrees(wrap_angle(ta - tb))) > q.ambiguity_yaw_deg)

    def _jump(self, T, T0) -> bool:
        q = self.params.quality
        x, y, t = pose2d(T)
        x0, y0, t0 = pose2d(T0)
        return (math.hypot(x - x0, y - y0) > q.track_jump_xy
                or abs(math.degrees(wrap_angle(t - t0))) > q.track_jump_yaw_deg)

    @staticmethod
    def _quality_dict(r) -> dict:
        return {"coverage": r.coverage, "inlier_ratio": r.inlier_ratio, "rmse": r.rmse,
                "sigma_xy": r.sigma_xy, "sigma_yaw_deg": math.degrees(r.sigma_yaw),
                "observability": r.observability, "score": r.score, "iterations": r.iterations}

    def _icp(self, S, T0, cam):
        return icp_point_to_plane(S, self.template, T0, self.params.icp, nn=self.nn,
                                  camera_pos_base=cam)

    # ------------------------------------------------------------------ xử lý 1 khung
    def process(self, points_base, camera_pos_base=None, prior_T_base_obj=None) -> Detection:
        p = self.params
        tm = {}
        t0 = time.perf_counter()
        S = self.preprocess(points_base)
        self.last_scene = S                       # để node phát debug ~/roi_cloud
        tm["preprocess"] = 1000 * (time.perf_counter() - t0)
        det = Detection(Status.NO_POINTS, n_points=len(S), timings=tm, floor=self.floor_info())

        def done(d):
            tm["total"] = 1000 * (time.perf_counter() - t0)
            return d

        if len(S) < p.min_scene_points:
            det.message = f"Quá ít điểm trong ROI ({len(S)})"
            return done(det)
        cam = None
        if camera_pos_base is not None:
            cam = np.asarray(camera_pos_base, dtype=float)
            if self._lev_valid:
                cam = transform_points(self.leveling_T, cam.reshape(1, 3))[0]

        # --- 1) chế độ bám: ICP từ pose dự đoán
        if prior_T_base_obj is not None:
            Tp = np.asarray(prior_T_base_obj, dtype=float)
            if self._lev_valid:
                Tp = self.leveling_T @ Tp
            x, y, yaw = pose2d(Tp)
            T0 = T_from_xyz_yaw(x, y, p.floor_z, yaw)
            t1 = time.perf_counter()
            r = self._icp(S, T0, cam)
            tm["icp"] = 1000 * (time.perf_counter() - t1)
            status, msg = self._judge(r)
            if status == Status.OK and not self._jump(r.T_base_obj, T0):
                return done(Detection(Status.OK, r.T_base_obj, r.cov_xyyaw, self._quality_dict(r),
                                      tm, len(S), "", "track", self.floor_info()))
            # bám thất bại / nhảy cóc -> tìm kiếm lại toàn cục ngay trong khung này

        # --- 2) chế độ tìm kiếm toàn cục
        t1 = time.perf_counter()
        hyps = self.searcher.search(S, p.roi_min[:2], p.roi_max[:2])
        tm["search"] = 1000 * (time.perf_counter() - t1)
        det.mode = "search"
        if not hyps:
            det.status, det.message = Status.NOT_FOUND, "Không tìm thấy vật trong ROI"
            return done(det)
        t1 = time.perf_counter()
        results = [self._icp(S, T_from_xyz_yaw(h.x, h.y, p.floor_z, h.yaw), cam) for h in hyps]
        tm["icp"] = 1000 * (time.perf_counter() - t1)
        results.sort(key=lambda r: r.score, reverse=True)
        best = results[0]
        status, msg = self._judge(best)
        det = Detection(status, best.T_base_obj, best.cov_xyyaw, self._quality_dict(best), tm,
                        len(S), msg, "search", self.floor_info())
        if status != Status.OK:
            return done(det)
        q = p.quality
        for r in results[1:]:
            if (self._judge(r)[0] == Status.OK and r.score >= q.ambiguity_ratio * best.score
                    and self._distinct(r.T_base_obj, best.T_base_obj)):
                x, y, yaw = pose2d(r.T_base_obj)
                det.status = Status.AMBIGUOUS
                det.message = (f"Mơ hồ: có vật thứ hai giống (x={x:.2f}, y={y:.2f}, "
                               f"yaw={math.degrees(yaw):.1f}°) — thu hẹp ROI hoặc dùng pose kỳ vọng")
                break
        return done(det)
