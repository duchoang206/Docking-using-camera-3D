"""Node ROS 2 `pose_estimator`: PointCloud2 -> pose vật cần dock -> `detected_dock_pose`.

Lớp vỏ mỏng quanh core.estimator.DockPoseEstimator + core.kalman.StaticPoseKF:
- Mỗi cloud (giới hạn max_rate_hz, chỉ xử lý khung MỚI NHẤT): TF tại stamp của cloud
  -> estimator -> nếu OK: đổi sang odom, cập nhật Kalman -> phát pose (stamp của cloud).
- Timer: phát lại pose Kalman (stamp = now) khi mới mất dấu < hold_timeout hoặc robot
  đang trong vùng mù (quá gần vật) -> Nav2/bộ điều khiển không báo "mất phát hiện".
"""
from __future__ import annotations

import json
import math
import os
import time

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import String
from std_srvs.srv import SetBool, Trigger
from tf2_ros import Buffer, TransformException

from ..core.estimator import DockPoseEstimator, EstimatorParams, QualityParams, Status
from ..core.geometry import T_from_xyz_yaw, inv_T, pose2d, transform_points, yaw_of
from ..core.kalman import StaticPoseKF
from ..core.registration import IcpParams
from ..core.search import SearchParams
from ..core.template import Template
from .conversions import (T_to_pose, pointcloud2_to_xyz, start_tf_listener, stop_tf_listener,
                          transform_to_T, xyz_to_pointcloud2)

DEFAULTS = {
    "base_frame": "base_link", "fixed_frame": "odom", "template_path": "euro_pallet.npz",
    "cloud_stride": 2, "max_rate_hz": 10.0, "publish_rate_hz": 10.0, "hold_timeout": 2.0,
    "blind_zone_distance": 0.6, "start_enabled": True, "use_expected_pose": False,
    "expected_pose": [1.2, 0.0, 0.0],
    "roi.min": [0.25, -1.5, 0.03], "roi.max": [3.5, 1.5, 2.0], "floor_z": 0.0,
    "floor_leveling": True, "voxel_size": 0.02, "min_points_per_voxel": 1, "nn_grid": True,
    "search.grid_res": 0.03, "search.yaw_range_deg": 45.0, "search.yaw_step_deg": 3.0,
    "search.n_bands": 3, "search.free_weight": 0.5,
    "icp.dof": "xyz_yaw", "icp.max_corr_dists": [0.15, 0.06, 0.03], "icp.min_observability": 0.008,
    "quality.min_coverage": 0.35, "quality.min_inlier_ratio": 0.5, "quality.max_rmse": 0.015,
    "quality.max_sigma_xy": 0.02, "quality.max_sigma_yaw_deg": 1.5, "quality.ambiguity_ratio": 0.9,
    "kalman.q_xy": 0.005, "kalman.q_yaw_deg": 0.2, "kalman.gate_chi2": 14.16,
    "kalman.r_min_xy": 0.005, "kalman.r_min_yaw_deg": 0.3,
    "debug.publish_aligned_template": True, "debug.publish_roi_cloud": False,
}


def resolve_template(path: str) -> str:
    """Đường dẫn tuyệt đối/tương đối, hoặc tên file trong share/camera3d_docking/templates."""
    if os.path.isfile(path):
        return path
    try:
        from ament_index_python.packages import get_package_share_directory

        cand = os.path.join(get_package_share_directory("camera3d_docking"), "templates", path)
        if os.path.isfile(cand):
            return cand
    except Exception:  # noqa: BLE001 - gói chưa cài: chỉ dùng đường dẫn trực tiếp
        pass
    raise FileNotFoundError(f"Không tìm thấy template: {path}")


class PoseEstimatorNode(Node):
    def __init__(self, **kwargs):
        super().__init__("pose_estimator", **kwargs)
        for name, value in DEFAULTS.items():
            self.declare_parameter(name, value)
        P = {k: self.get_parameter(k).value for k in DEFAULTS}
        self.P = P
        self.base_frame, self.fixed_frame = P["base_frame"], P["fixed_frame"]
        self.template = Template.load(resolve_template(P["template_path"]))
        ep = EstimatorParams(
            roi_min=tuple(P["roi.min"]), roi_max=tuple(P["roi.max"]), floor_z=P["floor_z"],
            floor_leveling=P["floor_leveling"], voxel_size=P["voxel_size"],
            min_points_per_voxel=P["min_points_per_voxel"], nn_grid=P["nn_grid"],
            search=SearchParams(grid_res=P["search.grid_res"], yaw_range_deg=P["search.yaw_range_deg"],
                                yaw_step_deg=P["search.yaw_step_deg"], n_bands=P["search.n_bands"],
                                free_weight=P["search.free_weight"]),
            icp=IcpParams(dof=P["icp.dof"], max_corr_dists=tuple(P["icp.max_corr_dists"]),
                          min_observability=P["icp.min_observability"]),
            quality=QualityParams(min_coverage=P["quality.min_coverage"],
                                  min_inlier_ratio=P["quality.min_inlier_ratio"],
                                  max_rmse=P["quality.max_rmse"],
                                  max_sigma_xy=P["quality.max_sigma_xy"],
                                  max_sigma_yaw_deg=P["quality.max_sigma_yaw_deg"],
                                  ambiguity_ratio=P["quality.ambiguity_ratio"]))
        t0 = time.perf_counter()
        self.estimator = DockPoseEstimator(self.template, ep)
        self.kf = StaticPoseKF(q_xy=P["kalman.q_xy"], q_yaw=math.radians(P["kalman.q_yaw_deg"]),
                               gate_chi2=P["kalman.gate_chi2"])
        self.r_min = np.diag([P["kalman.r_min_xy"] ** 2, P["kalman.r_min_xy"] ** 2,
                              math.radians(P["kalman.r_min_yaw_deg"]) ** 2])
        self.enabled = bool(P["start_enabled"])
        self._last_proc = 0.0
        self._last_accept = None
        self._last_tpl = 0.0
        self.z_obj = 0.0
        self.tf_buffer = Buffer()
        self.tf_listener = start_tf_listener(self.tf_buffer)
        qos = QoSProfile(depth=1, history=HistoryPolicy.KEEP_LAST,
                         reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(PointCloud2, "points", self._on_cloud, qos)
        self.pub_pose = self.create_publisher(PoseStamped, "detected_dock_pose", 10)
        self.pub_raw = self.create_publisher(PoseWithCovarianceStamped, "~/pose_raw", 10)
        self.pub_tpl = self.create_publisher(PointCloud2, "~/aligned_template", 1)
        self.pub_roi = self.create_publisher(PointCloud2, "~/roi_cloud", 1)
        self.pub_status = self.create_publisher(String, "~/status", 10)
        self.create_service(Trigger, "~/reset", self._on_reset)
        self.create_service(SetBool, "~/enable", self._on_enable)
        self.create_timer(1.0 / P["publish_rate_hz"], self._on_timer)
        self.get_logger().info(
            f"Template '{self.template.name}' ({len(self.template.points)} điểm) sẵn sàng sau "
            f"{time.perf_counter() - t0:.1f} s — chờ PointCloud2 trên 'points'")

    def destroy_node(self):
        stop_tf_listener(self.tf_listener)  # dừng luồng TF trước khi hủy node
        return super().destroy_node()

    # ------------------------------------------------------------------ services
    def _on_reset(self, _req, res):
        self.kf.reset()
        self.estimator.reset()
        self._last_accept = None
        res.success, res.message = True, "Đã reset bộ lọc và cân bằng sàn"
        return res

    def _on_enable(self, req, res):
        self.enabled = bool(req.data)
        res.success, res.message = True, "Bật" if self.enabled else "Tắt (tiết kiệm CPU)"
        return res

    # ------------------------------------------------------------------ tiện ích
    def _T_odom_obj(self):
        x, y, yaw = self.kf.x
        return T_from_xyz_yaw(x, y, self.z_obj, yaw)

    def _publish_pose(self, stamp_msg):
        msg = PoseStamped()
        msg.header.frame_id = self.fixed_frame
        msg.header.stamp = stamp_msg
        msg.pose = T_to_pose(self._T_odom_obj())
        self.pub_pose.publish(msg)

    def _distance_to_object(self) -> float:
        try:
            tf = self.tf_buffer.lookup_transform(self.fixed_frame, self.base_frame, Time())
        except TransformException:
            return float("inf")
        T = transform_to_T(tf.transform)
        return math.hypot(self.kf.x[0] - T[0, 3], self.kf.x[1] - T[1, 3])

    # ------------------------------------------------------------------ xử lý cloud
    def _on_cloud(self, msg: PointCloud2):
        if not self.enabled:
            return
        t_now = time.monotonic()
        if t_now - self._last_proc < 1.0 / self.P["max_rate_hz"]:
            return
        self._last_proc = t_now
        stamp = Time.from_msg(msg.header.stamp)
        try:
            tf_bc = self.tf_buffer.lookup_transform(self.base_frame, msg.header.frame_id, stamp,
                                                    timeout=Duration(seconds=0.1))
            tf_ob = self.tf_buffer.lookup_transform(self.fixed_frame, self.base_frame, stamp,
                                                    timeout=Duration(seconds=0.1))
        except TransformException as ex:
            self.get_logger().warn(f"Chưa tra được TF — bỏ khung: {ex}", throttle_duration_sec=2.0)
            return
        T_bc = transform_to_T(tf_bc.transform)
        T_ob = transform_to_T(tf_ob.transform)
        if self.kf.initialized and self._distance_to_object() < self.P["blind_zone_distance"]:
            self._status({"status": "BLIND_HOLD", "message": "Vùng mù — giữ pose Kalman"})
            return
        prior = None
        if self.kf.initialized:
            prior = inv_T(T_ob) @ self._T_odom_obj()
        elif self.P["use_expected_pose"]:
            ex, ey, eyaw = self.P["expected_pose"]
            prior = T_from_xyz_yaw(ex, ey, 0.0, math.radians(eyaw))
        P = transform_points(T_bc, pointcloud2_to_xyz(msg, int(self.P["cloud_stride"])))
        det = self.estimator.process(P, T_bc[:3, 3], prior)
        accepted = False
        if det.status == Status.OK:
            T_odom_obj = T_ob @ det.T_base_obj
            x, y, yaw = pose2d(T_odom_obj)
            c, s = math.cos(yaw_of(T_ob)), math.sin(yaw_of(T_ob))
            Rz = np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])
            R = Rz @ det.cov_xyyaw @ Rz.T + self.r_min
            accepted = self.kf.update((x, y, yaw), R, stamp.nanoseconds * 1e-9)
            if accepted:
                self.z_obj = 0.8 * self.z_obj + 0.2 * float(T_odom_obj[2, 3])
                self._last_accept = stamp
                self._publish_pose(msg.header.stamp)
        self._publish_debug(det, msg.header.stamp, t_now)
        self._status({"status": det.status.value, "message": det.message, "mode": det.mode,
                      "accepted": accepted, "n_points": det.n_points,
                      "quality": {k: round(float(v), 4) for k, v in det.quality.items()},
                      "timings_ms": {k: round(v, 1) for k, v in det.timings.items()},
                      "floor": det.floor})

    def _publish_debug(self, det, stamp_msg, t_now):
        if det.T_base_obj is None:
            return
        raw = PoseWithCovarianceStamped()
        raw.header.frame_id, raw.header.stamp = self.base_frame, stamp_msg
        raw.pose.pose = T_to_pose(det.T_base_obj)
        cov = np.diag([0.0, 0.0, 1e-4, 1e-4, 1e-4, 0.0])
        if det.cov_xyyaw is not None:
            idx = [0, 1, 5]
            for i in range(3):
                for j in range(3):
                    cov[idx[i], idx[j]] = det.cov_xyyaw[i, j]
        raw.pose.covariance = [float(v) for v in cov.ravel()]
        self.pub_raw.publish(raw)
        if self.P["debug.publish_aligned_template"] and t_now - self._last_tpl > 0.5:
            self._last_tpl = t_now
            pts = self.template.points[:: max(1, len(self.template.points) // 4000)]
            self.pub_tpl.publish(xyz_to_pointcloud2(transform_points(det.T_base_obj, pts),
                                                    self.base_frame, stamp_msg))
        if self.P["debug.publish_roi_cloud"] and self.estimator.last_scene is not None:
            self.pub_roi.publish(xyz_to_pointcloud2(self.estimator.last_scene, self.base_frame,
                                                    stamp_msg))

    def _status(self, d: dict):
        if self.kf.initialized:
            d["kalman"] = {"x": round(float(self.kf.x[0]), 4), "y": round(float(self.kf.x[1]), 4),
                           "yaw_deg": round(math.degrees(self.kf.x[2]), 2),
                           "sigma_xy": round(math.sqrt(max(self.kf.P[0, 0], self.kf.P[1, 1])), 4)}
        self.pub_status.publish(String(data=json.dumps(d, ensure_ascii=False)))

    # ------------------------------------------------------------------ giữ pose
    def _on_timer(self):
        if not self.enabled or not self.kf.initialized or self._last_accept is None:
            return
        now = self.get_clock().now()
        age = (now - self._last_accept).nanoseconds * 1e-9
        if age < 0.5 / self.P["publish_rate_hz"]:
            return
        if age <= self.P["hold_timeout"] or self._distance_to_object() < self.P["blind_zone_distance"]:
            self._publish_pose(now.to_msg())


def main(args=None):
    rclpy.init(args=args)
    node = PoseEstimatorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
