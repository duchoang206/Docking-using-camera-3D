"""Node `cloud_saver`: lưu N đám mây điểm kế tiếp (đã đổi về base_frame) thành file .pcd.

Dùng để quét template thật:
  1) Đặt robot trước vật tại pose đã đo (thước / AprilTag), vật thẳng hàng.
  2) ros2 run camera3d_docking cloud_saver --ros-args -r points:=/camera/points -p count:=5
  3) ros2 run camera3d_docking make_template from-cloud cloud_*.pcd --object-pose X Y YAW -o vat.npz
"""
from __future__ import annotations

import os

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2
from tf2_ros import Buffer, TransformException

from ..core.fileio import write_pcd
from ..core.geometry import transform_points
from .conversions import pointcloud2_to_xyz, start_tf_listener, stop_tf_listener, transform_to_T

DEFAULTS = {"base_frame": "base_link", "count": 5, "output_dir": ".", "prefix": "cloud", "stride": 1}


class CloudSaverNode(Node):
    def __init__(self, **kwargs):
        super().__init__("cloud_saver", **kwargs)
        for name, value in DEFAULTS.items():
            self.declare_parameter(name, value)
        self.P = {k: self.get_parameter(k).value for k in DEFAULTS}
        os.makedirs(self.P["output_dir"], exist_ok=True)
        self.saved = []
        self.done = False
        self.tf_buffer = Buffer()
        self.tf_listener = start_tf_listener(self.tf_buffer)
        qos = QoSProfile(depth=1, history=HistoryPolicy.KEEP_LAST,
                         reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(PointCloud2, "points", self._on_cloud, qos)
        self.get_logger().info(f"Sẽ lưu {self.P['count']} cloud vào '{self.P['output_dir']}'")

    def destroy_node(self):
        stop_tf_listener(self.tf_listener)  # dừng luồng TF trước khi hủy node
        return super().destroy_node()

    def _on_cloud(self, msg: PointCloud2):
        if self.done:
            return
        try:
            tf = self.tf_buffer.lookup_transform(self.P["base_frame"], msg.header.frame_id,
                                                 Time.from_msg(msg.header.stamp),
                                                 timeout=Duration(seconds=0.2))
        except TransformException as ex:
            self.get_logger().warn(f"Chưa có TF: {ex}", throttle_duration_sec=2.0)
            return
        P = transform_points(transform_to_T(tf.transform),
                             pointcloud2_to_xyz(msg, int(self.P["stride"])))
        path = os.path.join(self.P["output_dir"], f"{self.P['prefix']}_{len(self.saved):03d}.pcd")
        write_pcd(path, P)
        self.saved.append(path)
        self.get_logger().info(f"Đã lưu {path} ({len(P)} điểm)")
        if len(self.saved) >= int(self.P["count"]):
            self.done = True
            self.get_logger().info("Xong. Tiếp theo: make_template from-cloud <các file> "
                                   "--object-pose X Y YAW_DEG -o vat.npz")


def main(args=None):
    rclpy.init(args=args)
    node = CloudSaverNode()
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
