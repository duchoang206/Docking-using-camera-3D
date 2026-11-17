#!/usr/bin/env python3
"""Ghi lại một lần docking trong Gazebo để vẽ hình cho sổ tay.

Mỗi dòng JSONL: {"k": loại, "t": giây từ lúc bắt đầu, ...}
  gt   : pose thật của robot trong world (x, y, yaw)
  st   : status của pose_estimator (status, mode, accepted, kalman, timings_ms)
  ctl  : trạng thái docking_controller
  cmd  : lệnh vận tốc (v, w)
  pose : detected_dock_pose (x, y, yaw) trong odom
Dừng sau khi controller báo DOCKED/FAILED thêm 3 s, hoặc khi hết thời gian.
"""
import json
import math
import sys
import time

import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import String
from tf2_msgs.msg import TFMessage


def yaw_q(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class Recorder(Node):
    def __init__(self, path):
        super().__init__("guide_recorder")
        self.out = open(path, "w", encoding="utf-8")
        self.t0 = time.monotonic()
        self.end_at = None
        self.last_gt = 0.0
        self.create_subscription(TFMessage, "/ground_truth/pose", self.on_gt, 10)
        self.create_subscription(String, "/pose_estimator/status", self.on_status, 10)
        self.create_subscription(String, "/docking_controller/state", self.on_state,
                                 QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.create_subscription(Twist, "/cmd_vel", self.on_cmd, 10)
        self.create_subscription(PoseStamped, "/detected_dock_pose", self.on_pose, 10)

    def w(self, kind, d):
        d["k"] = kind
        d["t"] = round(time.monotonic() - self.t0, 3)
        self.out.write(json.dumps(d, ensure_ascii=False) + "\n")
        self.out.flush()

    def on_gt(self, m):
        now = time.monotonic()
        if now - self.last_gt < 0.05:
            return
        for t in m.transforms:
            if "docking_bot" in t.child_frame_id:
                tr = t.transform
                self.last_gt = now
                self.w("gt", {"x": tr.translation.x, "y": tr.translation.y, "yaw": yaw_q(tr.rotation)})
                return

    def on_status(self, m):
        try:
            d = json.loads(m.data)
        except ValueError:
            d = {"raw": m.data}
        self.w("st", d)

    def on_state(self, m):
        self.w("ctl", {"state": m.data})
        if m.data.startswith(("DOCKED", "FAILED")) and self.end_at is None:
            self.end_at = time.monotonic() + 3.0

    def on_cmd(self, m):
        self.w("cmd", {"v": m.linear.x, "w": m.angular.z})

    def on_pose(self, m):
        p = m.pose
        self.w("pose", {"x": p.position.x, "y": p.position.y, "yaw": yaw_q(p.orientation)})


def main():
    path = sys.argv[1]
    timeout = float(sys.argv[2]) if len(sys.argv) > 2 else 150.0
    rclpy.init()
    node = Recorder(path)
    t_end = time.monotonic() + timeout
    try:
        while rclpy.ok() and time.monotonic() < t_end:
            rclpy.spin_once(node, timeout_sec=0.05)
            if node.end_at is not None and time.monotonic() > node.end_at:
                break
    finally:
        node.out.close()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
