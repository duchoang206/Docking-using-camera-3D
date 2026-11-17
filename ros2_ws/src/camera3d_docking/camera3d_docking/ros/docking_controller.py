"""Node ROS 2 `docking_controller`: điều khiển docking cho robot KHÔNG chạy Nav2.

detected_dock_pose (frame bất kỳ, thường odom) --TF--> đích cuối trong base_link
    = pose vật ⊕ (dock_offset_x, dock_offset_y, dock_offset_yaw [+180° nếu lùi vào])
-> core.control.DockingFSM (luật Park–Kuipers + vùng mù + thử lại) -> cmd_vel.
Gọi ~/start để bắt đầu, ~/cancel để dừng. Trạng thái phát trên ~/state.
"""
from __future__ import annotations

import math

import rclpy
from geometry_msgs.msg import PoseStamped, Twist, TwistStamped
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from rclpy.time import Time
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformException

from ..core.control import ControlLawParams, DockingFSM, DockState, FsmParams
from ..core.geometry import T_from_xyz_yaw, pose2d
from .conversions import pose_to_T, start_tf_listener, stop_tf_listener, transform_to_T

DEFAULTS = {
    "base_frame": "base_link", "control_rate_hz": 20.0, "use_stamped_cmd_vel": False,
    "dock_offset_x": -0.35, "dock_offset_y": 0.0, "dock_offset_yaw_deg": 0.0, "backward": False,
    "law.k_phi": 3.0, "law.k_delta": 2.0, "law.beta": 0.4, "law.lambda": 2.0,
    "law.v_max": 0.2, "law.v_min": 0.02, "law.w_max": 0.6, "law.slowdown_radius": 0.25,
    "fsm.goal_tol_x": 0.01, "fsm.blind_zone": 0.6, "fsm.blind_speed": 0.08,
    "fsm.detection_timeout": 5.0, "fsm.lost_timeout": 1.0, "fsm.approach_timeout": 60.0,
    "fsm.max_retries": 2, "fsm.retreat_distance": 0.5, "fsm.retreat_speed": 0.1,
    "fsm.acc_v": 0.5, "fsm.acc_w": 1.5,
}
_TERMINAL = (DockState.IDLE, DockState.DOCKED, DockState.FAILED)


class DockingControllerNode(Node):
    def __init__(self, **kwargs):
        super().__init__("docking_controller", **kwargs)
        for name, value in DEFAULTS.items():
            self.declare_parameter(name, value)
        P = {k: self.get_parameter(k).value for k in DEFAULTS}
        self.P = P
        law = ControlLawParams(k_phi=P["law.k_phi"], k_delta=P["law.k_delta"], beta=P["law.beta"],
                               lam=P["law.lambda"], v_max=P["law.v_max"], v_min=P["law.v_min"],
                               w_max=P["law.w_max"], slowdown_radius=P["law.slowdown_radius"])
        fsm = FsmParams(**{k.split(".", 1)[1]: P[k] for k in P if k.startswith("fsm.")},
                        backward=P["backward"])
        self.fsm = DockingFSM(law, fsm)
        yaw = math.radians(P["dock_offset_yaw_deg"]) + (math.pi if P["backward"] else 0.0)
        self.T_obj_target = T_from_xyz_yaw(P["dock_offset_x"], P["dock_offset_y"], 0.0, yaw)
        self.dock_msg = None
        self.contact = False
        self.tf_buffer = Buffer()
        self.tf_listener = start_tf_listener(self.tf_buffer)
        self.create_subscription(PoseStamped, "detected_dock_pose", self._on_pose, 10)
        self.create_subscription(Bool, "~/contact", lambda m: setattr(self, "contact", bool(m.data)), 10)
        msg_type = TwistStamped if P["use_stamped_cmd_vel"] else Twist
        self.pub_cmd = self.create_publisher(msg_type, "cmd_vel", 10)
        self.pub_state = self.create_publisher(
            String, "~/state", QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.create_service(Trigger, "~/start", self._on_start)
        self.create_service(Trigger, "~/cancel", self._on_cancel)
        self.create_timer(1.0 / P["control_rate_hz"], self._on_timer)
        self._publish_state()
        self.get_logger().info("Bộ điều khiển docking sẵn sàng — gọi ~/start để bắt đầu")

    def destroy_node(self):
        stop_tf_listener(self.tf_listener)  # dừng luồng TF trước khi hủy node
        return super().destroy_node()

    # ------------------------------------------------------------------ giao tiếp
    def _on_pose(self, msg: PoseStamped):
        self.dock_msg = msg

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _publish_state(self):
        self.pub_state.publish(String(data=f"{self.fsm.state.value}: {self.fsm.message}"))

    def _send(self, v: float, w: float):
        if self.P["use_stamped_cmd_vel"]:
            msg = TwistStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = self.P["base_frame"]
            msg.twist.linear.x, msg.twist.angular.z = float(v), float(w)
        else:
            msg = Twist()
            msg.linear.x, msg.angular.z = float(v), float(w)
        self.pub_cmd.publish(msg)

    def _on_start(self, _req, res):
        self.contact = False
        self.fsm.start(self._now())
        self._publish_state()
        res.success, res.message = True, "Bắt đầu docking"
        return res

    def _on_cancel(self, _req, res):
        self.fsm.cancel()
        self._send(0.0, 0.0)
        self._publish_state()
        res.success, res.message = True, "Đã hủy docking"
        return res

    # ------------------------------------------------------------------ vòng điều khiển
    def _target(self):
        """(đích (x,y,yaw) trong base_link, tuổi phép đo) hoặc (None, None)."""
        m = self.dock_msg
        if m is None:
            return None, None
        try:
            tf = self.tf_buffer.lookup_transform(self.P["base_frame"], m.header.frame_id, Time())
        except TransformException as ex:
            self.get_logger().warn(f"Không tra được TF tới đích: {ex}", throttle_duration_sec=2.0)
            return None, None
        T = transform_to_T(tf.transform) @ pose_to_T(m.pose) @ self.T_obj_target
        age = (self.get_clock().now() - Time.from_msg(m.header.stamp)).nanoseconds * 1e-9
        return pose2d(T), age

    def _on_timer(self):
        if self.fsm.state == DockState.IDLE:
            return
        prev = self.fsm.state
        target, age = self._target()
        v, w = self.fsm.step(self._now(), target, age, self.contact)
        if self.fsm.state != prev:
            self._publish_state()
            self.get_logger().info(f"[{prev.value} -> {self.fsm.state.value}] {self.fsm.message}")
        if self.fsm.state in _TERMINAL:
            if prev not in _TERMINAL:
                self._send(0.0, 0.0)
            return
        self._send(v, w)


def main(args=None):
    rclpy.init(args=args)
    node = DockingControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
