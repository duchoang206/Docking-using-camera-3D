"""Task 12 — node docking_controller (tích hợp rclpy; cần ROS 2)"""
import time

import pytest

rclpy = pytest.importorskip("rclpy")
from geometry_msgs.msg import PoseStamped, TransformStamped, Twist  # noqa: E402
from rclpy.executors import SingleThreadedExecutor  # noqa: E402
from rclpy.qos import DurabilityPolicy, QoSProfile  # noqa: E402
from std_msgs.msg import String  # noqa: E402
from std_srvs.srv import Trigger  # noqa: E402
from tf2_ros import StaticTransformBroadcaster  # noqa: E402

from camera3d_docking.ros.docking_controller import DockingControllerNode  # noqa: E402


@pytest.fixture(scope="module", autouse=True)
def ros_ctx():
    rclpy.init()
    yield
    rclpy.shutdown()


def test_controller_node_drives_towards_pose():
    node = DockingControllerNode()
    helper = rclpy.create_node("ctrl_test_helper")
    cmds, states = [], []
    helper.create_subscription(Twist, "cmd_vel", cmds.append, 10)
    helper.create_subscription(String, "docking_controller/state", lambda m: states.append(m.data),
                               QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    pose_pub = helper.create_publisher(PoseStamped, "detected_dock_pose", 10)
    tf = TransformStamped()
    tf.header.frame_id, tf.child_frame_id = "odom", "base_link"
    tf.transform.rotation.w = 1.0
    StaticTransformBroadcaster(helper).sendTransform(tf)
    client = helper.create_client(Trigger, "docking_controller/start")
    ex = SingleThreadedExecutor()
    ex.add_node(node)
    ex.add_node(helper)
    try:
        def publish_pose():
            m = PoseStamped()
            m.header.frame_id = "odom"
            m.header.stamp = helper.get_clock().now().to_msg()
            m.pose.position.x = 1.4            # mặt vật cách 1.4 m; đích = 1.4 + dock_offset_x
            m.pose.orientation.w = 1.0
            pose_pub.publish(m)

        assert client.wait_for_service(timeout_sec=5.0)
        publish_pose()
        fut = client.call_async(Trigger.Request())
        end = time.monotonic() + 3.0
        while not fut.done() and time.monotonic() < end:
            ex.spin_once(timeout_sec=0.02)
        assert fut.done() and fut.result().success
        end = time.monotonic() + 1.5
        nxt = 0.0
        while time.monotonic() < end:
            if time.monotonic() >= nxt:
                publish_pose()
                nxt = time.monotonic() + 0.1
            ex.spin_once(timeout_sec=0.02)
        assert any(c.linear.x > 0.05 for c in cmds), "robot không được lệnh tiến"
        assert states and states[-1].startswith("APPROACH")
    finally:
        ex.shutdown()
        node.destroy_node()
        helper.destroy_node()
