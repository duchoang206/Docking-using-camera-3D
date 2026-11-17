"""Task 11 — node pose_estimator (tích hợp rclpy trong tiến trình; cần ROS 2)"""
import json
import math
import time

import numpy as np
import pytest

rclpy = pytest.importorskip("rclpy")
from geometry_msgs.msg import PoseStamped, TransformStamped  # noqa: E402
from rclpy.executors import SingleThreadedExecutor  # noqa: E402
from rclpy.parameter import Parameter  # noqa: E402
from sensor_msgs.msg import Image, PointCloud2  # noqa: E402
from std_msgs.msg import String  # noqa: E402
from tf2_ros import StaticTransformBroadcaster  # noqa: E402

from camera3d_docking.core import geometry as g  # noqa: E402
from camera3d_docking.core import shapes as s  # noqa: E402
from camera3d_docking.core import synthetic as syn  # noqa: E402
from camera3d_docking.core import template as tp  # noqa: E402
from camera3d_docking.ros import conversions as cv  # noqa: E402
from camera3d_docking.ros.pose_estimator import PoseEstimatorNode  # noqa: E402

TRUTH = (1.6, 0.1, 8.0)


@pytest.fixture(scope="module", autouse=True)
def ros_ctx():
    rclpy.init()
    yield
    rclpy.shutdown()


@pytest.fixture(scope="module")
def tpl_path(tmp_path_factory):
    p = tmp_path_factory.mktemp("tpl") / "pallet.npz"
    tp.template_from_boxes(s.euro_pallet_boxes(), "euro_pallet").save(p)
    return str(p)


@pytest.fixture(scope="module")
def scene():
    return syn.render_scene([(s.euro_pallet_boxes(), TRUTH[0], TRUTH[1], math.radians(TRUTH[2]))],
                            noise=s.DepthNoise(), rng=np.random.default_rng(0))


def _tf(parent, child, T, stamp):
    m = TransformStamped()
    m.header.frame_id, m.child_frame_id, m.header.stamp = parent, child, stamp
    pose = cv.T_to_pose(T)
    m.transform.translation.x = pose.position.x
    m.transform.translation.y = pose.position.y
    m.transform.translation.z = pose.position.z
    m.transform.rotation = pose.orientation
    return m


def _mask_msg(mask_u8, stamp):
    """Mặt nạ segmentation dạng sensor_msgs/Image mono8 (khác 0 = pallet)."""
    m = Image()
    m.header.frame_id, m.header.stamp = "camera_optical", stamp
    m.height, m.width = mask_u8.shape
    m.encoding, m.is_bigendian, m.step = "mono8", 0, mask_u8.shape[1]
    m.data = np.ascontiguousarray(mask_u8, dtype=np.uint8).tobytes()
    return m


class Harness:
    def __init__(self, tpl_path, scene, **params):
        over = [Parameter("template_path", value=tpl_path), Parameter("max_rate_hz", value=30.0)]
        over += [Parameter(k, value=v) for k, v in params.items()]
        self.node = PoseEstimatorNode(parameter_overrides=over)
        self.helper = rclpy.create_node("test_helper")
        self.poses = []
        self.helper.create_subscription(PoseStamped, "detected_dock_pose", self.poses.append, 10)
        self.pub = self.helper.create_publisher(PointCloud2, "points", 10)
        self.tfb = StaticTransformBroadcaster(self.helper)
        self.ex = SingleThreadedExecutor()
        self.ex.add_node(self.node)
        self.ex.add_node(self.helper)
        self.cloud = scene["organized_optical"]
        self.T_base_opt = scene["T_base_optical"]
        self.mask_pub = None              # đặt publisher + mặt nạ uint8 để phát cùng stamp với cloud
        self.mask = None
        self.status = []
        self.helper.create_subscription(String, "/pose_estimator/status",
                                        lambda m: self.status.append(json.loads(m.data)), 10)

    def send_tf(self):
        now = self.helper.get_clock().now().to_msg()
        self.tfb.sendTransform([_tf("base_link", "camera_optical", self.T_base_opt, now),
                                _tf("odom", "base_link", np.eye(4), now)])

    def spin(self, seconds, publish=True, until=None, period=0.1):
        end = time.monotonic() + seconds
        nxt = 0.0
        while time.monotonic() < end:
            if publish and time.monotonic() >= nxt:
                stamp = self.helper.get_clock().now().to_msg()
                if self.mask_pub is not None:
                    self.mask_pub.publish(_mask_msg(self.mask, stamp))
                self.pub.publish(cv.xyz_to_pointcloud2(self.cloud, "camera_optical", stamp,
                                                       organized=True))
                nxt = time.monotonic() + period
            self.ex.spin_once(timeout_sec=0.02)
            if until is not None and until():
                return True
        return until() if until is not None else True

    def close(self):
        self.ex.shutdown()
        self.node.destroy_node()
        self.helper.destroy_node()


def test_tf_listener_has_own_node_and_stops_cleanly(tpl_path):
    """TF listener phải dùng node nội bộ riêng (không giành executor của node chính) và
    luồng của nó phải dừng hẳn khi destroy_node()."""
    node = PoseEstimatorNode(parameter_overrides=[Parameter("template_path", value=tpl_path)])
    listener = node.tf_listener
    assert listener.node is not node
    node.destroy_node()
    assert not listener.dedicated_listener_thread.is_alive()


def test_node_publishes_detected_pose(tpl_path, scene):
    h = Harness(tpl_path, scene)
    try:
        h.send_tf()
        assert h.spin(15.0, until=lambda: len(h.poses) > 0), "không nhận được detected_dock_pose"
        msg = h.poses[-1]
        assert msg.header.frame_id == "odom"
        x, y, yaw = g.pose2d(cv.pose_to_T(msg.pose))
        assert math.hypot(x - TRUTH[0], y - TRUTH[1]) < 0.02
        assert abs(math.degrees(yaw) - TRUTH[2]) < 1.5
    finally:
        h.close()


def test_node_survives_missing_tf(tpl_path, scene):
    h = Harness(tpl_path, scene)
    try:
        h.spin(1.5)                       # có cloud nhưng chưa có TF -> không crash, không pose
        assert len(h.poses) == 0
        h.send_tf()
        assert h.spin(15.0, until=lambda: len(h.poses) > 0)
    finally:
        h.close()


def test_node_holds_pose_when_cloud_stops(tpl_path, scene):
    h = Harness(tpl_path, scene, hold_timeout=1.0)
    try:
        h.send_tf()
        assert h.spin(15.0, until=lambda: len(h.poses) > 0)
        n0 = len(h.poses)
        h.spin(0.8, publish=False)        # mất cloud -> vẫn phát pose giữ trong hold_timeout
        assert len(h.poses) > n0
        h.spin(1.0, publish=False)        # đã quá hold_timeout (ngoài vùng mù) -> ngừng phát
        n1 = len(h.poses)
        h.spin(0.6, publish=False)
        assert len(h.poses) == n1
    finally:
        h.close()


def test_node_uses_segmentation_mask(tpl_path, scene):
    h = Harness(tpl_path, scene, mask_topic="pallet_mask")
    try:
        h.mask_pub = h.helper.create_publisher(Image, "pallet_mask", 10)
        h.mask = (scene["labels"] == 0).astype(np.uint8) * 255
        h.send_tf()
        assert h.spin(15.0, until=lambda: len(h.poses) > 0 and any(
            st.get("mask") == "used" and st.get("accepted") for st in h.status)), h.status[-3:]
        x, y, _ = g.pose2d(cv.pose_to_T(h.poses[-1].pose))
        assert math.hypot(x - TRUTH[0], y - TRUTH[1]) < 0.02
    finally:
        h.close()


def test_node_without_mask_messages_falls_back_to_geometry(tpl_path, scene):
    h = Harness(tpl_path, scene, mask_topic="pallet_mask")    # bật mặt nạ nhưng không ai phát
    try:
        h.send_tf()
        assert h.spin(15.0, until=lambda: len(h.poses) > 0 and any("mask" in st for st in h.status))
        masks = {st["mask"] for st in h.status if "mask" in st}
        assert masks == {"no_mask"}, masks
    finally:
        h.close()
