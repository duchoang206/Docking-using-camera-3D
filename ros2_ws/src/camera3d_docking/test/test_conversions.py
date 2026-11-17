"""Task 10 — ros/conversions.py (cần môi trường ROS 2; tự bỏ qua nếu chưa source ROS)"""
import numpy as np
import pytest

pytest.importorskip("rclpy")
from builtin_interfaces.msg import Time  # noqa: E402
from sensor_msgs.msg import PointField  # noqa: E402
from sensor_msgs_py import point_cloud2 as pc2  # noqa: E402
from std_msgs.msg import Header  # noqa: E402

from camera3d_docking.core import geometry as g  # noqa: E402
from camera3d_docking.ros import conversions as cv  # noqa: E402


def test_pc2_roundtrip():
    P = np.random.default_rng(0).normal(size=(50, 3)).astype(np.float32)
    msg = cv.xyz_to_pointcloud2(P, "cam", Time(sec=1))
    assert msg.header.frame_id == "cam" and msg.width * msg.height == 50
    assert np.allclose(cv.pointcloud2_to_xyz(msg), P)


def test_pc2_organized_stride():
    H, W = 4, 6
    grid = np.arange(H * W * 3, dtype=np.float32).reshape(H, W, 3)
    msg = cv.xyz_to_pointcloud2(grid, "cam", Time(), organized=True)
    assert (msg.height, msg.width) == (H, W)
    out = cv.pointcloud2_to_xyz(msg, stride=2)
    assert np.allclose(out, grid[::2, ::2].reshape(-1, 3))


def test_pc2_skips_nan():
    P = np.array([[1, 2, 3], [np.nan, 0, 0], [4, 5, 6]], dtype=np.float32)
    assert cv.pointcloud2_to_xyz(cv.xyz_to_pointcloud2(P, "c", Time())).shape == (2, 3)


def test_pc2_xyzrgb_padding():
    fields = [PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
              PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
              PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
              PointField(name="rgb", offset=16, datatype=PointField.FLOAT32, count=1)]
    pts = [(1.0, 2.0, 3.0, 0.5), (4.0, 5.0, 6.0, 0.25)]
    msg = pc2.create_cloud(Header(frame_id="c"), fields, pts)
    msg.point_step = 32  # có padding
    raw = np.zeros((2, 32), dtype=np.uint8)
    for i, p in enumerate(pts):
        raw[i, :12] = np.frombuffer(np.array(p[:3], np.float32).tobytes(), np.uint8)
    msg.data = raw.tobytes()
    msg.row_step = 64
    assert np.allclose(cv.pointcloud2_to_xyz(msg), [[1, 2, 3], [4, 5, 6]])


def test_pose_roundtrip():
    T = g.T_from_xyz_rpy(1.0, -2.0, 0.5, 0.1, 0.2, -1.0)
    assert np.allclose(cv.pose_to_T(cv.T_to_pose(T)), T, atol=1e-9)
