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


def test_pc2_keep_nan_keeps_grid_order():
    """keep_nan=True: giữ đủ lưới (kể cả NaN) để ghép từng điểm với pixel của mặt nạ 2D."""
    grid = np.arange(4 * 6 * 3, dtype=np.float32).reshape(4, 6, 3)
    grid[1, 2] = np.nan
    msg = cv.xyz_to_pointcloud2(grid, "cam", Time(), organized=True)
    full = cv.pointcloud2_to_xyz(msg, keep_nan=True)
    assert full.shape == (24, 3) and np.isnan(full[1 * 6 + 2]).all()
    assert np.array_equal(cv.pointcloud2_to_xyz(msg, stride=2, keep_nan=True),
                          grid[::2, ::2].reshape(-1, 3))


def _image(arr, encoding, pad=0):
    from sensor_msgs.msg import Image

    a = np.ascontiguousarray(arr)
    h, w = a.shape[:2]
    raw = np.hstack([a.reshape(h, -1).view(np.uint8), np.zeros((h, pad), np.uint8)])
    return Image(height=h, width=w, encoding=encoding, is_bigendian=0, step=raw.shape[1],
                 data=raw.tobytes())


def test_image_to_mask_common_encodings():
    m8 = np.zeros((3, 5), np.uint8)
    m8[1, 2] = 255
    assert np.array_equal(cv.image_to_mask(_image(m8, "mono8", pad=3)), m8 > 0)   # hàng có đệm
    m16 = np.zeros((3, 5), np.uint16)
    m16[2, 4] = 7                                                          # mặt nạ kiểu nhãn
    assert np.array_equal(cv.image_to_mask(_image(m16, "16UC1")), m16 > 0)
    rgb = np.zeros((3, 5, 3), np.uint8)
    rgb[0, 0, 1] = 9
    out = cv.image_to_mask(_image(rgb, "rgb8"))
    assert out.shape == (3, 5) and out[0, 0] and out.sum() == 1
    f = np.zeros((3, 5), np.float32)
    f[0, 1], f[0, 2] = 1.0, np.nan                                         # NaN = nền
    assert np.array_equal(cv.image_to_mask(_image(f, "32FC1")), np.isfinite(f) & (f != 0))
    with pytest.raises(ValueError):
        cv.image_to_mask(_image(np.zeros((2, 2), np.float64), "64FC1"))
