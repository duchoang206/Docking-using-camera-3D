"""Chuyển đổi message ROS 2 <-> numpy (nhanh, không vòng lặp Python theo từng điểm)."""
from __future__ import annotations

import numpy as np
from geometry_msgs.msg import Point, Pose, Quaternion
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header

from ..core.geometry import R_from_quat, make_T, quat_from_R

_DT = {PointField.INT8: "i1", PointField.UINT8: "u1", PointField.INT16: "i2",
       PointField.UINT16: "u2", PointField.INT32: "i4", PointField.UINT32: "u4",
       PointField.FLOAT32: "f4", PointField.FLOAT64: "f8"}


def pointcloud2_to_xyz(msg: PointCloud2, stride: int = 1) -> np.ndarray:
    """PointCloud2 bất kỳ (organized/unorganized, có padding/rgb) -> (N,3) float32, bỏ NaN.

    stride > 1: lấy mẫu thưa theo hàng/cột (cloud organized) TRƯỚC khi chuyển đổi -> rất rẻ.
    """
    fields = {f.name: f for f in msg.fields}
    if not all(k in fields for k in ("x", "y", "z")):
        raise ValueError("PointCloud2 thiếu trường x/y/z")
    endian = ">" if msg.is_bigendian else "<"
    dtype = np.dtype({"names": ["x", "y", "z"],
                      "formats": [endian + _DT[fields[k].datatype] for k in ("x", "y", "z")],
                      "offsets": [fields[k].offset for k in ("x", "y", "z")],
                      "itemsize": msg.point_step})
    H, W = int(msg.height), int(msg.width)
    if H * W == 0:
        return np.zeros((0, 3), dtype=np.float32)
    buf = np.frombuffer(msg.data, dtype=np.uint8)
    if msg.row_step != W * msg.point_step:          # hàng có đệm -> cắt bỏ phần đệm
        buf = np.ascontiguousarray(buf.reshape(H, msg.row_step)[:, :W * msg.point_step])
    arr = np.frombuffer(buf, dtype=dtype, count=H * W).reshape(H, W)
    if stride > 1:
        arr = arr[::stride, ::stride] if H > 1 else arr[:, ::stride]
    xyz = np.empty(arr.shape + (3,), dtype=np.float32)
    xyz[..., 0] = arr["x"]
    xyz[..., 1] = arr["y"]
    xyz[..., 2] = arr["z"]
    xyz = xyz.reshape(-1, 3)
    return xyz[np.isfinite(xyz).all(axis=1)]


def xyz_to_pointcloud2(points, frame_id: str, stamp, organized: bool = False) -> PointCloud2:
    """(N,3) hoặc (H,W,3) -> PointCloud2 xyz float32 (organized nếu organized=True)."""
    P = np.asarray(points, dtype=np.float32)
    H, W = (P.shape[0], P.shape[1]) if (organized and P.ndim == 3) else (1, P.reshape(-1, 3).shape[0])
    data = np.ascontiguousarray(P.reshape(-1, 3), dtype="<f4")
    msg = PointCloud2()
    msg.header = Header(frame_id=frame_id, stamp=stamp)
    msg.height, msg.width = H, W
    msg.fields = [PointField(name=n, offset=4 * i, datatype=PointField.FLOAT32, count=1)
                  for i, n in enumerate(("x", "y", "z"))]
    msg.is_bigendian = False
    msg.point_step = 12
    msg.row_step = 12 * W
    msg.is_dense = bool(np.isfinite(data).all())
    msg.data = data.tobytes()
    return msg


def start_tf_listener(buffer):
    """TF listener trên NODE NỘI BỘ RIÊNG + luồng riêng.

    Không truyền node chính vào TransformListener(spin_thread=True): luồng TF sẽ thêm node
    đó vào executor của nó -> hai executor giành nhau một node (rclpy chỉ cho 1 executor),
    callback có thể chạy trên luồng TF và tra TF có timeout sẽ tự chặn chính mình.
    """
    from tf2_ros import TransformListener

    return TransformListener(buffer, None, spin_thread=True)


def stop_tf_listener(listener) -> None:
    """Dừng luồng của listener (gọi trong destroy_node, trước khi hủy node chính)."""
    executor = getattr(listener, "executor", None)
    thread = getattr(listener, "dedicated_listener_thread", None)
    if executor is not None:
        executor.shutdown()
    if thread is not None:
        thread.join(timeout=2.0)


def transform_to_T(tr) -> np.ndarray:
    """geometry_msgs/Transform -> 4x4."""
    q = tr.rotation
    t = tr.translation
    return make_T(R_from_quat((q.x, q.y, q.z, q.w)), (t.x, t.y, t.z))


def pose_to_T(pose: Pose) -> np.ndarray:
    q = pose.orientation
    p = pose.position
    return make_T(R_from_quat((q.x, q.y, q.z, q.w)), (p.x, p.y, p.z))


def T_to_pose(T) -> Pose:
    x, y, z, w = quat_from_R(np.asarray(T)[:3, :3])
    t = np.asarray(T)[:3, 3]
    return Pose(position=Point(x=float(t[0]), y=float(t[1]), z=float(t[2])),
                orientation=Quaternion(x=float(x), y=float(y), z=float(z), w=float(w)))
