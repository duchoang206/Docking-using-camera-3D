"""Tiện ích hình học SE(3) / SE(2) dùng chung (thuần numpy, không ROS).

Quy ước:
- Ma trận biến đổi 4x4 ``T_a_b`` đưa điểm từ frame b sang frame a: p_a = T_a_b @ p_b.
- Quaternion theo thứ tự ROS (x, y, z, w).
- Góc đơn vị radian.
"""
from __future__ import annotations

import math

import numpy as np

# Quay từ frame quang học (x phải, y xuống, z tới trước) sang frame link (x tới, y trái, z lên)
R_LINK_FROM_OPTICAL = np.array(
    [[0.0, 0.0, 1.0],
     [-1.0, 0.0, 0.0],
     [0.0, -1.0, 0.0]]
)


def wrap_angle(a):
    """Đưa góc về khoảng [-pi, pi)."""
    return (np.asarray(a, dtype=float) + np.pi) % (2.0 * np.pi) - np.pi


def skew(v) -> np.ndarray:
    """Ma trận phản đối xứng [v]x sao cho [v]x @ u = v x u."""
    return np.array([[0.0, -v[2], v[1]], [v[2], 0.0, -v[0]], [-v[1], v[0], 0.0]])


def rot_x(a: float) -> np.ndarray:
    c, s = math.cos(a), math.sin(a)
    return np.array([[1.0, 0.0, 0.0], [0.0, c, -s], [0.0, s, c]])


def rot_y(a: float) -> np.ndarray:
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]])


def rot_z(a: float) -> np.ndarray:
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def so3_exp(w) -> np.ndarray:
    """Công thức Rodrigues: vector quay (trục * góc) -> ma trận quay 3x3."""
    w = np.asarray(w, dtype=float)
    theta = float(np.linalg.norm(w))
    if theta < 1e-12:
        return np.eye(3) + skew(w)
    K = skew(w / theta)
    return np.eye(3) + math.sin(theta) * K + (1.0 - math.cos(theta)) * (K @ K)


def make_T(R, t) -> np.ndarray:
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = t
    return T


def T_from_xyz_yaw(x: float, y: float, z: float, yaw: float) -> np.ndarray:
    return make_T(rot_z(yaw), (x, y, z))


def T_from_xyz_rpy(x, y, z, roll, pitch, yaw) -> np.ndarray:
    """R = Rz(yaw) @ Ry(pitch) @ Rx(roll) (quy ước ROS / tf2)."""
    return make_T(rot_z(yaw) @ rot_y(pitch) @ rot_x(roll), (x, y, z))


def inv_T(T) -> np.ndarray:
    R = T[:3, :3]
    Ti = np.eye(4)
    Ti[:3, :3] = R.T
    Ti[:3, 3] = -R.T @ T[:3, 3]
    return Ti


def yaw_of(M) -> float:
    """Góc yaw của ma trận quay 3x3 hoặc biến đổi 4x4."""
    M = np.asarray(M)
    return math.atan2(M[1, 0], M[0, 0])


def rpy_from_R(R):
    """(roll, pitch, yaw) với R = Rz(yaw) Ry(pitch) Rx(roll)."""
    pitch = -math.asin(max(-1.0, min(1.0, float(R[2, 0]))))
    roll = math.atan2(R[2, 1], R[2, 2])
    yaw = math.atan2(R[1, 0], R[0, 0])
    return roll, pitch, yaw


def pose2d(T):
    """(x, y, yaw) của biến đổi 4x4."""
    return float(T[0, 3]), float(T[1, 3]), yaw_of(T)


def transform_points(T, P) -> np.ndarray:
    """Áp dụng T (4x4) cho mảng điểm (N,3). Giữ nguyên float32 nếu đầu vào là float32."""
    P = np.asarray(P)
    dt = P.dtype if P.dtype in (np.float32, np.float64) else np.float64
    R = np.asarray(T[:3, :3], dtype=dt)
    t = np.asarray(T[:3, 3], dtype=dt)
    return P.astype(dt, copy=False) @ R.T + t


def quat_from_R(R):
    """Ma trận quay -> quaternion (x, y, z, w), thuật toán Shepperd (ổn định số)."""
    R = np.asarray(R, dtype=float)
    tr = R[0, 0] + R[1, 1] + R[2, 2]
    if tr > 0.0:
        s = math.sqrt(tr + 1.0) * 2.0
        w = 0.25 * s
        x = (R[2, 1] - R[1, 2]) / s
        y = (R[0, 2] - R[2, 0]) / s
        z = (R[1, 0] - R[0, 1]) / s
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2.0
        w = (R[2, 1] - R[1, 2]) / s
        x = 0.25 * s
        y = (R[0, 1] + R[1, 0]) / s
        z = (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = math.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2.0
        w = (R[0, 2] - R[2, 0]) / s
        x = (R[0, 1] + R[1, 0]) / s
        y = 0.25 * s
        z = (R[1, 2] + R[2, 1]) / s
    else:
        s = math.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2.0
        w = (R[1, 0] - R[0, 1]) / s
        x = (R[0, 2] + R[2, 0]) / s
        y = (R[1, 2] + R[2, 1]) / s
        z = 0.25 * s
    q = np.array([x, y, z, w])
    q /= np.linalg.norm(q)
    if q[3] < 0.0:  # chọn dạng có w >= 0 cho nhất quán
        q = -q
    return q


def R_from_quat(q) -> np.ndarray:
    """Quaternion (x, y, z, w) -> ma trận quay 3x3."""
    x, y, z, w = np.asarray(q, dtype=float) / np.linalg.norm(q)
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])
