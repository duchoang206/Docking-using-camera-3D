"""Bộ lọc Kalman cho pose (x, y, yaw) của VẬT ĐỨNG YÊN trong frame cố định (odom).

Bài học từ Aghili (arXiv 2211.03026): phép đo thị giác 3D nhiễu, tần số thấp, có lúc bị
che -> dùng mô hình chuyển động + Kalman để luôn có pose dự đoán. Ở đây vật đứng yên
trong odom nên mô hình là "hằng số + nhiễu quá trình nhỏ" (cho phép odometry trôi chậm).
Khi robot vào vùng mù (quá gần camera), pose trong odom vẫn đúng -> tiếp tục điều khiển.
"""
from __future__ import annotations

import math

import numpy as np


def _wrap(a: float) -> float:
    return (a + math.pi) % (2.0 * math.pi) - math.pi


class StaticPoseKF:
    def __init__(self, q_xy: float = 0.005, q_yaw: float = math.radians(0.2),
                 gate_chi2: float = 14.16, reset_after: int = 5,
                 reset_consistency_xy: float = 0.05,
                 reset_consistency_yaw: float = math.radians(3.0)):
        self.q = np.array([q_xy**2, q_xy**2, q_yaw**2])  # nhiễu quá trình mỗi giây
        self.gate_chi2 = gate_chi2                        # χ²(3 bậc, 99.7%) = 14.16
        self.reset_after = reset_after
        self.reset_xy = reset_consistency_xy
        self.reset_yaw = reset_consistency_yaw
        self.reset()

    def reset(self) -> None:
        self.x = None
        self.P = None
        self.last_update = None
        self._t = None
        self._rejects = []

    @property
    def initialized(self) -> bool:
        return self.x is not None

    def predict(self, stamp: float) -> None:
        """Tăng hiệp phương sai theo thời gian (vật đứng yên: trạng thái không đổi)."""
        if not self.initialized:
            return
        dt = max(0.0, float(stamp) - self._t)
        self.P = self.P + np.diag(self.q * dt)
        self._t = max(self._t, float(stamp))

    def _init(self, z, R, stamp):
        self.x = np.array(z, dtype=float)
        self.x[2] = _wrap(self.x[2])
        self.P = np.array(R, dtype=float)
        self._t = self.last_update = float(stamp)
        self._rejects = []

    def update(self, z, R, stamp: float) -> bool:
        """Cập nhật với phép đo z=(x,y,yaw), hiệp phương sai R. Trả False nếu bị cổng loại."""
        z = np.asarray(z, dtype=float)
        R = np.asarray(R, dtype=float)
        if not self.initialized:
            self._init(z, R, stamp)
            return True
        self.predict(stamp)
        nu = z - self.x
        nu[2] = _wrap(nu[2])
        S = self.P + R
        d2 = float(nu @ np.linalg.solve(S, nu))
        if d2 > self.gate_chi2:
            self._rejects.append(z)
            recent = self._rejects[-self.reset_after:]
            if len(recent) >= self.reset_after and self._consistent(recent):
                self._init(self._mean(recent), R, stamp)   # khóa sai lúc đầu -> khởi tạo lại
                return True
            return False
        K = self.P @ np.linalg.inv(S)
        self.x = self.x + K @ nu
        self.x[2] = _wrap(self.x[2])
        I_K = np.eye(3) - K
        self.P = I_K @ self.P @ I_K.T + K @ R @ K.T          # dạng Joseph (ổn định số)
        self._rejects = []
        self.last_update = float(stamp)
        return True

    def _mean(self, zs):
        Z = np.array(zs)
        yaw = math.atan2(np.sin(Z[:, 2]).mean(), np.cos(Z[:, 2]).mean())
        return np.array([Z[:, 0].mean(), Z[:, 1].mean(), yaw])

    def _consistent(self, zs) -> bool:
        m = self._mean(zs)
        Z = np.array(zs)
        dxy = np.hypot(Z[:, 0] - m[0], Z[:, 1] - m[1]).max()
        dyaw = max(abs(_wrap(a - m[2])) for a in Z[:, 2])
        return dxy <= self.reset_xy and dyaw <= self.reset_yaw
