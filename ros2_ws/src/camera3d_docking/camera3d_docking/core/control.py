"""Điều khiển docking: luật điều khiển mượt Park & Kuipers (2011) + máy trạng thái.

Luật giống nav2_graceful_controller / opennav_docking (cùng tham số mặc định), viết lại
bằng Python cho robot KHÔNG chạy Nav2. Đích (x, y, ψ) tính trong frame robot.
    α = atan2(y, x);  φ = wrap(ψ − α);  δ = wrap(−α);  r = √(x²+y²)
    κ = −(1/r)·[k_δ(δ − atan(−k_φ φ)) + (1 + k_φ/(1+(k_φ φ)²)) sin δ]
    v = v_max / (1 + β|κ|^λ) (giảm khi r < slowdown_radius);  ω = κ·v
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from enum import Enum


def _wrap(a: float) -> float:
    return (a + math.pi) % (2.0 * math.pi) - math.pi


@dataclass
class ControlLawParams:
    k_phi: float = 3.0
    k_delta: float = 2.0
    beta: float = 0.4
    lam: float = 2.0
    v_max: float = 0.25
    v_min: float = 0.02
    w_max: float = 0.75
    slowdown_radius: float = 0.25


def smooth_control(x: float, y: float, yaw: float, p: ControlLawParams, backward: bool = False):
    """(v, ω) đưa robot tới đích (x, y, yaw) trong frame robot. backward: lùi vào đích."""
    if backward:  # robot "ảo" quay 180°: đích trong frame ảo = (−x, −y, yaw)
        v, w = smooth_control(-x, -y, yaw, p, backward=False)
        return -v, w
    r = math.hypot(x, y)
    if r < 1e-6:
        return 0.0, 0.0
    alpha = math.atan2(y, x)
    phi = _wrap(yaw - alpha)
    delta = _wrap(-alpha)
    kappa = -(1.0 / r) * (p.k_delta * (delta - math.atan(-p.k_phi * phi))
                          + (1.0 + p.k_phi / (1.0 + (p.k_phi * phi) ** 2)) * math.sin(delta))
    v = p.v_max / (1.0 + p.beta * abs(kappa) ** p.lam)
    v = min(v, p.v_max * r / p.slowdown_radius)
    v = min(max(v, p.v_min), p.v_max)
    if abs(kappa * v) > p.w_max:  # giữ nguyên độ cong khi bão hòa vận tốc góc
        v = p.w_max / abs(kappa)
    return v, kappa * v


class DockState(str, Enum):
    IDLE = "IDLE"
    WAIT_DETECTION = "WAIT_DETECTION"
    APPROACH = "APPROACH"
    BLIND = "BLIND"
    DOCKED = "DOCKED"
    RETREAT = "RETREAT"
    FAILED = "FAILED"


@dataclass
class FsmParams:
    goal_tol_x: float = 0.01          # dừng khi còn < 1 cm dọc trục dock
    blind_zone: float = 0.6           # < 0.6 m tới đích: không cần phát hiện mới (vùng mù)
    blind_speed: float = 0.08         # tốc độ tối đa trong vùng mù (m/s)
    detection_timeout: float = 5.0    # chờ phát hiện ban đầu (s)
    lost_timeout: float = 1.0         # mất dấu ngoài vùng mù quá lâu -> lùi lại thử lại
    approach_timeout: float = 60.0
    max_retries: int = 2
    retreat_distance: float = 0.5
    retreat_speed: float = 0.1
    acc_v: float = 0.5                # giới hạn gia tốc (m/s²)
    acc_w: float = 1.5                # (rad/s²)
    backward: bool = False            # lùi vào (xe nâng càng phía sau)


class DockingFSM:
    """IDLE → WAIT_DETECTION → APPROACH → BLIND → DOCKED  (lỗi: RETREAT → thử lại | FAILED)."""

    def __init__(self, law: ControlLawParams | None = None, params: FsmParams | None = None):
        self.law = law or ControlLawParams()
        self.p = params or FsmParams()
        self.state = DockState.IDLE
        self.message = ""
        self.retries = 0
        self._v = self._w = 0.0
        self._t_state = self._t_start = self._t_last = 0.0
        self._retreated = 0.0

    # ------------------------------------------------------------------ điều khiển vòng ngoài
    def start(self, now: float) -> None:
        self.retries = 0
        self._t_start = now
        self._set(DockState.WAIT_DETECTION, now, "Đang chờ phát hiện vật")
        self._v = self._w = 0.0
        self._t_last = now

    def cancel(self) -> None:
        self.state = DockState.IDLE
        self.message = "Đã hủy"
        self._v = self._w = 0.0

    def _set(self, state: DockState, now: float, msg: str = "") -> None:
        self.state = state
        self._t_state = now
        self.message = msg

    def _remaining(self, tgt) -> float:
        """Quãng còn lại dọc trục đích (dương = chưa tới)."""
        x, y, yaw = tgt
        along = -(math.cos(yaw) * -x + math.sin(yaw) * -y)  # = −x của robot trong frame đích
        return -along if self.p.backward else along

    def _limit(self, v: float, w: float, dt: float):
        dv = self.p.acc_v * dt
        dw = self.p.acc_w * dt
        self._v = min(max(v, self._v - dv), self._v + dv)
        self._w = min(max(w, self._w - dw), self._w + dw)
        return self._v, self._w

    def _fail_or_retry(self, now: float, reason: str) -> None:
        if self.retries < self.p.max_retries:
            self._retreated = 0.0
            self._set(DockState.RETREAT, now, f"{reason} — lùi lại thử lần {self.retries + 1}")
        else:
            self._set(DockState.FAILED, now, f"{reason} — đã hết số lần thử")

    def step(self, now: float, target_base, detection_age, contact: bool = False):
        """Một chu kỳ điều khiển -> (v, ω). target_base: đích cuối (x,y,yaw) trong base hoặc None."""
        dt = max(0.0, now - self._t_last)
        self._t_last = now
        s, p = self.state, self.p
        if s in (DockState.IDLE, DockState.DOCKED, DockState.FAILED):
            self._v = self._w = 0.0
            return 0.0, 0.0
        if contact and s in (DockState.APPROACH, DockState.BLIND):
            self._set(DockState.DOCKED, now, "Tiếp xúc — đã dock")
            self._v = self._w = 0.0
            return 0.0, 0.0
        if now - self._t_start > p.approach_timeout:
            self._set(DockState.FAILED, now, "Quá thời gian docking")
            self._v = self._w = 0.0
            return 0.0, 0.0
        fresh = detection_age is not None and detection_age < p.lost_timeout

        if s == DockState.WAIT_DETECTION:
            if target_base is not None and fresh:
                self._set(DockState.APPROACH, now, "Đang tiếp cận")
            elif now - self._t_state > p.detection_timeout:
                self._set(DockState.FAILED, now, "Không phát hiện được vật")
            self._v = self._w = 0.0
            return 0.0, 0.0

        if s == DockState.RETREAT:
            sign = 1.0 if p.backward else -1.0   # đi ngược hướng tiếp cận
            # chỉ tính phần đi THEO hướng lùi (lúc đầu còn trôi tới do giới hạn gia tốc)
            self._retreated += max(0.0, sign * self._v) * dt
            if self._retreated >= p.retreat_distance:
                self.retries += 1
                self._set(DockState.WAIT_DETECTION, now, "Thử tiếp cận lại")
                self._v = self._w = 0.0
                return 0.0, 0.0
            return self._limit(sign * p.retreat_speed, 0.0, dt)

        # APPROACH / BLIND
        if target_base is None:
            self._fail_or_retry(now, "Mất pose đích")
            return self._limit(0.0, 0.0, dt)
        remaining = self._remaining(target_base)
        if remaining <= p.goal_tol_x:
            self._set(DockState.DOCKED, now, f"Đã dock (còn {100 * remaining:.1f} cm)")
            self._v = self._w = 0.0
            return 0.0, 0.0
        if s == DockState.APPROACH:
            if remaining < p.blind_zone:
                self._set(DockState.BLIND, now, "Vào vùng mù — bám theo pose giữ trong odom")
            elif not fresh:
                self._fail_or_retry(now, "Mất dấu vật")
                return self._limit(0.0, 0.0, dt)
        v, w = smooth_control(*target_base, self.law, backward=p.backward)
        if self.state == DockState.BLIND and abs(v) > p.blind_speed:
            scale = p.blind_speed / abs(v)
            v, w = v * scale, w * scale
        return self._limit(v, w, dt)
