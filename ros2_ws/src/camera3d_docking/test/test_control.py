"""Task 9 — core/control.py (luật điều khiển mượt Park–Kuipers + máy trạng thái docking)"""
import math

import numpy as np
import pytest

from camera3d_docking.core import control as ctl
from camera3d_docking.core import geometry as g

DT = 0.05


def _target_in_robot(robot, target):
    """robot, target: (x, y, yaw) trong world -> target trong frame robot."""
    T = g.inv_T(g.T_from_xyz_yaw(robot[0], robot[1], 0, robot[2])) @ g.T_from_xyz_yaw(
        target[0], target[1], 0, target[2])
    return g.pose2d(T)


def _step(robot, v, w):
    x, y, th = robot
    return (x + v * math.cos(th) * DT, y + v * math.sin(th) * DT, th + w * DT)


def _err_in_target(robot, target):
    """Sai số robot trong frame đích: (dọc trục, ngang, hướng)."""
    x, y, yaw = _target_in_robot(target, robot)
    return x, y, yaw


@pytest.mark.parametrize("lat,yaw_deg", [(0.0, 0.0), (0.3, 0.0), (-0.3, 0.0), (0.2, 20.0),
                                         (-0.2, -20.0), (0.3, -15.0)])
def test_control_converges_from_offsets(lat, yaw_deg):
    target = (0.0, 0.0, 0.0)
    robot = (-1.5, lat, math.radians(yaw_deg))
    p = ctl.ControlLawParams()
    for _ in range(2000):
        x, y, yaw = _target_in_robot(robot, target)
        if x <= 0.01:  # đã tới (theo trục robot)
            break
        v, w = ctl.smooth_control(x, y, yaw, p)
        assert v >= 0.0 and abs(w) <= p.w_max + 1e-9 and v <= p.v_max + 1e-9
        robot = _step(robot, v, w)
    ax, lat_err, yaw_err = _err_in_target(robot, target)
    assert abs(ax) < 0.02
    assert abs(lat_err) < 0.01 and abs(math.degrees(yaw_err)) < 1.0


def test_control_backward_docking():
    target = (0.0, 0.0, 0.0)               # robot phải dừng tại đây, mặt hướng +x
    robot = (1.5, 0.2, math.radians(10))    # đứng phía trước, lùi vào
    p = ctl.ControlLawParams()
    for _ in range(2000):
        x, y, yaw = _target_in_robot(robot, target)
        if x >= -0.01:
            break
        v, w = ctl.smooth_control(x, y, yaw, p, backward=True)
        assert v <= 0.0
        robot = _step(robot, v, w)
    ax, lat_err, yaw_err = _err_in_target(robot, target)
    assert abs(ax) < 0.02 and abs(lat_err) < 0.01 and abs(math.degrees(yaw_err)) < 1.0


def _run_fsm(fsm, robot, target, seconds, detect=lambda t, r: True, contact=lambda r: False):
    t = 0.0
    while t < seconds:
        tgt = _target_in_robot(robot, target)
        age = 0.0 if detect(t, robot) else 99.0
        v, w = fsm.step(t, tgt, age, contact(robot))
        robot = _step(robot, v, w)
        t += DT
        if fsm.state in (ctl.DockState.DOCKED, ctl.DockState.FAILED):
            break
    return robot


def test_fsm_happy_path_reaches_docked():
    fsm = ctl.DockingFSM(ctl.ControlLawParams(), ctl.FsmParams())
    fsm.start(0.0)
    target = (0.0, 0.0, 0.0)
    # camera mất vật khi còn < 0.4 m (vùng mù) -> FSM vẫn phải vào tới đích
    robot = _run_fsm(fsm, (-1.4, 0.15, math.radians(8)), target, 60.0,
                     detect=lambda t, r: _target_in_robot(r, target)[0] > 0.4)
    assert fsm.state == ctl.DockState.DOCKED, fsm.message
    ax, lat_err, yaw_err = _err_in_target(robot, target)
    assert abs(ax) < 0.02 and abs(lat_err) < 0.01 and abs(math.degrees(yaw_err)) < 1.0


def test_fsm_fails_without_detection():
    fsm = ctl.DockingFSM(ctl.ControlLawParams(), ctl.FsmParams(detection_timeout=2.0))
    fsm.start(0.0)
    for k in range(60):
        assert fsm.step(k * DT, None, None) == (0.0, 0.0)
    assert fsm.state == ctl.DockState.FAILED


def test_fsm_retreat_and_retry():
    p = ctl.FsmParams(retreat_distance=0.3, max_retries=2)
    fsm = ctl.DockingFSM(ctl.ControlLawParams(), p)
    fsm.start(0.0)
    target = (0.0, 0.0, 0.0)
    robot = (-1.5, 0.0, 0.0)
    states = set()
    retreat_v, retreat_x = [], []
    t = 0.0
    for _ in range(400):
        lost = t > 2.0                            # mất dấu khi còn xa (ngoài vùng mù)
        tgt = _target_in_robot(robot, target)
        v, w = fsm.step(t, tgt, 99.0 if lost else 0.0)
        states.add(fsm.state)
        robot = _step(robot, v, w)
        if fsm.state == ctl.DockState.RETREAT:
            retreat_v.append(v)
            retreat_x.append(robot[0])
        t += DT
        if fsm.state == ctl.DockState.WAIT_DETECTION and fsm.retries == 1:
            break
    assert ctl.DockState.RETREAT in states
    assert fsm.state == ctl.DockState.WAIT_DETECTION and fsm.retries == 1
    assert min(retreat_v) < 0.0                   # giảm tốc (giới hạn gia tốc) rồi mới lùi
    backed_off = max(retreat_x) - robot[0]        # lùi thật từ điểm xa nhất
    assert backed_off >= 0.9 * p.retreat_distance


def test_fsm_contact_ends_docking_and_cancel_stops():
    fsm = ctl.DockingFSM(ctl.ControlLawParams(), ctl.FsmParams())
    fsm.start(0.0)
    fsm.step(0.0, (1.0, 0.0, 0.0), 0.0)
    fsm.step(0.05, (1.0, 0.0, 0.0), 0.0, contact=True)
    assert fsm.state == ctl.DockState.DOCKED
    fsm.start(1.0)
    fsm.cancel()
    assert fsm.state == ctl.DockState.IDLE and fsm.step(1.1, (1.0, 0, 0), 0.0) == (0.0, 0.0)
