"""Test end-to-end docking trong Gazebo (headless).

Với mỗi pose xuất phát: khởi chạy sim + pose_estimator + docking_controller (domain ROS riêng),
gọi /docking_controller/start, chờ DOCKED/FAILED, rồi so pose THẬT của robot (ground truth từ
Gazebo) với pose đích = mặt trước pallet ⊕ dock_offset_x. In bảng PASS/FAIL.

    ros2 run camera3d_docking_sim e2e_docking_test
    ros2 run camera3d_docking_sim e2e_docking_test --starts "1.6,0,0" "1.5,0.25,15"
"""
from __future__ import annotations

import argparse
import math
import os
import signal
import subprocess
import sys
import time

PALLET = (3.0, 0.0, 0.0)          # mặt trước pallet trong world (generate_world.PALLET_POSE)
DOCK_OFFSET_X = -0.35             # config/controller.yaml
DEFAULT_STARTS = ["1.6,0.0,0", "1.5,0.25,15", "1.7,-0.3,-20"]


def _yaw_from_quat(q) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def run_case(x, y, yaw_deg, domain, timeout, log_path, tol_lat, tol_yaw, tol_along):
    import rclpy
    from rclpy.context import Context
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.qos import DurabilityPolicy, QoSProfile
    from std_msgs.msg import String
    from std_srvs.srv import Trigger
    from tf2_msgs.msg import TFMessage

    env = dict(os.environ, PYTHONNOUSERSITE="1", ROS_DOMAIN_ID=str(domain),
               ROS_AUTOMATIC_DISCOVERY_RANGE="LOCALHOST")
    cmd = ["ros2", "launch", "camera3d_docking_sim", "sim.launch.py", "headless:=true",
           "rviz:=false", f"x:={x}", f"y:={y}", f"yaw:={yaw_deg}"]
    log = open(log_path, "w")
    proc = subprocess.Popen(cmd, env=env, stdout=log, stderr=subprocess.STDOUT,
                            start_new_session=True)
    ctx = Context()
    rclpy.init(context=ctx, domain_id=domain)
    node = rclpy.create_node("e2e_monitor", context=ctx)
    ex = SingleThreadedExecutor(context=ctx)
    ex.add_node(node)
    state = {"ctrl": "", "gt": None, "status": ""}

    def on_gt(msg):
        for t in msg.transforms:
            if "docking_bot" in t.child_frame_id:
                tr = t.transform
                state["gt"] = (tr.translation.x, tr.translation.y, _yaw_from_quat(tr.rotation))

    node.create_subscription(String, "/docking_controller/state",
                             lambda m: state.__setitem__("ctrl", m.data),
                             QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    node.create_subscription(String, "/pose_estimator/status",
                             lambda m: state.__setitem__("status", m.data), 10)
    node.create_subscription(TFMessage, "/ground_truth/pose", on_gt, 10)
    client = node.create_client(Trigger, "/docking_controller/start")
    result = {"start": (x, y, yaw_deg), "state": "TIMEOUT", "ok": False}
    t0 = time.monotonic()

    def spin_until(cond, seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            ex.spin_once(timeout_sec=0.05)
            if cond():
                return True
        return False

    try:
        if not spin_until(lambda: '"accepted": true' in state["status"], 60.0):
            result["state"] = "NO_DETECTION"
            return result
        if not client.wait_for_service(timeout_sec=10.0):
            result["state"] = "NO_SERVICE"
            return result
        fut = client.call_async(Trigger.Request())
        spin_until(fut.done, 10.0)
        spin_until(lambda: state["ctrl"].startswith(("DOCKED", "FAILED")), timeout)
        spin_until(lambda: False, 1.0)                     # chờ robot đứng yên, cập nhật GT
        result["state"] = state["ctrl"].split(":")[0] or "TIMEOUT"
        if state["gt"] is not None:
            gx, gy, gyaw = state["gt"]
            ex_x = PALLET[0] + DOCK_OFFSET_X * math.cos(PALLET[2])
            ex_y = PALLET[1] + DOCK_OFFSET_X * math.sin(PALLET[2])
            c, s = math.cos(PALLET[2]), math.sin(PALLET[2])
            along = c * (gx - ex_x) + s * (gy - ex_y)
            lat = -s * (gx - ex_x) + c * (gy - ex_y)
            dyaw = math.degrees(math.atan2(math.sin(gyaw - PALLET[2]), math.cos(gyaw - PALLET[2])))
            result.update(along=along, lat=lat, yaw=dyaw)
            result["ok"] = (result["state"] == "DOCKED" and abs(lat) <= tol_lat
                            and abs(dyaw) <= tol_yaw and abs(along) <= tol_along)
        return result
    finally:
        result["time"] = time.monotonic() - t0
        ex.shutdown()
        node.destroy_node()
        rclpy.shutdown(context=ctx)
        try:
            os.killpg(proc.pid, signal.SIGINT)
            proc.wait(timeout=20)
        except (subprocess.TimeoutExpired, ProcessLookupError):
            try:
                os.killpg(proc.pid, signal.SIGKILL)
            except ProcessLookupError:
                pass
        log.close()


def main(argv=None) -> int:
    from camera3d_docking._env import ensure_clean_python

    ensure_clean_python()
    ap = argparse.ArgumentParser(description="E2E docking trong Gazebo")
    ap.add_argument("--starts", nargs="+", default=DEFAULT_STARTS, help='"x,y,yaw_deg" (world)')
    ap.add_argument("--timeout", type=float, default=90.0)
    ap.add_argument("--log-dir", default=os.path.join(os.getcwd(), "logs"))
    ap.add_argument("--tol-lat", type=float, default=0.02)
    ap.add_argument("--tol-yaw", type=float, default=2.0)
    ap.add_argument("--tol-along", type=float, default=0.03)
    a = ap.parse_args(argv if argv is not None else sys.argv[1:])
    os.makedirs(a.log_dir, exist_ok=True)
    rows = []
    for i, st in enumerate(a.starts):
        x, y, yaw = (float(v) for v in st.split(","))
        log_path = os.path.join(a.log_dir, f"e2e_case{i}.log")
        print(f"[{i + 1}/{len(a.starts)}] xuất phát (x={x}, y={y}, yaw={yaw}°) ... log: {log_path}",
              flush=True)
        r = run_case(x, y, yaw, 60 + (os.getpid() + i) % 40, a.timeout, log_path,
                     a.tol_lat, a.tol_yaw, a.tol_along)
        rows.append(r)
        if "lat" in r:
            print(f"    -> {r['state']}: dọc={100 * r['along']:+.1f} cm, ngang={100 * r['lat']:+.1f} cm, "
                  f"yaw={r['yaw']:+.2f}° ({r['time']:.0f} s) {'PASS' if r['ok'] else 'FAIL'}", flush=True)
        else:
            print(f"    -> {r['state']} FAIL ({r['time']:.0f} s)", flush=True)
    n_ok = sum(r["ok"] for r in rows)
    print(f"KẾT QUẢ: {n_ok}/{len(rows)} PASS (ngưỡng ngang ≤ {100 * a.tol_lat:.0f} cm, "
          f"yaw ≤ {a.tol_yaw}°, dọc ≤ {100 * a.tol_along:.0f} cm)")
    return 0 if n_ok == len(rows) else 1


if __name__ == "__main__":
    sys.exit(main())
