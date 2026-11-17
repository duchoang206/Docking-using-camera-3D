"""Benchmark độ chính xác + thời gian xử lý trên dữ liệu tổng hợp có ground-truth.

Chạy trên CHÍNH máy tính của robot để biết có đủ nhanh không:
    ros2 run camera3d_docking benchmark            # đầy đủ
    ros2 run camera3d_docking benchmark --quick    # nhanh
"""
from __future__ import annotations

import argparse
import sys


def main(argv=None) -> int:
    from camera3d_docking._env import ensure_clean_python

    ensure_clean_python()
    import time

    import numpy as np

    from camera3d_docking.core import estimator as est_mod
    from camera3d_docking.core import geometry as g
    from camera3d_docking.core import shapes as s
    from camera3d_docking.core import synthetic as syn
    from camera3d_docking.core import template as tp

    ap = argparse.ArgumentParser(description="Benchmark ước lượng pose docking")
    ap.add_argument("--quick", action="store_true", help="ít kịch bản hơn")
    ap.add_argument("--object", choices=["pallet", "rack", "vdock"], default="pallet")
    ap.add_argument("--noise-quad", type=float, default=0.002, help="σ(z)=0.001+q·z² (m)")
    ap.add_argument("--seed", type=int, default=0)
    args = ap.parse_args(argv if argv is not None else sys.argv[1:])

    boxes = {"pallet": s.euro_pallet_boxes, "rack": s.rack_boxes, "vdock": s.v_dock_boxes}[args.object]()
    t0 = time.perf_counter()
    tpl = tp.template_from_boxes(boxes, args.object)
    est = est_mod.DockPoseEstimator(tpl, est_mod.EstimatorParams())
    print(f"Khởi tạo template ({len(tpl.points)} điểm) + estimator: {time.perf_counter() - t0:.2f} s")

    dists = [1.2, 2.0] if args.quick else [1.0, 1.5, 2.0, 2.5]
    yaws = [-20, 0, 20] if args.quick else [-30, -15, 0, 15, 30]
    lats = [0.0] if args.quick else [0.0, 0.25]
    rng = np.random.default_rng(args.seed)
    noise = s.DepthNoise(0.001, args.noise_quad)
    rows = []
    print(f"{'x':>5} {'y':>5} {'yaw':>5} | {'trạng thái':<11} {'Δpos(cm)':>8} {'Δyaw(°)':>7} "
          f"| {'tìm(ms)':>7} {'bám(ms)':>7}")
    for d in dists:
        for yaw in yaws:
            for lat in lats:
                sc = syn.render_scene([(boxes, d, lat, np.radians(yaw))], noise=noise, rng=rng)
                est.reset()
                det = est.process(sc["points_base"], sc["camera_pos_base"])
                prior = g.T_from_xyz_yaw(d + 0.02, lat - 0.02, 0.0, np.radians(yaw + 1.0))
                det_t = est.process(sc["points_base"], sc["camera_pos_base"], prior_T_base_obj=prior)
                if det.T_base_obj is not None:
                    x, y, t = g.pose2d(det.T_base_obj)
                    dp = 100 * np.hypot(x - d, y - lat)
                    dy = abs(np.degrees(g.wrap_angle(t - np.radians(yaw))))
                else:
                    dp, dy = float("nan"), float("nan")
                rows.append((det.status.value, dp, dy, det.timings["total"], det_t.timings["total"]))
                print(f"{d:5.2f} {lat:5.2f} {yaw:5d} | {det.status.value:<11} {dp:8.2f} {dy:7.2f} "
                      f"| {det.timings['total']:7.1f} {det_t.timings['total']:7.1f}")
    ok = [r for r in rows if r[0] == "OK"]
    print("-" * 64)
    print(f"Thành công: {len(ok)}/{len(rows)}")
    if ok:
        dp = np.array([r[1] for r in ok])
        dy = np.array([r[2] for r in ok])
        print(f"Sai số vị trí : trung vị {np.median(dp):.2f} cm, P95 {np.percentile(dp, 95):.2f} cm")
        print(f"Sai số hướng  : trung vị {np.median(dy):.2f}°,  P95 {np.percentile(dy, 95):.2f}°")
    print(f"Thời gian     : tìm kiếm TB {np.mean([r[3] for r in rows]):.1f} ms, "
          f"bám TB {np.mean([r[4] for r in rows]):.1f} ms / khung")
    return 0


if __name__ == "__main__":
    sys.exit(main())
