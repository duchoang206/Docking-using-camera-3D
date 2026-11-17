"""Benchmark độ chính xác + thời gian xử lý trên dữ liệu tổng hợp có ground-truth.

Chạy trên CHÍNH máy tính của robot để biết có đủ nhanh không:
    ros2 run camera3d_docking benchmark            # đầy đủ
    ros2 run camera3d_docking benchmark --quick    # nhanh

Segmentation có làm chính xác hơn không? So sánh trên các cảnh kho hàng với mặt nạ hoàn hảo
(oracle = trần trên của MỌI mạng segmentation) và các mặt nạ lỗi giống mạng thật:
    ros2 run camera3d_docking benchmark --seg-study [--quick]
"""
from __future__ import annotations

import argparse
import re
import sys

SEG_HELP = ("các kiểu mặt nạ, cách nhau dấu phẩy: none (không segmentation) | oracle (hoàn hảo) | "
            "dilateN (lem ra nền N px) | erodeN (hụt mép N px) | bboxN (chỉ có bbox, nới N px) | "
            "shiftN (ảnh màu lệch depth N px); thêm '@search' = mặt nạ chỉ dùng để tìm kiếm, "
            "ICP vẫn chạy trên mọi điểm")
DEFAULT_SEG = "none,oracle,oracle@search,dilate3,erode3,erode3@search,bbox3,shift3"


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
    ap.add_argument("--seg-study", action="store_true",
                    help="so sánh có/không mặt nạ segmentation trên các cảnh kho hàng")
    ap.add_argument("--scenes", default="clean,wall,load,row",
                    help="cảnh cho --seg-study: clean | wall | load | row")
    ap.add_argument("--seg", default=DEFAULT_SEG, help=SEG_HELP)
    args = ap.parse_args(argv if argv is not None else sys.argv[1:])

    boxes = {"pallet": s.euro_pallet_boxes, "rack": s.rack_boxes, "vdock": s.v_dock_boxes}[args.object]()
    t0 = time.perf_counter()
    tpl = tp.template_from_boxes(boxes, args.object)
    if args.seg_study:
        return seg_study(args, boxes, tpl)
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


# --------------------------------------------------------------------------- --seg-study
def _parse_seg(spec: str) -> list:
    """'oracle,erode3@search' -> [(tên, kiểu, px, mask_mode), ...]."""
    out = []
    for item in (x.strip() for x in spec.split(",")):
        m = re.fullmatch(r"(none|oracle|dilate|erode|bbox|shift)(\d*)(@search)?", item)
        if not m:
            raise SystemExit(f"--seg không hợp lệ: {item!r}\n{SEG_HELP}")
        out.append((item, m.group(1), int(m.group(2) or 0), "search" if m.group(3) else "hard"))
    return out


def _obj_errors(det, x, y, yaw):
    """(nhận?, sai dọc m, sai ngang m, sai yaw °) trong frame vật thật; NaN nếu không nhận."""
    import numpy as np

    from camera3d_docking.core import geometry as g

    if det.status.value != "OK":
        return False, np.nan, np.nan, np.nan
    ex, ey, eyaw = g.pose2d(det.T_base_obj)
    c, s_ = np.cos(yaw), np.sin(yaw)
    dx, dy = ex - x, ey - y
    return True, c * dx + s_ * dy, -s_ * dx + c * dy, float(np.degrees(g.wrap_angle(eyaw - yaw)))


def seg_study(args, boxes, tpl) -> int:
    """Mỗi cảnh x mỗi kiểu mặt nạ: tỉ lệ nhận, số lần nhận SAI, sai số, độ lệch, thời gian."""
    import numpy as np

    from camera3d_docking.core import estimator as est_mod
    from camera3d_docking.core import geometry as g
    from camera3d_docking.core import segmentation as seg
    from camera3d_docking.core import shapes as s
    from camera3d_docking.core import synthetic as syn

    conds = _parse_seg(args.seg)
    scenes = [k.strip() for k in args.scenes.split(",") if k.strip()]
    P = tpl.points
    dims = {"depth": float(P[:, 0].max()), "width": float(np.ptp(P[:, 1])),
            "height": float(P[:, 2].max())}
    ests = {mode: est_mod.DockPoseEstimator(tpl, est_mod.EstimatorParams(mask_mode=mode))
            for mode in sorted({c[3] for c in conds})}
    dists = [1.2, 2.0] if args.quick else [1.0, 1.5, 2.0, 2.5]
    yaws = [-20, 0, 20] if args.quick else [-30, -15, 0, 15, 30]
    lats = [0.0] if args.quick else [0.0, 0.25]
    noise = s.DepthNoise(0.001, args.noise_quad)
    rng = np.random.default_rng(args.seed)
    rows = {}
    for scene in scenes:
        for d in dists:
            for yaw_deg in yaws:
                for lat in lats:
                    yaw = np.radians(yaw_deg)
                    sc = syn.render_scene([(boxes, d, lat, yaw)], noise=noise, rng=rng,
                                          extra_boxes=syn.warehouse_clutter(scene, d, lat, yaw, **dims))
                    truth = sc["labels"] == 0
                    prior = g.T_from_xyz_yaw(d + 0.02, lat - 0.02, 0.0, yaw + np.radians(1.0))
                    for name, kind, px, mode in conds:
                        m = (None if kind == "none"
                             else seg.degrade_mask(truth, kind, px).ravel()[sc["pixel_index"]])
                        est = ests[mode]
                        est.reset()
                        det = est.process(sc["points_base"], sc["camera_pos_base"], object_mask=m)
                        trk = est.process(sc["points_base"], sc["camera_pos_base"],
                                          prior_T_base_obj=prior, object_mask=m)
                        ok_t, a_t, l_t, _ = _obj_errors(trk, d, lat, yaw)
                        rows.setdefault((scene, name), []).append(
                            _obj_errors(det, d, lat, yaw)
                            + (det.timings["total"], ok_t, float(np.hypot(a_t, l_t))))
    _print_seg_table(scenes, conds, rows)
    return 0


def _print_seg_table(scenes, conds, rows) -> None:
    import numpy as np

    def stat(v, fn, prec=1):
        return f"{fn(v):.{prec}f}" if len(v) else "-"

    def p95(v):
        return np.percentile(v, 95)

    print("Mặt nạ: none = không segmentation | oracle = hoàn hảo (trần trên của mọi mạng) | "
          "dilateN / erodeN = lem / hụt N px | bboxN = chỉ có bbox | shiftN = ảnh màu lệch "
          "depth N px | @search = mặt nạ chỉ dùng để tìm kiếm")
    print("Tìm = tìm kiếm toàn cục + ICP từ đầu; bám = ICP từ pose trước (lệch 2 cm, 1°). "
          "Nhận SAI = báo OK nhưng lệch > 2 cm hoặc > 2°. Lệch dọc = sai số dọc trung bình có dấu.")
    hdr = (f"{'cảnh':<6} {'mặt nạ':<15} {'nhận':>7} {'nhận SAI':>8} {'vị trí mm TB / P95':>18} "
           f"{'yaw ° TB / P95':>15} {'lệch dọc mm':>11} {'bám P95 mm':>10} {'ms':>5}")
    print(hdr)
    print("-" * len(hdr))
    for scene in scenes:
        for name, *_ in conds:
            R = rows[(scene, name)]
            acc = [r for r in R if r[0]]
            pos = np.array([1000 * np.hypot(r[1], r[2]) for r in acc])
            yaw = np.array([abs(r[3]) for r in acc])
            along = np.array([1000 * r[1] for r in acc])
            trk = np.array([1000 * r[6] for r in R if r[5]])
            wrong = int(np.sum((pos > 20.0) | (yaw > 2.0)))
            ms = float(np.median([r[4] for r in R]))
            print(f"{scene:<6} {name:<15} {len(acc):>3}/{len(R):<3} {wrong:>8} "
                  f"{stat(pos, np.median):>9} / {stat(pos, p95):<6} "
                  f"{stat(yaw, np.median, 2):>7} / {stat(yaw, p95, 2):<5} "
                  f"{stat(along, np.mean):>11} {stat(trk, p95):>10} {ms:>5.0f}")
        print()


if __name__ == "__main__":
    sys.exit(main())
