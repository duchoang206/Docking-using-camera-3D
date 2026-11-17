"""CLI tạo template cho vật cần dock.

  ros2 run camera3d_docking make_template pallet -o euro_pallet.npz
  ros2 run camera3d_docking make_template rack --bay-width 1.0 -o rack.npz
  ros2 run camera3d_docking make_template vdock -o v_dock.npz
  ros2 run camera3d_docking make_template from-mesh dock.stl --scale 0.001 -o dock.npz
  ros2 run camera3d_docking make_template from-cloud c_000.pcd c_001.pcd \
        --object-pose 1.20 0.03 2.5 --camera-pos 0.2 0 0.3 -o my_pallet.npz
  ros2 run camera3d_docking make_template export-pcd my_pallet.npz my_pallet.pcd  (xem bằng CloudCompare)

Frame vật: gốc tại tâm mặt trước, trên sàn; +x hướng vào trong vật; +z lên.
"""
from __future__ import annotations

import argparse
import sys


def _build_parser():
    ap = argparse.ArgumentParser(prog="make_template", description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="cmd", required=True)

    def out_args(p):
        p.add_argument("-o", "--output", required=True, help="file .npz (hoặc .pcd)")
        p.add_argument("--name", default=None)
        p.add_argument("--spacing", type=float, default=0.01, help="mật độ điểm (m)")

    p = sub.add_parser("pallet", help="pallet khối (mặc định EUR 1200x800x144, vào từ mặt 800)")
    p.add_argument("--depth", type=float, default=1.2)
    p.add_argument("--width", type=float, default=0.8)
    p.add_argument("--block-h", type=float, default=0.078)
    p.add_argument("--board-t", type=float, default=0.022)
    p.add_argument("--block-widths", type=float, nargs=3, default=[0.10, 0.145, 0.10])
    p.add_argument("--block-depth", type=float, default=0.145)
    out_args(p)
    p = sub.add_parser("rack", help="kệ 1 khoang (4 cột + dầm)")
    p.add_argument("--bay-width", type=float, default=1.0)
    p.add_argument("--depth", type=float, default=0.9)
    p.add_argument("--height", type=float, default=2.0)
    p.add_argument("--upright", type=float, nargs=2, default=[0.08, 0.08])
    p.add_argument("--beam-levels", type=float, nargs="+", default=[0.15, 1.2])
    p.add_argument("--beam-h", type=float, default=0.10)
    out_args(p)
    p = sub.add_parser("vdock", help="trạm sạc rãnh chữ V (khuyến nghị cho docking chính xác)")
    p.add_argument("--opening", type=float, default=0.5)
    p.add_argument("--depth", type=float, default=0.2)
    p.add_argument("--height", type=float, default=0.25)
    p.add_argument("--thickness", type=float, default=0.03)
    out_args(p)
    p = sub.add_parser("from-mesh", help="từ CAD .stl/.obj (mặt trước phải quay về -x)")
    p.add_argument("mesh")
    p.add_argument("--scale", type=float, default=1.0, help="vd 0.001 nếu CAD tính bằng mm")
    p.add_argument("--no-auto-origin", action="store_true",
                   help="không tự dời gốc về tâm mặt trước trên sàn")
    out_args(p)
    p = sub.add_parser("from-cloud", help="từ cloud quét thực tế (.pcd trong base_link)")
    p.add_argument("clouds", nargs="+")
    p.add_argument("--object-pose", type=float, nargs=3, required=True, metavar=("X", "Y", "YAW_DEG"),
                   help="pose frame vật trong frame của cloud (đo bằng thước / AprilTag)")
    p.add_argument("--crop", type=float, nargs=6, default=[-0.05, 2.0, -1.0, 1.0, 0.02, 2.0],
                   metavar=("XMIN", "XMAX", "YMIN", "YMAX", "ZMIN", "ZMAX"),
                   help="hộp cắt trong frame vật")
    p.add_argument("--camera-pos", type=float, nargs=3, default=[0.0, 0.0, 0.3],
                   help="vị trí camera trong frame của cloud (để định hướng pháp tuyến)")
    out_args(p)
    p = sub.add_parser("export-pcd", help="xuất template .npz sang .pcd (CloudCompare)")
    p.add_argument("input")
    p.add_argument("output")
    return ap


def main(argv=None) -> int:
    from camera3d_docking._env import ensure_clean_python

    ensure_clean_python()
    import math

    import numpy as np

    from camera3d_docking.core import fileio, shapes
    from camera3d_docking.core import template as tp
    from camera3d_docking.core.cloud import crop_box_mask
    from camera3d_docking.core.geometry import T_from_xyz_yaw, inv_T, transform_points

    a = _build_parser().parse_args(argv if argv is not None else sys.argv[1:])
    if a.cmd == "export-pcd":
        T = tp.Template.load(a.input)
        fileio.write_pcd(a.output, T.points, T.normals)
        print(f"Đã xuất {len(T.points)} điểm -> {a.output}")
        return 0
    name = a.name
    if a.cmd == "pallet":
        boxes = shapes.euro_pallet_boxes(a.depth, a.width, a.block_h, a.board_t, tuple(a.block_widths),
                                         a.block_depth)
        T = tp.template_from_boxes(boxes, name or "pallet", a.spacing,
                                   meta={"model": "pallet", "depth": a.depth, "width": a.width})
    elif a.cmd == "rack":
        boxes = shapes.rack_boxes(a.bay_width, a.depth, a.height, tuple(a.upright),
                                  tuple(a.beam_levels), a.beam_h)
        T = tp.template_from_boxes(boxes, name or "rack", a.spacing,
                                   cam_heights=(0.3, 0.8, 1.4), meta={"model": "rack"})
    elif a.cmd == "vdock":
        boxes = shapes.v_dock_boxes(a.opening, a.depth, a.height, a.thickness)
        T = tp.template_from_boxes(boxes, name or "v_dock", a.spacing, meta={"model": "vdock"})
    elif a.cmd == "from-mesh":
        V, F = fileio.read_mesh(a.mesh)
        V = V * a.scale
        if not a.no_auto_origin:   # gốc: tâm mặt trước (x min), giữa theo y, trên sàn (z min)
            V = V - [V[:, 0].min(), (V[:, 1].min() + V[:, 1].max()) / 2, V[:, 2].min()]
        T = tp.template_from_mesh(V, F, a.spacing, name=name or "mesh", meta={"model": "mesh"})
    else:  # from-cloud
        P = np.vstack([np.c_[d["x"], d["y"], d["z"]] for d in map(fileio.read_pcd, a.clouds)])
        P = P[np.isfinite(P).all(axis=1)]
        x, y, yaw = a.object_pose
        T_obj_cloud = inv_T(T_from_xyz_yaw(x, y, 0.0, math.radians(yaw)))
        Q = transform_points(T_obj_cloud, P)
        c = a.crop
        Q = Q[crop_box_mask(Q, (c[0], c[2], c[4]), (c[1], c[3], c[5]))]
        if len(Q) < 50:
            print(f"[x] Chỉ còn {len(Q)} điểm sau khi cắt — kiểm tra --object-pose/--crop")
            return 1
        cam = transform_points(T_obj_cloud, np.array([a.camera_pos], dtype=float))[0]
        T = tp.template_from_points(Q, cam, voxel=a.spacing, name=name or "captured",
                                    meta={"model": "captured", "n_clouds": len(a.clouds)})
    T.save(a.output)
    lo, hi = T.bounds
    print(f"Template '{T.name}': {len(T.points)} điểm, hộp bao "
          f"x[{lo[0]:.3f},{hi[0]:.3f}] y[{lo[1]:.3f},{hi[1]:.3f}] z[{lo[2]:.3f},{hi[2]:.3f}] -> {a.output}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
