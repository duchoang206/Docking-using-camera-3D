"""Task 13 — tools/make_template.py (CLI tạo template)"""
import struct

import numpy as np

from camera3d_docking.core import fileio
from camera3d_docking.core import shapes as s
from camera3d_docking.core import synthetic as syn
from camera3d_docking.core import template as tp
from camera3d_docking.tools import make_template as mt


def test_make_template_pallet_cli(tmp_path):
    out = tmp_path / "p.npz"
    assert mt.main(["pallet", "-o", str(out)]) == 0
    T = tp.Template.load(out)
    assert len(T.points) > 500 and T.meta["model"] == "pallet"


def test_make_template_rack_and_vdock(tmp_path):
    for kind in ("rack", "vdock"):
        out = tmp_path / f"{kind}.npz"
        assert mt.main([kind, "-o", str(out)]) == 0
        assert len(tp.Template.load(out).points) > 200


def test_make_template_from_cloud_cli(tmp_path):
    sc = syn.render_scene([(s.euro_pallet_boxes(), 1.5, 0.1, np.radians(5.0))], noise=None)
    P = sc["points_base"]
    f = tmp_path / "c.pcd"
    fileio.write_pcd(f, P[P[:, 2] > 0.03])
    out = tmp_path / "cap.npz"
    assert mt.main(["from-cloud", str(f), "--object-pose", "1.5", "0.1", "5",
                    "--crop", "-0.05", "1.25", "-0.45", "0.45", "0.0", "0.3",
                    "--camera-pos", "0", "0", "0.3", "-o", str(out)]) == 0
    T = tp.Template.load(out)
    lo, hi = T.bounds
    assert lo[0] > -0.06 and hi[0] < 1.26 and max(abs(lo[1]), abs(hi[1])) < 0.46
    front = T.points[:, 0] < 0.01
    assert front.sum() > 50 and np.median(T.normals[front, 0]) < -0.8


def test_cloud_saver_node_writes_pcd(tmp_path):
    import time

    import pytest

    rclpy = pytest.importorskip("rclpy")
    from geometry_msgs.msg import TransformStamped
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.parameter import Parameter
    from sensor_msgs.msg import PointCloud2
    from tf2_ros import StaticTransformBroadcaster

    from camera3d_docking.ros.cloud_saver import CloudSaverNode
    from camera3d_docking.ros.conversions import xyz_to_pointcloud2

    rclpy.init()
    try:
        node = CloudSaverNode(parameter_overrides=[Parameter("output_dir", value=str(tmp_path)),
                                                   Parameter("count", value=2)])
        helper = rclpy.create_node("saver_helper")
        tf = TransformStamped()
        tf.header.frame_id, tf.child_frame_id = "base_link", "cam"
        tf.transform.translation.z = 0.5
        tf.transform.rotation.w = 1.0
        StaticTransformBroadcaster(helper).sendTransform(tf)
        pub = helper.create_publisher(PointCloud2, "points", 10)
        ex = SingleThreadedExecutor()
        ex.add_node(node)
        ex.add_node(helper)
        P = np.random.default_rng(0).uniform(-1, 1, size=(100, 3)).astype(np.float32)
        end = time.monotonic() + 8.0
        while not node.done and time.monotonic() < end:
            pub.publish(xyz_to_pointcloud2(P, "cam", helper.get_clock().now().to_msg()))
            for _ in range(5):
                ex.spin_once(timeout_sec=0.02)
        files = sorted(tmp_path.glob("cloud_*.pcd"))
        assert node.done and len(files) == 2
        d = fileio.read_pcd(files[0])
        assert np.allclose(np.c_[d["x"], d["y"], d["z"]], P + [0, 0, 0.5], atol=1e-5)
        ex.shutdown()
        node.destroy_node()
        helper.destroy_node()
    finally:
        rclpy.shutdown()


def test_make_template_from_mesh_and_export(tmp_path):
    V = np.array([[x, y, z] for x in (0, 1) for y in (0, 1) for z in (0, 1)], float)
    quads = [(0, 1, 3, 2), (4, 6, 7, 5), (0, 4, 5, 1), (2, 3, 7, 6), (0, 2, 6, 4), (1, 5, 7, 3)]
    stl = tmp_path / "cube.stl"
    with open(stl, "wb") as f:
        tris = [t for a, b, c, d in quads for t in ((a, b, c), (a, c, d))]
        f.write(b"\0" * 80 + struct.pack("<I", len(tris)))
        for t in tris:
            f.write(struct.pack("<12fH", 0, 0, 0, *V[list(t)].ravel(), 0))
    out = tmp_path / "cube.npz"
    assert mt.main(["from-mesh", str(stl), "--scale", "0.5", "-o", str(out)]) == 0
    T = tp.Template.load(out)
    lo, hi = T.bounds
    assert np.allclose(lo, [0.0, -0.25, 0.0], atol=0.02) and hi[0] <= 0.5 + 1e-6
    pcd = tmp_path / "cube.pcd"
    assert mt.main(["export-pcd", str(out), str(pcd)]) == 0
    assert "normal_x" in fileio.read_pcd(pcd)
