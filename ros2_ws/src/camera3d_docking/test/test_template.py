"""Task 4 — core/fileio.py + core/template.py"""
import struct

import numpy as np
import pytest

from camera3d_docking.core import fileio as fio
from camera3d_docking.core import shapes as s
from camera3d_docking.core import template as tp


def _cube_mesh():
    V = np.array([[x, y, z] for x in (0, 1) for y in (0, 1) for z in (0, 1)], float)
    quads = [(0, 1, 3, 2), (4, 6, 7, 5), (0, 4, 5, 1), (2, 3, 7, 6), (0, 2, 6, 4), (1, 5, 7, 3)]
    F = []
    for a, b, c, d in quads:  # tam giác hóa, pháp tuyến hướng ra ngoài
        F += [(a, b, c), (a, c, d)]
    return V, np.array(F)


def _write_binary_stl(path, V, F):
    with open(path, "wb") as f:
        f.write(b"\0" * 80 + struct.pack("<I", len(F)))
        for tri in F:
            f.write(struct.pack("<12fH", 0, 0, 0, *V[tri].ravel(), 0))


def test_template_from_boxes_only_visible():
    T = tp.template_from_boxes([s.Box((0.25, 0.0, 0.3), (0.5, 0.6, 0.6))], name="box", spacing=0.01)
    assert len(T.points) > 200
    back = T.points[:, 0] > 0.499
    assert not np.any(np.isclose(T.normals[back, 0], 1.0))  # mặt sau (pháp tuyến +x) không thấy được
    assert (T.normals[:, 0] <= 1e-6).all()        # chỉ mặt trước/mặt bên
    assert np.allclose(np.linalg.norm(T.normals, axis=1), 1.0)


def test_template_pallet_metadata_and_bounds():
    T = tp.template_from_boxes(s.euro_pallet_boxes(), name="euro_pallet")
    lo, hi = T.bounds
    assert lo[0] > -0.01 and lo[2] > -1e-6 and hi[1] <= 0.4 + 1e-6 and hi[2] <= 0.144 + 1e-6
    assert T.name == "euro_pallet" and T.meta["source"] == "boxes"
    assert len(T.points) > 500


def test_template_save_load_npz_roundtrip(tmp_path):
    T = tp.template_from_boxes(s.v_dock_boxes(), name="v_dock")
    f = tmp_path / "v.npz"
    T.save(f)
    T2 = tp.Template.load(f)
    assert np.allclose(T2.points, T.points) and np.allclose(T2.normals, T.normals)
    assert T2.name == "v_dock" and T2.meta == T.meta


def test_template_save_load_pcd_roundtrip(tmp_path):
    T = tp.template_from_boxes(s.v_dock_boxes(), name="v_dock")
    f = tmp_path / "v.pcd"
    T.save(f)
    T2 = tp.Template.load(f)
    assert np.allclose(T2.points, T.points, atol=1e-6)
    assert np.allclose(T2.normals, T.normals, atol=1e-5)


def test_pcd_roundtrip_ascii_and_binary(tmp_path):
    rng = np.random.default_rng(0)
    P = rng.normal(size=(100, 3))
    N = rng.normal(size=(100, 3))
    N /= np.linalg.norm(N, axis=1, keepdims=True)
    for binary in (False, True):
        f = tmp_path / f"c_{binary}.pcd"
        fio.write_pcd(f, P, N, binary=binary)
        d = fio.read_pcd(f)
        assert np.allclose(np.c_[d["x"], d["y"], d["z"]], P, atol=1e-5)
        assert np.allclose(np.c_[d["normal_x"], d["normal_y"], d["normal_z"]], N, atol=1e-5)


def test_read_pcd_rejects_compressed(tmp_path):
    f = tmp_path / "c.pcd"
    f.write_bytes(b"VERSION .7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\n"
                  b"WIDTH 1\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\nPOINTS 1\nDATA binary_compressed\n")
    with pytest.raises(ValueError):
        fio.read_pcd(f)


def test_read_binary_stl_cube(tmp_path):
    V, F = _cube_mesh()
    f = tmp_path / "cube.stl"
    _write_binary_stl(f, V, F)
    V2, F2 = fio.read_mesh(f)
    assert F2.shape == (12, 3)
    assert V2.min() >= 0.0 and V2.max() <= 1.0


def test_read_obj_quads_triangulated(tmp_path):
    f = tmp_path / "q.obj"
    f.write_text("v 0 0 0\nv 1 0 0\nv 1 1 0\nv 0 1 0\nf 1/1/1 2/2/2 3/3/3 4/4/4\n")
    V, F = fio.read_mesh(f)
    assert V.shape == (4, 3) and F.tolist() == [[0, 1, 2], [0, 2, 3]]


def test_template_from_mesh_culls_back_faces():
    V, F = _cube_mesh()
    T = tp.template_from_mesh(V, F, spacing=0.02, name="cube")
    assert (T.normals[:, 0] <= 1e-6).all()
    assert (T.normals[:, 2] >= -0.5).all()                    # bỏ mặt đáy
    assert (np.isclose(T.normals[:, 0], -1.0)).sum() > 100     # có mặt trước


def test_template_from_points_normals_toward_viewpoint():
    yy, zz = np.meshgrid(np.linspace(-0.3, 0.3, 31), np.linspace(0.0, 0.4, 21))
    P = np.c_[np.zeros(yy.size), yy.ravel(), zz.ravel()]
    T = tp.template_from_points(P, viewpoint_obj=(-1.0, 0.0, 0.3), voxel=0.02, name="plate")
    assert np.allclose(T.normals, [-1.0, 0.0, 0.0], atol=0.05)


def test_facing_mask():
    T = tp.template_from_boxes([s.Box((0.25, 0.0, 0.3), (0.5, 0.6, 0.6))], name="box")
    m = T.facing_mask((-1.0, 0.0, 0.3))
    front = np.isclose(T.normals[:, 0], -1.0)
    assert m[front].all()
    side = np.isclose(np.abs(T.normals[:, 1]), 1.0)
    assert not m[side].any()
