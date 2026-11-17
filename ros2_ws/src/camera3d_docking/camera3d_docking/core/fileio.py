"""Đọc/ghi file đám mây điểm và mesh — không cần Open3D/PCL.

- PCD (ascii, binary)  : đọc/ghi; binary_compressed -> ValueError có hướng dẫn
- STL (ascii, binary), OBJ (v/f, tam giác hóa đa giác) : đọc mesh từ CAD
"""
from __future__ import annotations

import re
from pathlib import Path

import numpy as np

_PCD_TYPES = {("F", 4): "<f4", ("F", 8): "<f8", ("U", 1): "<u1", ("U", 2): "<u2",
              ("U", 4): "<u4", ("I", 1): "<i1", ("I", 2): "<i2", ("I", 4): "<i4",
              ("U", 8): "<u8", ("I", 8): "<i8"}


def read_pcd(path) -> dict:
    """Đọc PCD, trả dict {tên trường: mảng}. Trường có COUNT>1 trả mảng (N, COUNT)."""
    raw = Path(path).read_bytes()
    header = {}
    pos = 0
    while True:
        end = raw.index(b"\n", pos)
        line = raw[pos:end].decode("ascii", errors="replace").strip()
        pos = end + 1
        if not line or line.startswith("#"):
            continue
        key, *vals = line.split()
        header[key.upper()] = vals
        if key.upper() == "DATA":
            break
    fields = header["FIELDS"]
    sizes = [int(v) for v in header["SIZE"]]
    types = header["TYPE"]
    counts = [int(v) for v in header.get("COUNT", ["1"] * len(fields))]
    n = int(header["POINTS"][0])
    kind = header["DATA"][0].lower()
    if kind == "binary_compressed":
        raise ValueError("PCD 'binary_compressed' chưa được hỗ trợ — hãy lưu lại ở dạng "
                         "binary hoặc ascii (CloudCompare: Save As > PCD, bỏ chọn compressed).")
    dtype = np.dtype([(f if f != "_" else f"_pad{i}", _PCD_TYPES[(t, sz)], (c,) if c > 1 else ())
                      for i, (f, sz, t, c) in enumerate(zip(fields, sizes, types, counts))])
    if kind == "binary":
        data = np.frombuffer(raw, dtype=dtype, count=n, offset=pos)
        return {name: np.array(data[name]) for name in dtype.names if not name.startswith("_pad")}
    if kind == "ascii":
        text = raw[pos:].decode("ascii")
        table = np.loadtxt(text.splitlines(), ndmin=2) if n > 0 else np.zeros((0, sum(counts)))
        out, col = {}, 0
        for f, c in zip(fields, counts):
            if f != "_":
                out[f] = table[:, col] if c == 1 else table[:, col:col + c]
            col += c
        return out
    raise ValueError(f"Kiểu DATA PCD không hỗ trợ: {kind}")


def write_pcd(path, points, normals=None, binary: bool = True) -> None:
    """Ghi PCD gồm x y z (+ normal_x normal_y normal_z) float32."""
    P = np.asarray(points, dtype=np.float32).reshape(-1, 3)
    cols = [P]
    fields = ["x", "y", "z"]
    if normals is not None:
        cols.append(np.asarray(normals, dtype=np.float32).reshape(-1, 3))
        fields += ["normal_x", "normal_y", "normal_z"]
    data = np.hstack(cols).astype(np.float32)
    k = len(fields)
    header = (
        "# .PCD v0.7 - camera3d_docking\nVERSION .7\n"
        f"FIELDS {' '.join(fields)}\nSIZE {' '.join(['4'] * k)}\nTYPE {' '.join(['F'] * k)}\n"
        f"COUNT {' '.join(['1'] * k)}\nWIDTH {len(data)}\nHEIGHT 1\n"
        f"VIEWPOINT 0 0 0 1 0 0 0\nPOINTS {len(data)}\nDATA {'binary' if binary else 'ascii'}\n"
    )
    with open(path, "wb") as f:
        f.write(header.encode("ascii"))
        if binary:
            f.write(np.ascontiguousarray(data, dtype="<f4").tobytes())
        else:
            np.savetxt(f, data, fmt="%.7g")


def read_mesh(path):
    """Đọc mesh tam giác từ .stl (ascii/binary) hoặc .obj. Trả (V (M,3), F (K,3) int)."""
    path = Path(path)
    ext = path.suffix.lower()
    if ext == ".stl":
        return _read_stl(path)
    if ext == ".obj":
        return _read_obj(path)
    raise ValueError(f"Định dạng mesh không hỗ trợ: {ext} (dùng .stl hoặc .obj)")


def _read_stl(path: Path):
    raw = path.read_bytes()
    if len(raw) >= 84:
        n = int(np.frombuffer(raw, dtype="<u4", count=1, offset=80)[0])
        if len(raw) == 84 + 50 * n:  # STL nhị phân
            dt = np.dtype([("n", "<f4", (3,)), ("v", "<f4", (3, 3)), ("a", "<u2")])
            tris = np.frombuffer(raw, dtype=dt, count=n, offset=84)["v"].astype(np.float64)
            return tris.reshape(-1, 3), np.arange(3 * n).reshape(n, 3)
    text = raw.decode("ascii", errors="replace")
    num = r"[-+]?(?:\d+\.?\d*|\.\d+)(?:[eE][-+]?\d+)?"
    verts = re.findall(rf"vertex\s+({num})\s+({num})\s+({num})", text)
    V = np.array(verts, dtype=np.float64).reshape(-1, 3)
    n = len(V) // 3
    return V[: 3 * n], np.arange(3 * n).reshape(n, 3)


def _read_obj(path: Path):
    V, F = [], []
    for line in path.read_text(errors="replace").splitlines():
        parts = line.split()
        if not parts:
            continue
        if parts[0] == "v":
            V.append([float(x) for x in parts[1:4]])
        elif parts[0] == "f":
            idx = []
            for tok in parts[1:]:
                i = int(tok.split("/")[0])
                idx.append(i - 1 if i > 0 else len(V) + i)
            for k in range(1, len(idx) - 1):  # tam giác hóa kiểu quạt
                F.append([idx[0], idx[k], idx[k + 1]])
    return np.array(V, dtype=np.float64).reshape(-1, 3), np.array(F, dtype=np.int64).reshape(-1, 3)
