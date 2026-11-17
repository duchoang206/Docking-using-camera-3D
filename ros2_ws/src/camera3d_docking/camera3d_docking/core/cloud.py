"""Xử lý đám mây điểm cơ bản (thuần numpy/scipy, vector hóa, không vòng lặp Python).

- finite_points / crop_box_mask : lọc điểm hỏng, cắt hộp ROI
- voxel_downsample              : giảm mẫu bằng trọng tâm mỗi voxel (+ lọc voxel thưa)
- estimate_normals              : pháp tuyến bằng PCA k-láng-giềng
- fit_plane_ransac              : RANSAC mặt phẳng (dùng tìm mặt sàn)
- leveling_transform            : phép quay nhỏ nhất đưa mặt sàn về z = floor_z
"""
from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

from .geometry import make_T, so3_exp


def finite_points(P: np.ndarray) -> np.ndarray:
    """Bỏ các điểm có NaN/inf."""
    P = np.asarray(P)
    if P.size == 0:
        return P.reshape(0, 3)
    return P[np.isfinite(P).all(axis=1)]


def crop_box_mask(P: np.ndarray, lo, hi) -> np.ndarray:
    """Mặt nạ các điểm nằm trong hộp [lo, hi] (bao gồm biên)."""
    lo = np.asarray(lo, dtype=P.dtype)
    hi = np.asarray(hi, dtype=P.dtype)
    return ((P >= lo) & (P <= hi)).all(axis=1)


def voxel_downsample(P: np.ndarray, voxel: float, min_points: int = 1):
    """Gộp điểm theo lưới voxel, trả (trọng tâm mỗi voxel, số điểm mỗi voxel).

    min_points > 1 sẽ bỏ các voxel thưa (lọc nhiễu "flying pixels" gần như miễn phí).
    """
    P = np.asarray(P)
    dtype = P.dtype if P.dtype in (np.float32, np.float64) else np.float64
    if len(P) == 0:
        return np.zeros((0, 3), dtype=dtype), np.zeros(0, dtype=np.int64)
    keys3 = np.floor(P / voxel).astype(np.int64)
    keys3 -= keys3.min(axis=0)
    dims = keys3.max(axis=0) + 1
    if float(dims[0]) * float(dims[1]) * float(dims[2]) < 2**62:
        key = (keys3[:, 0] * dims[1] + keys3[:, 1]) * dims[2] + keys3[:, 2]
        _, inv, counts = np.unique(key, return_inverse=True, return_counts=True)
    else:  # phạm vi quá lớn cho khóa gộp -> unique theo hàng (chậm hơn nhưng đúng)
        _, inv, counts = np.unique(keys3, axis=0, return_inverse=True, return_counts=True)
    inv = inv.ravel()
    n = len(counts)
    Q = np.empty((n, 3), dtype=np.float64)
    for i in range(3):
        Q[:, i] = np.bincount(inv, weights=P[:, i], minlength=n)
    Q /= counts[:, None]
    if min_points > 1:
        keep = counts >= min_points
        Q, counts = Q[keep], counts[keep]
    return Q.astype(dtype, copy=False), counts.astype(np.int64)


def estimate_normals(P: np.ndarray, k: int = 10, viewpoint=None) -> np.ndarray:
    """Pháp tuyến bằng PCA trên k láng giềng gần nhất; quay về phía viewpoint nếu có."""
    from scipy.spatial import cKDTree

    P = np.asarray(P, dtype=np.float64)
    if len(P) < 3:
        return np.tile([0.0, 0.0, 1.0], (len(P), 1))
    k = int(min(k, len(P)))
    _, idx = cKDTree(P).query(P, k=k)
    nb = P[idx]
    X = nb - nb.mean(axis=1, keepdims=True)
    C = np.einsum("nki,nkj->nij", X, X)
    _, vecs = np.linalg.eigh(C)
    N = vecs[:, :, 0].copy()  # vector riêng của trị riêng nhỏ nhất
    if viewpoint is not None:
        flip = ((np.asarray(viewpoint, dtype=float) - P) * N).sum(axis=1) < 0.0
        N[flip] *= -1.0
    return N


@dataclass
class PlaneFit:
    """Mặt phẳng n·p + d = 0 với |n| = 1, n_z > 0."""
    normal: np.ndarray
    d: float
    inliers: np.ndarray  # mặt nạ bool trên tập điểm đầu vào


def fit_plane_ransac(P: np.ndarray, threshold: float = 0.01, iterations: int = 100,
                     rng=None, max_score_points: int = 2000):
    """RANSAC mặt phẳng, chấm điểm vector hóa trên tập con, tinh chỉnh bằng SVD.

    Trả None nếu không đủ điểm.
    """
    P = np.asarray(P, dtype=np.float64)
    if len(P) < 3:
        return None
    rng = np.random.default_rng() if rng is None else rng
    idx = rng.integers(0, len(P), size=(iterations, 3))
    A, B, C = P[idx[:, 0]], P[idx[:, 1]], P[idx[:, 2]]
    n = np.cross(B - A, C - A)
    norm = np.linalg.norm(n, axis=1)
    ok = norm > 1e-9
    if not ok.any():
        return None
    n = n[ok] / norm[ok, None]
    d = -(n * A[ok]).sum(axis=1)
    S = P if len(P) <= max_score_points else P[rng.choice(len(P), max_score_points, replace=False)]
    scores = (np.abs(S @ n.T + d) < threshold).sum(axis=0)
    best = int(np.argmax(scores))
    nn, dd = n[best], d[best]
    inl = np.abs(P @ nn + dd) < threshold
    for _ in range(2):  # tinh chỉnh bình phương tối thiểu trên tập inlier
        if inl.sum() < 3:
            break
        Q = P[inl]
        mu = Q.mean(axis=0)
        _, _, Vt = np.linalg.svd(Q - mu, full_matrices=False)
        nn = Vt[2]
        dd = -float(nn @ mu)
        inl = np.abs(P @ nn + dd) < threshold
    if nn[2] < 0.0:
        nn, dd = -nn, -dd
    return PlaneFit(normal=nn, d=float(dd), inliers=inl)


def leveling_transform(normal, d: float, floor_z: float = 0.0) -> np.ndarray:
    """Biến đổi 4x4 đưa mặt phẳng (normal, d) về mặt z = floor_z, pháp tuyến +z.

    Dùng phép quay NHỎ NHẤT (trục = n x z) nên không thêm yaw; chỉ dịch theo z.
    """
    n = np.asarray(normal, dtype=float)
    scale = np.linalg.norm(n)
    n = n / scale
    d = float(d) / scale
    z = np.array([0.0, 0.0, 1.0])
    axis = np.cross(n, z)
    s = float(np.linalg.norm(axis))
    c = float(n @ z)
    if s < 1e-12:
        R = np.eye(3) if c > 0 else np.diag([1.0, -1.0, -1.0])
    else:
        R = so3_exp(axis / s * math.atan2(s, c))
    # Sau khi quay, mặt phẳng là z = -d  ->  dịch thêm (floor_z + d)
    return make_T(R, (0.0, 0.0, floor_z + d))
