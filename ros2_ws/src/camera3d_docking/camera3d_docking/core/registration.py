"""Khớp template với đám mây cảnh: ICP point-to-plane + thành phần tiếp tuyến.

Ẩn số: T_obj_base (đưa điểm cảnh về frame vật). Mỗi vòng nhân trái một bước nhỏ ΔT
trong frame vật với các bậc tự do chọn được (mặc định x, y, z, yaw — vật đứng trên sàn).

Chi phí cho mỗi tương ứng (q điểm cảnh, m điểm template, n pháp tuyến, e = q - m):
    w·(n·e)² + ε·w_t·‖e‖²
Riêng point-to-plane sẽ trượt ngang khi mặt vật nhìn thẳng vào camera; số hạng tiếp
tuyến ε kéo theo các mép/hốc nên khử được trượt đó.

Tối ưu cho CPU yếu: tương ứng tra bằng LƯỚI TÍNH SẴN (O(1)/điểm) ở các tầng thô,
KD-tree chính xác ở tầng cuối.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field

import numpy as np

from .cloud import crop_box_mask
from .geometry import inv_T, make_T, rot_z, skew, so3_exp, transform_points

# chỉ số cột trong vector tham số [tx, ty, tz, rx, ry, rz]
_DOF = {"xy_yaw": (0, 1, 5), "xyz_yaw": (0, 1, 2, 5), "6dof": (0, 1, 2, 3, 4, 5)}


class NearestTemplateGrid:
    """Lưới 3D chứa chỉ số điểm template gần nhất với tâm mỗi ô -> tra cứu O(1).

    Sai số so với KD-tree chính xác <= 0.87 * res (nửa đường chéo ô).
    """

    def __init__(self, points, resolution: float = 0.01, margin: float = 0.25,
                 max_cells: int = 4_000_000):
        from scipy.spatial import cKDTree

        P = np.asarray(points, dtype=np.float64)
        lo = P.min(axis=0) - margin
        hi = P.max(axis=0) + margin
        res = float(resolution)
        while True:
            dims = np.ceil((hi - lo) / res).astype(np.int64)
            if int(np.prod(dims)) <= max_cells:
                break
            res *= 1.2
        self.lo, self.res, self.dims = lo, res, dims
        tree = cKDTree(P)
        grid = np.empty(tuple(int(d) for d in dims), dtype=np.int32)
        ys = lo[1] + (np.arange(dims[1]) + 0.5) * res
        zs = lo[2] + (np.arange(dims[2]) + 0.5) * res
        YY, ZZ = np.meshgrid(ys, zs, indexing="ij")
        yz = np.c_[YY.ravel(), ZZ.ravel()]
        chunk = max(1, 1_000_000 // len(yz))
        for i0 in range(0, int(dims[0]), chunk):
            xs = lo[0] + (np.arange(i0, min(int(dims[0]), i0 + chunk)) + 0.5) * res
            C = np.c_[np.repeat(xs, len(yz)), np.tile(yz, (len(xs), 1))]
            _, idx = tree.query(C, workers=-1)
            grid[i0:i0 + len(xs)] = idx.reshape(len(xs), dims[1], dims[2])
        self.grid = grid

    def query(self, Q) -> np.ndarray:
        """Chỉ số điểm template gần nhất cho mỗi điểm Q; -1 nếu nằm ngoài lưới."""
        ijk = np.floor((np.asarray(Q, dtype=np.float64) - self.lo) / self.res).astype(np.int64)
        ok = ((ijk >= 0) & (ijk < self.dims)).all(axis=1)
        out = np.full(len(ijk), -1, dtype=np.int64)
        if ok.any():
            k = ijk[ok]
            out[ok] = self.grid[k[:, 0], k[:, 1], k[:, 2]]
        return out


@dataclass
class IcpParams:
    max_corr_dists: tuple = (0.15, 0.06, 0.03)   # khoảng cách tương ứng tối đa từng tầng (m)
    max_iters: int = 15                          # số vòng tối đa mỗi tầng
    dof: str = "xyz_yaw"                         # "xy_yaw" | "xyz_yaw" | "6dof"
    tangent_weight: float = 0.0                  # ε số hạng tiếp tuyến (đo thực nghiệm: không lợi, chậm hơn)
    kernel: str = "tukey"                        # "tukey" | "huber" | "none"
    kernel_min_scale: float = 0.01               # tỷ lệ nhỏ nhất của kernel bền vững (m)
    lm_lambda: float = 1e-4                      # giảm chấn Levenberg–Marquardt
    eps_t: float = 1e-4                          # điều kiện hội tụ tịnh tiến (m)
    eps_r: float = 1e-4                          # điều kiện hội tụ quay (rad)
    max_step_t: float = 0.05                     # giới hạn bước tịnh tiến mỗi vòng (m)
    max_step_r: float = math.radians(5.0)        # giới hạn bước quay mỗi vòng
    exact_final_stage: bool = True               # tầng cuối dùng KD-tree chính xác
    min_corr: int = 30                           # số tương ứng tối thiểu
    inlier_dist: float = 0.02                    # |n·e| < inlier_dist là inlier
    coverage_radius: float = 0.03                # bán kính tính coverage
    coverage_min_z: float = -1e9                 # bỏ điểm template thấp hơn (đã bị cắt sàn)
    coverage_min_view_cos: float = 0.26          # bỏ mặt nhìn quá xiên (> ~75°) khỏi coverage
    crop_margin: float = 0.2                     # cắt cảnh quanh hộp bao template
    metric_margin: float = 0.05                  # vùng cảnh dùng tính chỉ số chất lượng
    cov_inflation: float = 25.0                  # phóng đại hiệp phương sai ICP (thận trọng)
    obs_shift: float = 0.03                      # dịch thử ngang khi đo khả năng quan sát (m)
    obs_yaw: float = math.radians(1.5)           # xoay thử khi đo khả năng quan sát
    min_observability: float = 0.008             # dưới ngưỡng -> suy biến


@dataclass
class IcpResult:
    T_base_obj: np.ndarray
    converged: bool = False
    iterations: int = 0
    rmse: float = float("inf")
    inlier_ratio: float = 0.0
    coverage: float = 0.0
    n_corr: int = 0
    cov_xyyaw: np.ndarray = field(default_factory=lambda: np.eye(3))
    sigma_xy: float = float("inf")
    sigma_yaw: float = float("inf")
    observability: float = 0.0
    degenerate: bool = True

    @property
    def score(self) -> float:
        return float(self.coverage * self.inlier_ratio)


def _robust_weights(u: np.ndarray, kernel: str) -> np.ndarray:
    a = np.abs(u)
    if kernel == "tukey":
        return np.where(a < 1.0, (1.0 - a * a) ** 2, 0.0)
    if kernel == "huber":
        return np.where(a <= 1.0, 1.0, 1.0 / np.maximum(a, 1e-12))
    return np.ones_like(u)


def _kernel_scale(r: np.ndarray, p: IcpParams, dmax: float) -> float:
    """Tỷ lệ kernel bền vững cho tầng có khoảng tương ứng ``dmax``.

    - Đo độ lớn residual QUANH 0 (median |r|), không quanh trung vị: khi còn lệch hệ thống
      (mọi r ≈ 10 cm) MAD-quanh-trung-vị rất nhỏ và sẽ loại hết mọi điểm -> ICP đứng yên.
    - Không nhỏ hơn ``dmax`` của tầng (annealing thô -> tinh): các điểm ở mép/hốc mang thông
      tin ngang có residual lớn hơn đa số điểm mặt trước; kernel quá hẹp sẽ loại chúng.
    """
    med = float(np.median(np.abs(r))) if len(r) else 0.0
    c = 4.685 if p.kernel == "tukey" else 1.345
    return max(c * 1.4826 * med, p.kernel_min_scale, dmax)


def _correspond(Q, template, tree, nn, exact, dmax):
    """Trả (q, chỉ số template) cho các điểm có tương ứng trong vòng dmax."""
    if exact or nn is None:
        d, idx = tree.query(Q, distance_upper_bound=dmax)
        ok = np.isfinite(d)
        return Q[ok], idx[ok]
    idx = nn.query(Q)
    ok = idx >= 0
    q, idx = Q[ok], idx[ok]
    e = q - template.points[idx]
    ok = np.einsum("ij,ij->i", e, e) <= dmax * dmax
    return q[ok], idx[ok]


def _normal_equations(q, n, e, r_n, w, wt, eps):
    """H, g cho tham số [t, ω]: chi phí Σ w r_n² + ε Σ w_t ‖e‖²."""
    J = np.hstack([n, np.cross(q, n)])
    Jw = J * w[:, None]
    H = J.T @ Jw
    g = Jw.T @ r_n
    if eps > 0.0:
        sw = float(wt.sum())
        swq = (wt[:, None] * q).sum(axis=0)
        Ht = np.zeros((6, 6))
        Ht[:3, :3] = sw * np.eye(3)
        Ht[:3, 3:] = -skew(swq)
        Ht[3:, :3] = skew(swq)
        Ht[3:, 3:] = float((wt * (q * q).sum(axis=1)).sum()) * np.eye(3) - (q * wt[:, None]).T @ q
        gt = np.zeros(6)
        gt[:3] = (wt[:, None] * e).sum(axis=0)
        gt[3:] = (wt[:, None] * np.cross(q, e)).sum(axis=0)
        H = H + eps * Ht
        g = g + eps * gt
    return H, g


def _profile_cost(q, template, tree, nn, c):
    """Chi phí bình phương cắt ngưỡng (point-to-point) — dùng đo khả năng quan sát."""
    if nn is not None:
        idx = nn.query(q)
        d2 = np.full(len(q), c * c)
        ok = idx >= 0
        diff = q[ok] - template.points[idx[ok]]
        d2[ok] = np.minimum(np.einsum("ij,ij->i", diff, diff), c * c)
    else:
        d, _ = tree.query(q, distance_upper_bound=c)
        d2 = np.where(np.isfinite(d), d * d, c * c)
    return float(d2.mean())


def icp_point_to_plane(scene_base, template, T_init, params: IcpParams | None = None, nn=None,
                       camera_pos_base=None) -> IcpResult:
    """Khớp ``template`` vào ``scene_base`` (điểm trong base_link) từ pose ban đầu ``T_init``.

    ``T_init``/kết quả là T_base_obj (pose của vật trong base_link). ``nn`` là
    NearestTemplateGrid (tùy chọn, tăng tốc); ``camera_pos_base`` để tính coverage đúng phía.
    """
    p = params or IcpParams()
    S = np.asarray(scene_base, dtype=np.float64).reshape(-1, 3)
    T_ob = inv_T(np.asarray(T_init, dtype=np.float64))
    lo, hi = template.bounds
    lo_c, hi_c = lo - p.crop_margin, hi + p.crop_margin
    cols = list(_DOF[p.dof])
    tree = template.kdtree()
    iters = 0
    converged = False
    n_stages = len(p.max_corr_dists)
    for stage, dmax in enumerate(p.max_corr_dists):
        exact = p.exact_final_stage and stage == n_stages - 1
        stage_conv = False
        for _ in range(p.max_iters):
            Q = transform_points(T_ob, S)
            Q = Q[crop_box_mask(Q, lo_c, hi_c)]
            if len(Q) < p.min_corr:
                break
            q, idx = _correspond(Q, template, tree, nn, exact, dmax)
            if len(q) < p.min_corr:
                break
            iters += 1
            n = template.normals[idx]
            e = q - template.points[idx]
            r_n = np.einsum("ij,ij->i", n, e)
            w = _robust_weights(r_n / _kernel_scale(r_n, p, dmax), p.kernel)
            wt = w * _robust_weights(np.linalg.norm(e, axis=1) / dmax, "tukey")
            H, g = _normal_equations(q, n, e, r_n, w, wt, p.tangent_weight)
            Hr = H[np.ix_(cols, cols)]
            Hr = Hr + p.lm_lambda * np.diag(np.diag(Hr)) + 1e-12 * np.eye(len(cols))
            try:
                d = -np.linalg.solve(Hr, g[cols])
            except np.linalg.LinAlgError:
                break
            full = np.zeros(6)
            full[cols] = d
            tn = np.linalg.norm(full[:3])
            if tn > p.max_step_t:
                full[:3] *= p.max_step_t / tn
            rn = np.linalg.norm(full[3:])
            if rn > p.max_step_r:
                full[3:] *= p.max_step_r / rn
            T_ob = make_T(so3_exp(full[3:]), full[:3]) @ T_ob
            if np.linalg.norm(full[:3]) < p.eps_t and np.linalg.norm(full[3:]) < p.eps_r:
                stage_conv = True
                break
        converged = stage_conv
    return _evaluate(S, T_ob, template, tree, nn, p, cols, camera_pos_base, iters, converged)


def _evaluate(S, T_ob, template, tree, nn, p, cols, camera_pos_base, iters, converged):
    """Chỉ số chất lượng, hiệp phương sai và khả năng quan sát tại nghiệm."""
    T_bo = inv_T(T_ob)
    res = IcpResult(T_base_obj=T_bo, converged=converged, iterations=iters)
    lo, hi = template.bounds
    Q = transform_points(T_ob, S)
    q = Q[crop_box_mask(Q, lo - p.metric_margin, hi + p.metric_margin)]
    if len(q) < p.min_corr:
        res.converged = False
        return res
    d, idx = tree.query(q)
    n = template.normals[idx]
    e = q - template.points[idx]
    r_n = np.einsum("ij,ij->i", n, e)
    inl = (d < 1.5 * p.inlier_dist) & (np.abs(r_n) < p.inlier_dist)
    res.inlier_ratio = float(inl.mean())
    res.rmse = float(np.sqrt(np.mean(r_n[inl] ** 2))) if inl.any() else float("inf")
    # --- coverage: tỷ lệ điểm template (quay về camera) có điểm cảnh gần
    cand = template.points[:, 2] >= p.coverage_min_z
    if camera_pos_base is not None:
        cam_obj = transform_points(T_ob, np.asarray(camera_pos_base, dtype=float).reshape(1, 3))[0]
        cand &= template.facing_mask(cam_obj, p.coverage_min_view_cos)
    if cand.any():
        from scipy.spatial import cKDTree

        dd, _ = cKDTree(q).query(template.points[cand], distance_upper_bound=p.coverage_radius)
        res.coverage = float(np.isfinite(dd).mean())
    # --- hiệp phương sai từ Hessian tại nghiệm (tương ứng chính xác, tầng cuối)
    dmax = p.max_corr_dists[-1]
    sel = d <= dmax
    res.n_corr = int(sel.sum())
    if res.n_corr >= p.min_corr:
        qs, ns, es, rs = q[sel], n[sel], e[sel], r_n[sel]
        w = _robust_weights(rs / _kernel_scale(rs, p, dmax), p.kernel)
        wt = w * _robust_weights(np.linalg.norm(es, axis=1) / dmax, "tukey")
        H, _ = _normal_equations(qs, ns, es, rs, w, wt, p.tangent_weight)
        Hr = H[np.ix_(cols, cols)] + 1e-12 * np.eye(len(cols))
        dof_eff = max(float(w.sum()) - len(cols), 1.0)
        sigma2 = max(float((w * rs * rs).sum()) / dof_eff, (0.002) ** 2)
        C = np.zeros((6, 6))
        try:
            C[np.ix_(cols, cols)] = np.linalg.inv(Hr) * sigma2 * p.cov_inflation
        except np.linalg.LinAlgError:
            C[np.ix_(cols, cols)] = np.eye(len(cols))
        R = T_bo[:3, :3]
        J = np.zeros((3, 6))
        J[0, :3] = -R[0]
        J[1, :3] = -R[1]
        J[2, 3:] = -R[2]
        cov = J @ C @ J.T
        res.cov_xyyaw = 0.5 * (cov + cov.T) + np.diag([1e-10, 1e-10, 1e-12])
        res.sigma_xy = float(math.sqrt(max(np.linalg.eigvalsh(res.cov_xyyaw[:2, :2]).max(), 0.0)))
        res.sigma_yaw = float(math.sqrt(max(res.cov_xyyaw[2, 2], 0.0)))
    # --- khả năng quan sát: chi phí có tăng khi dịch ngang / xoay thử không?
    c = p.coverage_radius
    f0 = _profile_cost(q, template, tree, nn, c)
    shift = np.array([0.0, p.obs_shift, 0.0])
    f_y = min(_profile_cost(q - shift, template, tree, nn, c),
              _profile_cost(q + shift, template, tree, nn, c))
    f_r = min(_profile_cost(q @ rot_z(p.obs_yaw).T, template, tree, nn, c),
              _profile_cost(q @ rot_z(-p.obs_yaw).T, template, tree, nn, c))
    res.observability = float(min(f_y - f0, f_r - f0) / (c * c))
    res.degenerate = bool(res.observability < p.min_observability)
    return res
