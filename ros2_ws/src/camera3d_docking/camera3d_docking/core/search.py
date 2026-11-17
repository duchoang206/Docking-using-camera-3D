"""Tìm kiếm toàn cục: tương quan 2.5D đa dải độ cao + phạt vùng-phải-trống (thay FPFH/YOLO).

Ý tưởng (mở rộng kỹ thuật "ép depth thành laser scan" của bài ICARCV'18):
- Chia chiều cao template thành n_bands dải; mỗi dải chiếu điểm xuống lưới (x, y).
- Cảnh: O_b = lưới chiếm chỗ, L_b = O_b làm mờ Gauss (trường khả năng, 0..1).
- Template, với từng góc yaw: P_b = ô bề mặt, F_b = ô PHẢI TRỐNG (trước bề mặt đầu
  tiên theo +x trong mỗi cột y — ví dụ hốc càng pallet). Tường phẳng có điểm trong hốc
  -> bị phạt -> không "giả làm" pallet được.
- score(x, y, θ) = Σ L·P / Σ P  -  free_weight · Σ O·F / Σ F   (tính bằng FFT, cache phổ).
"""
from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np


@dataclass
class SearchParams:
    grid_res: float = 0.03            # kích thước ô lưới (m)
    yaw_range_deg: float = 45.0       # quét yaw trong ±range
    yaw_step_deg: float = 3.0
    n_bands: int = 3                  # số dải độ cao
    blur_sigma_cells: float = 1.0     # độ mờ trường khả năng (ô)
    free_weight: float = 0.5          # trọng số phạt vùng-phải-trống
    free_margin: float = 0.05         # vùng trống tính từ x = -free_margin
    top_k: int = 5
    nms_dist: float = 0.15
    nms_yaw_deg: float = 10.0
    min_score: float = 0.05
    z_min: float = -1e9               # bỏ phần template thấp hơn (đã bị cắt cùng sàn)


@dataclass
class Hypothesis:
    x: float
    y: float
    yaw: float
    score: float


class CorrelativeSearch:
    def __init__(self, template, params: SearchParams | None = None):
        from scipy.ndimage import gaussian_filter

        self.params = p = params or SearchParams()
        P = template.points[template.points[:, 2] >= p.z_min]
        if len(P) == 0:
            raise ValueError("Template không còn điểm nào trên z_min")
        z0, z1 = float(P[:, 2].min()), float(P[:, 2].max())
        self.edges = np.linspace(z0, z1 + 1e-6, p.n_bands + 1)
        self.z_lo, self.z_hi = z0 - 0.02, z1 + 0.02
        res = p.grid_res
        band = np.clip(np.searchsorted(self.edges, P[:, 2], side="right") - 1, 0, p.n_bands - 1)
        surf = [P[band == b, :2] for b in range(p.n_bands)]
        free = [self._free_cells(surf[b], P[:, :2], res, p.free_margin) for b in range(p.n_bands)]
        radius = max(np.linalg.norm(P[:, :2], axis=1).max(),
                     max((np.linalg.norm(f, axis=1).max() for f in free if len(f)), default=0.0))
        self.R = int(math.ceil(radius / res)) + 2
        k = 2 * self.R + 1
        n_yaw = int(round(p.yaw_range_deg / p.yaw_step_deg))
        self.yaws = np.radians(np.arange(-n_yaw, n_yaw + 1) * p.yaw_step_deg)
        self.P = np.zeros((len(self.yaws), p.n_bands, k, k), dtype=np.float32)
        self.F = np.zeros_like(self.P)
        for t, yaw in enumerate(self.yaws):
            c, s = math.cos(yaw), math.sin(yaw)
            R2 = np.array([[c, -s], [s, c]])
            for b in range(p.n_bands):
                self._raster(self.P[t, b], surf[b] @ R2.T, res)
                if len(free[b]):
                    self._raster(self.F[t, b], free[b] @ R2.T, res)
            self.F[t] *= 1.0 - np.clip(self.P[t], 0, 1)      # ô bề mặt không tính là phải trống
        self.P_sum = self.P.sum(axis=(1, 2, 3))
        self.F_sum = np.maximum(self.F.sum(axis=(1, 2, 3)), 1e-9)
        self._gauss = gaussian_filter
        self._fft_cache = {}

    # ------------------------------------------------------------------ dựng kernel
    def _raster(self, K, xy, res):
        ij = np.floor(xy / res + 0.5).astype(int) + self.R
        ok = ((ij >= 0) & (ij < K.shape[0])).all(axis=1)
        K[ij[ok, 0], ij[ok, 1]] = 1.0

    @staticmethod
    def _free_cells(surf_b, all_xy, res, margin):
        """Tâm các ô phải trống của một dải: trước bề mặt đầu tiên theo +x trong mỗi cột y."""
        y0, y1 = all_xy[:, 1].min(), all_xy[:, 1].max()
        x_max = all_xy[:, 0].max()
        ys = np.arange(y0 + res / 2, y1, res)
        cells = []
        for y in ys:
            col = surf_b[np.abs(surf_b[:, 1] - y) <= res / 2] if len(surf_b) else surf_b
            x_end = (col[:, 0].min() - 1.5 * res) if len(col) else x_max
            xs = np.arange(-margin, x_end, res)
            if len(xs):
                cells.append(np.c_[xs, np.full(len(xs), y)])
        return np.vstack(cells) if cells else np.zeros((0, 2))

    def _kernel_ffts(self, shape):
        """Phổ (rfft2) của mọi kernel lật ngược cho kích thước FFT ``shape`` (có cache)."""
        if shape not in self._fft_cache:
            flipP = self.P[:, :, ::-1, ::-1]
            flipF = self.F[:, :, ::-1, ::-1]
            self._fft_cache.clear()  # ROI cố định -> chỉ cần 1 mục; complex64 để tiết kiệm RAM
            self._fft_cache[shape] = (np.fft.rfft2(flipP, s=shape).astype(np.complex64),
                                      np.fft.rfft2(flipF, s=shape).astype(np.complex64))
        return self._fft_cache[shape]

    # ------------------------------------------------------------------ tìm kiếm
    def search(self, scene_base, roi_lo, roi_hi) -> list:
        """Trả tối đa top_k giả thuyết (x, y, yaw) của gốc frame vật trong base_link."""
        from scipy.fft import next_fast_len
        from scipy.ndimage import maximum_filter

        p = self.params
        res = p.grid_res
        S = np.asarray(scene_base, dtype=np.float64).reshape(-1, 3)
        lo = np.asarray(roi_lo, dtype=float)
        hi = np.asarray(roi_hi, dtype=float)
        S = S[(S[:, 0] >= lo[0]) & (S[:, 0] < hi[0]) & (S[:, 1] >= lo[1]) & (S[:, 1] < hi[1])
              & (S[:, 2] >= self.z_lo) & (S[:, 2] <= self.z_hi)]
        if len(S) < 10:
            return []
        H = int(math.ceil((hi[0] - lo[0]) / res))
        W = int(math.ceil((hi[1] - lo[1]) / res))
        band = np.clip(np.searchsorted(self.edges, S[:, 2], side="right") - 1, 0, p.n_bands - 1)
        ij = np.floor((S[:, :2] - lo) / res).astype(int)
        ij[:, 0] = np.clip(ij[:, 0], 0, H - 1)
        ij[:, 1] = np.clip(ij[:, 1], 0, W - 1)
        O = np.zeros((p.n_bands, H, W), dtype=np.float32)
        O[band, ij[:, 0], ij[:, 1]] = 1.0
        peak = 1.0 / (2.0 * math.pi * p.blur_sigma_cells ** 2)
        L = np.stack([np.minimum(self._gauss(O[b], p.blur_sigma_cells) / peak, 1.0)
                      for b in range(p.n_bands)]).astype(np.float32)
        k = 2 * self.R + 1
        shape = (next_fast_len(H + k - 1, real=True), next_fast_len(W + k - 1, real=True))
        FP, FF = self._kernel_ffts(shape)
        FL = np.fft.rfft2(L, s=shape)
        FO = np.fft.rfft2(O, s=shape)
        pos = np.fft.irfft2((FP * FL[None]).sum(axis=1), s=shape)
        neg = np.fft.irfft2((FF * FO[None]).sum(axis=1), s=shape)
        r = self.R
        pos = pos[:, r:r + H, r:r + W] / self.P_sum[:, None, None]
        neg = neg[:, r:r + H, r:r + W] / self.F_sum[:, None, None]
        score = pos - p.free_weight * np.clip(neg, 0.0, None)
        peaks = (score == maximum_filter(score, size=(3, 3, 3), mode="nearest")) & (score > p.min_score)
        t_idx, i_idx, j_idx = np.nonzero(peaks)
        order = np.argsort(-score[t_idx, i_idx, j_idx])
        out = []
        for o in order:
            t, i, j = t_idx[o], i_idx[o], j_idx[o]
            h = Hypothesis(lo[0] + (i + 0.5) * res, lo[1] + (j + 0.5) * res, float(self.yaws[t]),
                           float(score[t, i, j]))
            if all(math.hypot(h.x - q.x, h.y - q.y) >= p.nms_dist
                   or abs(math.degrees(h.yaw - q.yaw)) >= p.nms_yaw_deg for q in out):
                out.append(h)
                if len(out) >= p.top_k:
                    break
        return out
