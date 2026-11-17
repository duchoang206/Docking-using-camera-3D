#!/usr/bin/env python3
"""Sinh hình minh họa (bản sáng + tối) cho sổ tay Docking Camera 3D.

Mọi số liệu trong hình được tính bằng CHÍNH code lõi của dự án (camera3d_docking.core).
Chạy (sau `source env.sh`):  python3 figs.py [f01 f02 ... | all]
"""
from __future__ import annotations

import json
import math
import os
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
PKG = os.path.abspath(os.path.join(HERE, "..", "..", "..", "ros2_ws", "src", "camera3d_docking"))
sys.path.insert(0, PKG)

import numpy as np  # noqa: E402
import matplotlib  # noqa: E402

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
from matplotlib.patches import Polygon, Rectangle  # noqa: E402
from mpl_toolkits.mplot3d import Axes3D  # noqa: E402,F401
from PIL import Image  # noqa: E402

from camera3d_docking.core import cloud as c  # noqa: E402
from camera3d_docking.core import estimator as es  # noqa: E402
from camera3d_docking.core import geometry as g  # noqa: E402
from camera3d_docking.core import registration as reg  # noqa: E402
from camera3d_docking.core import shapes as s  # noqa: E402
from camera3d_docking.core import synthetic as syn  # noqa: E402
from camera3d_docking.core import template as tp  # noqa: E402

OUT = os.environ.get("GUIDE_OUT", os.path.join(HERE, "..", "img"))     # ảnh xuất vào docs/huong-dan/img
META_PATH = os.path.join(HERE, "data", "figs_meta.json")
DPI = 130

THEMES = {
    "light": dict(bg="#FFFFFF", ink="#1B2430", muted="#5E6B78", grid="#E6EBF0", scene="#3B7DB0",
                  tpl="#C47F0C", init="#C2462B", ok="#2F8F5B", bad="#C2462B", floor="#A9B4BF",
                  faint="#CCD4DC", pane=(0.97, 0.975, 0.98, 1.0)),
    "dark": dict(bg="#161D24", ink="#E4E9EE", muted="#95A3B1", grid="#27313B", scene="#6CB4E4",
                 tpl="#F0B347", init="#F07A5F", ok="#5CCB8A", bad="#F07A5F", floor="#5A6773",
                 faint="#3A4652", pane=(0.11, 0.14, 0.17, 1.0)),
}

PALLET = s.euro_pallet_boxes()
_cache: dict = {}
META: dict = {}


# ============================================================================ tiện ích
def style(th):
    plt.rcParams.update({
        "figure.facecolor": th["bg"], "axes.facecolor": th["bg"], "savefig.facecolor": th["bg"],
        "text.color": th["ink"], "axes.labelcolor": th["ink"], "axes.edgecolor": th["muted"],
        "xtick.color": th["muted"], "ytick.color": th["muted"], "grid.color": th["grid"],
        "axes.grid": True, "grid.linewidth": 0.6, "font.size": 9.5, "axes.titlesize": 10.5,
        "axes.titleweight": "bold", "axes.titlelocation": "left", "axes.spines.top": False,
        "axes.spines.right": False, "legend.frameon": False, "font.family": "DejaVu Sans",
        "legend.fontsize": 8.5, "axes.labelsize": 9.5,
    })


def style3d(ax, th):
    for axis in (ax.xaxis, ax.yaxis, ax.zaxis):
        axis.set_pane_color(th["pane"])
        axis._axinfo["grid"].update(color=th["grid"], linewidth=0.5)
        axis.line.set_color(th["muted"])
        axis.label.set_color(th["ink"])
    ax.tick_params(colors=th["muted"], labelsize=7.5)


def save(fig, name, theme):
    os.makedirs(OUT, exist_ok=True)
    png = os.path.join(OUT, f"{name}-{theme}.png")
    fig.savefig(png, dpi=DPI, bbox_inches="tight", pad_inches=0.10)
    plt.close(fig)
    im = Image.open(png)
    size = im.size
    im.save(png[:-4] + ".webp", "WEBP", quality=86, method=6)
    os.remove(png)
    META.setdefault(name, {})["size"] = list(size)
    return size


def topview(ax, xlim=(0.0, 3.6), ylim=(-1.6, 1.6)):
    """Nhìn từ trên xuống: trục dọc = x (robot nhìn lên trên), trục ngang = y (+y bên trái)."""
    ax.set_xlim(ylim[1], ylim[0])
    ax.set_ylim(*xlim)
    ax.set_aspect("equal")
    ax.set_xlabel("y (m)  ← trái | phải →")
    ax.set_ylabel("x (m)  phía trước ↑")


def draw_robot(ax, th, x=0.0, y=0.0, yaw=0.0, L=(-0.32, 0.18), W=0.44, label=True):
    corners = np.array([[L[0], -W / 2], [L[1], -W / 2], [L[1], W / 2], [L[0], W / 2]])
    R = np.array([[math.cos(yaw), -math.sin(yaw)], [math.sin(yaw), math.cos(yaw)]])
    P = corners @ R.T + [x, y]
    ax.add_patch(Polygon(P[:, ::-1], closed=True, fill=True, fc=th["faint"], ec=th["muted"], lw=1.0,
                         zorder=5))
    tip = np.array([0.30, 0.0]) @ R.T + [x, y]
    ax.annotate("", xy=(tip[1], tip[0]), xytext=(y, x),
                arrowprops=dict(arrowstyle="-|>", color=th["ink"], lw=1.2), zorder=6)
    if label:
        ax.text(y - 0.28, x - 0.1, "robot", color=th["muted"], fontsize=8, ha="left", va="center")


def footprint(T_or_pose, length=1.2, width=0.8):
    """Đường bao (y, x) của pallet trong base_link từ pose vật (gốc = tâm mặt trước)."""
    if isinstance(T_or_pose, np.ndarray) and T_or_pose.shape == (4, 4):
        x, y, yaw = g.pose2d(T_or_pose)
    else:
        x, y, yaw = T_or_pose
    pts = np.array([[0, -width / 2], [length, -width / 2], [length, width / 2], [0, width / 2]])
    R = np.array([[math.cos(yaw), -math.sin(yaw)], [math.sin(yaw), math.cos(yaw)]])
    P = pts @ R.T + [x, y]
    return P[:, ::-1]


def pallet_template():
    if "tpl" not in _cache:
        _cache["tpl"] = tp.template_from_boxes(PALLET, "euro_pallet")
    return _cache["tpl"]


def demo_scene():
    """Cảnh minh họa chung: pallet trước tường, 2 hộp gây nhiễu, nhiễu depth kiểu stereo."""
    if "demo" not in _cache:
        extra = [s.Box((3.85, 0.0, 1.0), (0.10, 6.0, 2.0)),          # tường phía sau (ngoài ROI)
                 s.Box((1.05, -0.95, 0.20), (0.40, 0.40, 0.40)),       # thùng bên phải
                 s.Box((2.45, 1.05, 0.35), (0.50, 0.50, 0.70))]        # thùng bên trái
        sc = syn.render_scene([(PALLET, 1.6, 0.10, math.radians(8.0))], extra_boxes=extra,
                              noise=s.DepthNoise(), rng=np.random.default_rng(7))
        sc["truth"] = (1.6, 0.10, math.radians(8.0))
        org2 = sc["organized_optical"][::2, ::2].reshape(-1, 3)      # cloud_stride = 2 như node
        org2 = org2[np.isfinite(org2).all(axis=1)]
        sc["points_stride2"] = g.transform_points(sc["T_base_optical"], org2).astype(np.float32)
        _cache["demo"] = sc
    return _cache["demo"]


def estimator():
    if "est" not in _cache:
        _cache["est"] = es.DockPoseEstimator(pallet_template(), es.EstimatorParams())
    return _cache["est"]


# ============================================================================ F01 đám mây điểm
def f01(th, theme):
    sc = demo_scene()
    org = sc["organized_optical"]
    depth = org[..., 2]
    fig = plt.figure(figsize=(11.0, 4.1))
    ax = fig.add_subplot(1, 2, 1)
    ax.grid(False)
    im = ax.imshow(np.where(np.isfinite(depth), depth, np.nan), cmap="viridis", vmin=0.3, vmax=3.4)
    ax.set_title("A. Ảnh độ sâu 424×240 (mỗi pixel → 1 điểm)")
    ax.set_xlabel("cột u (pixel)")
    ax.set_ylabel("hàng v (pixel)")
    cb = fig.colorbar(im, ax=ax, fraction=0.035, pad=0.02)
    cb.set_label("độ sâu z (m)")
    cb.ax.yaxis.set_tick_params(color=th["muted"])
    cb.outline.set_edgecolor(th["muted"])
    n_nan = int((~np.isfinite(depth)).sum())
    ax.text(4, 232, f"{n_nan} pixel không đo được (NaN)", color="white", fontsize=8,
            bbox=dict(fc="#00000088", ec="none", pad=2))

    ax3 = fig.add_subplot(1, 2, 2, projection="3d")
    P = sc["points_base"][::3]
    P = P[(P[:, 0] < 3.95)]
    fl = P[:, 2] < 0.02
    ax3.scatter(P[fl, 0], P[fl, 1], P[fl, 2], color=th["faint"], s=0.2, linewidths=0, depthshade=False)
    ax3.scatter(P[~fl, 0], P[~fl, 1], P[~fl, 2], c=P[~fl, 2], cmap="viridis", vmin=0.0, vmax=1.0,
                s=0.35, linewidths=0, depthshade=False)
    rob = np.array([[-0.32, -0.22, 0], [0.18, -0.22, 0], [0.18, 0.22, 0], [-0.32, 0.22, 0],
                    [-0.32, -0.22, 0]])
    ax3.plot(rob[:, 0], rob[:, 1], rob[:, 2], color=th["ink"], lw=1.2)
    cam = sc["camera_pos_base"]
    ax3.scatter([cam[0]], [cam[1]], [cam[2]], color=th["init"], s=28, depthshade=False)
    ax3.text(cam[0] - 0.1, cam[1] + 0.35, cam[2] + 0.12, "camera", color=th["init"], fontsize=8)
    ax3.set_xlim(-0.4, 3.95)
    ax3.set_ylim(-1.7, 1.7)
    ax3.set_zlim(0, 1.6)
    ax3.set_box_aspect((4.35, 3.4, 1.6))
    ax3.view_init(elev=26, azim=-158)
    ax3.set_xlabel("x (m)")
    ax3.set_ylabel("y (m)")
    ax3.set_zlabel("z (m)")
    ax3.set_title("B. Đổi sang base_link (màu = độ cao, xám = sàn)", pad=2)
    style3d(ax3, th)
    META["f01"] = dict(n_points=int(len(sc["points_base"])), n_nan=n_nan, size_wh=[424, 240])
    return save(fig, "f01-pointcloud", theme)


# ============================================================================ F02 cân bằng sàn
def f02(th, theme):
    sc = syn.render_scene([(PALLET, 2.0, 0.1, 0.0)], noise=s.DepthNoise(),
                          rng=np.random.default_rng(3))
    org = sc["organized_optical"].reshape(-1, 3)
    org = org[np.isfinite(org).all(axis=1)]
    T_wrong = s.camera_mount(0.0, 0.3, math.radians(12.0))      # robot TƯỞNG camera chúc 12°
    P = g.transform_points(T_wrong, org)                          # thực tế camera chúc 10°
    band = P[(np.abs(P[:, 2]) < 0.15) & (P[:, 0] > 0.25) & (P[:, 0] < 3.5)]
    fit = c.fit_plane_ransac(band, 0.015, iterations=60, rng=np.random.default_rng(0))
    T_lev = c.leveling_transform(fit.normal, fit.d, 0.0)
    Q = g.transform_points(T_lev, P)
    tilt = math.degrees(math.acos(min(1.0, fit.normal[2])))
    strip = lambda A: A[np.abs(A[:, 1]) < 0.35]  # noqa: E731
    fig, axs = plt.subplots(1, 2, figsize=(11.0, 3.6), sharey=True)
    for ax, A, title in ((axs[0], strip(P), "A. TF sai pitch 2° → sàn và pallet bị nghiêng"),
                         (axs[1], strip(Q), "B. Sau khi cân bằng sàn bằng RANSAC")):
        floor = np.abs(A[:, 2] - (0 if ax is axs[1] else 0)) < 1  # placeholder để giữ thứ tự vẽ
        A2 = A[floor]
        is_floor = A2[:, 2] < 0.03 if ax is axs[1] else (np.abs(A2 @ fit.normal + fit.d) < 0.015)
        ax.scatter(A2[~is_floor, 0], A2[~is_floor, 2], s=0.6, color=th["tpl"], linewidths=0,
                   label="điểm vật (pallet)")
        ax.scatter(A2[is_floor, 0], A2[is_floor, 2], s=0.6, color=th["scene"], linewidths=0,
                   label="điểm sàn (inlier RANSAC)")
        ax.axhline(0.0, color=th["muted"], lw=0.8, ls="--")
        ax.axhline(0.144, color=th["muted"], lw=0.6, ls=":")
        ax.text(0.32, 0.152, "mặt pallet thật 0.144 m", color=th["muted"], fontsize=7.5)
        ax.set_xlim(0.2, 3.3)
        ax.set_ylim(-0.12, 0.30)
        ax.set_title(title)
        ax.set_xlabel("x (m) — khoảng cách phía trước robot")
    axs[0].set_ylabel("z (m) — độ cao")
    xs = np.array([0.3, 3.2])
    z_line = -(fit.normal[0] * xs + fit.d) / fit.normal[2]
    axs[0].plot(xs, z_line, color=th["init"], lw=1.4, label=f"mặt sàn ước lượng (nghiêng {tilt:.1f}°)")
    z2 = -(fit.normal[0] * 2.0 + fit.d) / fit.normal[2]
    axs[0].annotate(f"lệch {100 * z2:+.1f} cm ở x = 2 m", xy=(2.0, z2), xytext=(2.25, 0.2),
                    color=th["init"], fontsize=8.5,
                    arrowprops=dict(arrowstyle="->", color=th["init"], lw=1))
    axs[0].legend(loc="upper left", markerscale=12)
    axs[1].legend(loc="upper left", markerscale=12)
    META["f02"] = dict(tilt_deg=tilt, dz_at_2m_cm=100 * z2)
    return save(fig, "f02-leveling", theme)


# ============================================================================ F03 ROI + voxel
def f03(th, theme):
    sc = demo_scene()
    P = sc["points_stride2"].astype(np.float64)
    p = es.EstimatorParams()
    lo = np.array([p.roi_min[0], p.roi_min[1], p.floor_z + p.roi_min[2]])
    hi = np.array([p.roi_max[0], p.roi_max[1], p.floor_z + p.roi_max[2]])
    inroi = c.crop_box_mask(P, lo, hi)
    V, counts = c.voxel_downsample(P[inroi], p.voxel_size)
    fig, axs = plt.subplots(1, 3, figsize=(11.5, 4.3))
    sub = P[::2]
    axs[0].scatter(sub[:, 1], sub[:, 0], s=0.15, color=th["muted"], linewidths=0)
    axs[0].set_title(f"A. Đám mây (stride 2)\n{len(P):,} điểm".replace(",", "."))
    floor = P[~inroi][::2]
    axs[1].scatter(floor[:, 1], floor[:, 0], s=0.15, color=th["faint"], linewidths=0)
    K = P[inroi]
    axs[1].scatter(K[:, 1], K[:, 0], s=0.25, c=K[:, 2], cmap="viridis", vmin=0, vmax=1.0, linewidths=0)
    axs[1].add_patch(Rectangle((hi[1], lo[0]), lo[1] - hi[1], hi[0] - lo[0], fill=False,
                               ec=th["init"], lw=1.3, ls="--"))
    axs[1].text(1.45, 0.32, "hộp ROI", color=th["init"], fontsize=8.5, ha="left")
    axs[1].set_title(f"B. Cắt ROI + bỏ sàn\ncòn {inroi.sum():,} điểm".replace(",", "."))
    axs[2].scatter(V[:, 1], V[:, 0], s=1.3, c=V[:, 2], cmap="viridis", vmin=0, vmax=1.0, linewidths=0)
    axs[2].set_title(f"C. Gộp voxel 2 cm\ncòn {len(V):,} điểm".replace(",", "."))
    for ax in axs:
        topview(ax, xlim=(-0.4, 4.0), ylim=(-1.9, 1.9))
        draw_robot(ax, th, label=False)
    for ax in axs[1:]:
        ax.set_ylabel("")
    META["f03"] = dict(n_raw=int(len(P)), n_roi=int(inroi.sum()), n_voxel=int(len(V)))
    return save(fig, "f03-roi-voxel", theme)


# ============================================================================ F04 template
def _normal_colors(N, th):
    col = np.empty(len(N), dtype=object)
    col[:] = th["muted"]
    col[N[:, 0] < -0.7] = th["tpl"]
    col[np.abs(N[:, 1]) > 0.7] = th["scene"]
    col[N[:, 2] > 0.7] = th["ok"]
    return list(col)


def f04(th, theme):
    t = pallet_template()
    P, N = t.points, t.normals
    fig = plt.figure(figsize=(11.0, 4.0))
    gsp = fig.add_gridspec(1, 2, width_ratios=[1.0, 1.15], wspace=0.05)
    ax3 = fig.add_subplot(gsp[0, 0], projection="3d")
    ax3.scatter(P[:, 0], P[:, 1], P[:, 2], c=_normal_colors(N, th), s=0.6, linewidths=0,
                depthshade=False)
    ax3.set_box_aspect((1.2, 0.8, 0.3))
    ax3.view_init(elev=24, azim=-150)
    ax3.set_xlabel("x (m) vào trong vật")
    ax3.set_ylabel("y (m)")
    ax3.set_zlabel("z")
    ax3.set_zticks([0, 0.144])
    ax3.set_title(f"A. Template pallet EUR: {len(P):,} điểm có pháp tuyến".replace(",", "."), pad=0)
    style3d(ax3, th)
    for lbl, key in (("mặt trước (n hướng −x)", "tpl"), ("mặt bên (n hướng ±y)", "scene"),
                     ("mặt trên (n hướng +z)", "ok")):
        ax3.scatter([], [], [], color=th[key], s=12, label=lbl)
    ax3.legend(loc="upper left", bbox_to_anchor=(0.0, 0.98))

    ax = fig.add_subplot(gsp[0, 1])
    F = P[(P[:, 0] < 0.02)]
    ax.scatter(F[:, 1], F[:, 2], s=2.0, color=th["tpl"], linewidths=0)
    ax.set_xlim(0.45, -0.45)
    ax.set_ylim(-0.05, 0.2)
    ax.set_aspect("equal")
    ax.set_xlabel("y (m)")
    ax.set_ylabel("z (m)")
    ax.set_title("B. Nhìn thẳng mặt trước: 2 hốc càng phải TRỐNG")
    for yc in (0.18625, -0.18625):
        ax.add_patch(Rectangle((yc + 0.11375, 0.022), -0.2275, 0.078, fill=True, fc=th["scene"],
                               alpha=0.18, ec=th["scene"], lw=1))
        ax.text(yc, 0.061, "hốc càng\n227.5 × 78 mm", ha="center", va="center", fontsize=7.5,
                color=th["ink"])
    ax.annotate("", xy=(0.4, 0.172), xytext=(-0.4, 0.172),
                arrowprops=dict(arrowstyle="<->", color=th["muted"], lw=0.9))
    ax.text(0, 0.178, "rộng 800 mm (mặt tiếp cận)", ha="center", fontsize=7.5, color=th["muted"])
    ax.text(-0.012, -0.012, "gốc frame vật (0,0,0)", ha="left", va="top", fontsize=7.5, color=th["init"])
    ax.plot([0], [0], marker="o", color=th["init"], ms=4)
    META["f04"] = dict(n_points=int(len(P)))
    return save(fig, "f04-template", theme)


def f04b(th, theme):
    if "rack" not in _cache:
        _cache["rack"] = tp.template_from_boxes(s.rack_boxes(), "rack")
        _cache["vdock"] = tp.template_from_boxes(s.v_dock_boxes(), "v_dock")
    items = [(pallet_template(), "Pallet EUR 1200×800×144", (1.2, 0.8, 0.3)),
             (_cache["rack"], "Rack 1 khoang (cột + dầm)", (0.9, 1.2, 1.6)),
             (_cache["vdock"], "Dock chữ V (trạm sạc)", (0.3, 0.6, 0.35))]
    fig = plt.figure(figsize=(11.0, 3.6))
    for i, (t, name, box) in enumerate(items):
        ax = fig.add_subplot(1, 3, i + 1, projection="3d")
        P = t.points
        ax.scatter(P[:, 0], P[:, 1], P[:, 2], c=_normal_colors(t.normals, th), s=0.5, linewidths=0,
                   depthshade=False)
        ax.set_box_aspect(box)
        ax.view_init(elev=22, azim=-145)
        ax.set_title(f"{name}\n{len(P):,} điểm".replace(",", "."), pad=0, fontsize=9.5)
        ax.tick_params(labelsize=6.5)
        style3d(ax, th)
    return save(fig, "f04b-templates", theme)


# ============================================================================ F05 tìm kiếm 2.5D
def _search_internals(searcher, S, lo, hi):
    """Bản sao từng dòng của CorrelativeSearch.search() nhưng trả cả lưới O và điểm số."""
    from scipy.fft import next_fast_len

    p = searcher.params
    res = p.grid_res
    S = S[(S[:, 0] >= lo[0]) & (S[:, 0] < hi[0]) & (S[:, 1] >= lo[1]) & (S[:, 1] < hi[1])
          & (S[:, 2] >= searcher.z_lo) & (S[:, 2] <= searcher.z_hi)]
    H = int(math.ceil((hi[0] - lo[0]) / res))
    W = int(math.ceil((hi[1] - lo[1]) / res))
    band = np.clip(np.searchsorted(searcher.edges, S[:, 2], side="right") - 1, 0, p.n_bands - 1)
    ij = np.floor((S[:, :2] - lo) / res).astype(int)
    ij[:, 0] = np.clip(ij[:, 0], 0, H - 1)
    ij[:, 1] = np.clip(ij[:, 1], 0, W - 1)
    O = np.zeros((p.n_bands, H, W), dtype=np.float32)
    O[band, ij[:, 0], ij[:, 1]] = 1.0
    peak = 1.0 / (2.0 * math.pi * p.blur_sigma_cells ** 2)
    L = np.stack([np.minimum(searcher._gauss(O[b], p.blur_sigma_cells) / peak, 1.0)
                  for b in range(p.n_bands)]).astype(np.float32)
    k = 2 * searcher.R + 1
    shape = (next_fast_len(H + k - 1, real=True), next_fast_len(W + k - 1, real=True))
    FP, FF = searcher._kernel_ffts(shape)
    FL = np.fft.rfft2(L, s=shape)
    FO = np.fft.rfft2(O, s=shape)
    pos = np.fft.irfft2((FP * FL[None]).sum(axis=1), s=shape)
    neg = np.fft.irfft2((FF * FO[None]).sum(axis=1), s=shape)
    r = searcher.R
    pos = pos[:, r:r + H, r:r + W] / searcher.P_sum[:, None, None]
    neg = neg[:, r:r + H, r:r + W] / searcher.F_sum[:, None, None]
    return O, pos - p.free_weight * np.clip(neg, 0.0, None)


def f05(th, theme):
    est = estimator()
    est.reset()
    sc = demo_scene()
    S = est.preprocess(sc["points_stride2"])
    srch = est.searcher
    lo = np.array(est.params.roi_min[:2], float)
    hi = np.array(est.params.roi_max[:2], float)
    O, score = _search_internals(srch, S, lo, hi)
    hyps = srch.search(S, lo, hi)
    ext = (lo[1], hi[1], lo[0], hi[0])
    fig = plt.figure(figsize=(11.6, 7.0))
    gs = fig.add_gridspec(2, 1, height_ratios=[1, 1.2], hspace=0.32)
    top = gs[0].subgridspec(1, 4, wspace=0.32)
    bot = gs[1].subgridspec(1, 2, wspace=0.30, width_ratios=[1, 1.1])
    edges = srch.edges
    for b in range(3):
        ax = fig.add_subplot(top[0, b])
        ax.grid(False)
        ax.imshow(O[b], origin="lower", extent=ext, cmap="Greys" if theme == "light" else "gray",
                  vmin=0, vmax=1, interpolation="nearest")
        ax.set_xlim(1.6, -1.6)
        ax.set_ylim(0.2, 3.6)
        ax.set_title(f"A{b + 1}. Cảnh, dải {b + 1}: z {100 * edges[b]:.0f}–{100 * edges[b + 1]:.0f} cm",
                     fontsize=9)
        ax.tick_params(labelsize=7)
    t0 = int(np.argmin(np.abs(srch.yaws)))
    R = srch.R
    res = srch.params.grid_res
    kx = (-R - 0.5) * res, (R + 0.5) * res
    ax = fig.add_subplot(top[0, 3])
    ax.grid(False)
    rgb = np.zeros(srch.P.shape[2:] + (4,))
    tplc = matplotlib.colors.to_rgba(th["tpl"])
    frc = matplotlib.colors.to_rgba(th["scene"])
    rgb[srch.F[t0, 0] > 0] = (frc[0], frc[1], frc[2], 0.5)
    rgb[srch.P[t0, 0] > 0] = tplc
    ax.imshow(rgb, origin="lower", extent=(kx[0], kx[1], kx[0], kx[1]), interpolation="nearest")
    ax.set_xlim(0.55, -0.55)
    ax.set_ylim(-0.15, 1.35)
    ax.set_title("A4. Mẫu pallet (dải 1, yaw 0°)\ncam = bề mặt P · xanh = phải trống F", fontsize=8.5)
    ax.tick_params(labelsize=7)

    ax = fig.add_subplot(bot[0, 0])
    ax.grid(False)
    best = np.clip(score.max(axis=0), 0, None)
    im = ax.imshow(best, origin="lower", extent=ext, cmap="magma", interpolation="nearest")
    ax.set_xlim(1.6, -1.6)
    ax.set_ylim(0.2, 3.6)
    cb = fig.colorbar(im, ax=ax, fraction=0.045, pad=0.03)
    cb.outline.set_edgecolor(th["muted"])
    cb.ax.tick_params(labelsize=7)
    for i, h in enumerate(hyps):
        ax.plot(h.y, h.x, "o", ms=12, mfc="none", mec="#7FE0FF" if i == 0 else "#FFFFFF", mew=1.5)
        ax.text(h.y - 0.12, h.x + 0.1, f"#{i + 1}", color="#FFFFFF", fontsize=8, weight="bold")
    tx, ty, tyaw = sc["truth"]
    ax.plot(ty, tx, "+", ms=12, mew=2, color="#7FE0FF")
    ax.set_title("B. Điểm số cao nhất tại mỗi ô (x, y)")
    ax.set_xlabel("y (m)")
    ax.set_ylabel("x (m)")

    ax = fig.add_subplot(bot[0, 1])
    h0 = hyps[0]
    i0 = int((h0.x - lo[0]) / res)
    j0 = int((h0.y - lo[1]) / res)
    ys = score[:, i0, j0]
    ax.plot(np.degrees(srch.yaws), ys, color=th["tpl"], lw=1.6, marker="o", ms=3)
    ax.axvline(math.degrees(tyaw), color=th["muted"], ls="--", lw=1)
    k = int(np.argmax(ys))
    ax.annotate(f"đỉnh: yaw {math.degrees(srch.yaws[k]):.0f}°\n(thật {math.degrees(tyaw):.0f}°)",
                xy=(math.degrees(srch.yaws[k]), ys[k]), xytext=(18, ys[k] * 0.97), fontsize=8,
                color=th["ink"], arrowprops=dict(arrowstyle="->", color=th["muted"], lw=0.9))
    ax.set_title("C. Điểm số theo yaw tại giả thuyết #1")
    ax.set_xlabel("yaw thử (°): bước 3°, quét ±45°")
    ax.set_ylabel("điểm số")
    META["f05"] = dict(hyps=[dict(x=h.x, y=h.y, yaw_deg=math.degrees(h.yaw), score=h.score)
                             for h in hyps], truth=[tx, ty, math.degrees(tyaw)],
                       bands_cm=[100 * float(e) for e in edges], n_yaw=int(len(srch.yaws)),
                       grid=list(O.shape[1:]))
    return save(fig, "f05-search", theme)


# ============================================================================ F06 ICP
def _icp_trace(S, t, T0, nn, cam):
    base = reg.IcpParams()
    T = T0.copy()
    hist, stage = [T.copy()], [0]
    n = len(base.max_corr_dists)
    for si, dmax in enumerate(base.max_corr_dists):
        for _ in range(base.max_iters):
            prm = reg.IcpParams(max_corr_dists=(dmax,), max_iters=1, exact_final_stage=(si == n - 1))
            if si != n - 1:
                prm.exact_final_stage = False
            r = reg.icp_point_to_plane(S, t, T, prm, nn=nn, camera_pos_base=cam)
            Tn = r.T_base_obj
            dt = np.linalg.norm(Tn[:3, 3] - T[:3, 3])
            dr = abs(g.wrap_angle(g.yaw_of(Tn) - g.yaw_of(T)))
            T = Tn
            hist.append(T.copy())
            stage.append(si)
            if dt < base.eps_t and dr < base.eps_r:
                break
    return hist, stage, r


def f06(th, theme):
    t = pallet_template()
    truth = (1.5, 0.2, math.radians(10.0))
    sc = syn.render_scene([(PALLET, *truth)], noise=s.DepthNoise(), rng=np.random.default_rng(0))
    est = estimator()
    est.reset()
    S = est.preprocess(sc["points_base"])
    T0 = g.T_from_xyz_yaw(truth[0] - 0.05, truth[1] + 0.05, 0.0, truth[2] - math.radians(5))
    if "nn" not in _cache:
        _cache["nn"] = reg.NearestTemplateGrid(t.points, 0.01, margin=0.2)
    hist, stage, last = _icp_trace(S, t, T0, _cache["nn"], sc["camera_pos_base"])
    errs = np.array([[100 * math.hypot(*(np.array(g.pose2d(T)[:2]) - truth[:2])),
                      abs(math.degrees(g.wrap_angle(g.yaw_of(T) - truth[2])))] for T in hist])
    fig, axs = plt.subplots(1, 2, figsize=(11.0, 4.6), gridspec_kw=dict(width_ratios=[1.0, 1.1]))
    ax = axs[0]
    near = S[(S[:, 0] > 1.0) & (S[:, 0] < 3.0) & (np.abs(S[:, 1]) < 1.0)]
    Q = g.transform_points(hist[-1], t.points)
    Q = Q[Q[:, 2] > 0.03]
    ax.scatter(Q[::2, 1], Q[::2, 0], s=0.35, color=th["tpl"], linewidths=0, alpha=0.8)
    ax.scatter(near[:, 1], near[:, 0], s=2.6, color=th["scene"], linewidths=0)
    ax.add_patch(Polygon(footprint(T0), closed=True, fill=False, ec=th["init"], lw=1.5, ls="--"))
    ax.add_patch(Polygon(footprint(hist[-1]), closed=True, fill=False, ec=th["tpl"], lw=1.5))
    topview(ax, xlim=(1.3, 2.85), ylim=(-1.05, 0.95))
    ax.set_title("A. Nhìn từ trên xuống")
    from matplotlib.lines import Line2D
    hl = [Line2D([], [], marker="o", ls="", color=th["scene"], label="điểm cảnh (sau voxel)"),
          Line2D([], [], color=th["init"], ls="--", label="pose khởi tạo (lệch 7 cm, 5°)"),
          Line2D([], [], color=th["tpl"], label="template sau ICP")]
    ax.legend(handles=hl, loc="upper right", fontsize=7.5)
    ax = axs[1]
    it = np.arange(len(errs))
    st = np.array(stage)
    for si, d in enumerate(reg.IcpParams().max_corr_dists):
        idx = np.nonzero(st == si)[0]
        if len(idx):
            if si % 2 == 0:
                ax.axvspan(idx[0] - 0.5, idx[-1] + 0.5, color=th["grid"], alpha=0.8, zorder=0, lw=0)
            ax.text((idx[0] + idx[-1]) / 2, 0.98 - 0.11 * (si % 2), f"tầng {si + 1}\n≤{100 * d:.0f} cm",
                    transform=ax.get_xaxis_transform(), ha="center", va="top", fontsize=7.5,
                    color=th["muted"])
    ax.plot(it, errs[:, 0], color=th["scene"], lw=2, marker="o", ms=3.5, label="sai số vị trí (cm)")
    ax.plot(it, errs[:, 1], color=th["tpl"], lw=2, marker="s", ms=3.5, label="sai số yaw (°)")
    ax.set_xlabel("vòng lặp ICP")
    ax.set_ylabel("sai số")
    ax.set_ylim(-0.3, 9.6)
    ax.set_yticks([0, 2, 4, 6, 8])
    ax.annotate(f"cuối: {10 * errs[-1, 0]:.1f} mm, {errs[-1, 1]:.2f}°", xy=(it[-1], errs[-1, 0]),
                xytext=(it[-1] - 4.5, 2.2), fontsize=8.5, color=th["ink"],
                arrowprops=dict(arrowstyle="->", color=th["muted"], lw=0.9))
    ax.set_title("B. Sai số qua 3 tầng thô → tinh")
    ax.legend(loc="center right")
    META["f06"] = dict(n_iter=int(len(hist) - 1), err_init=errs[0].tolist(), err_final=errs[-1].tolist(),
                       coverage=last.coverage, inlier=last.inlier_ratio, rmse_mm=1000 * last.rmse,
                       stages=[int((st == k).sum()) for k in range(3)])
    return save(fig, "f06-icp", theme)


# ============================================================================ F07 cổng chất lượng
def _prep(points):
    P = points[points[:, 2] > 0.03]
    return c.voxel_downsample(P, 0.02)[0]


def _profile(case):
    key = "prof_" + case
    if key in _cache:
        return _cache[key]
    if case == "pallet":
        t = pallet_template()
        sc = syn.render_scene([(PALLET, 1.6, 0.0, 0.0)], noise=s.DepthNoise(), rng=np.random.default_rng(1))
        T0 = g.T_from_xyz_yaw(1.6, 0.0, 0.0, 0.0)
    elif case == "plate":
        t = tp.template_from_boxes([s.Box((0.01, 0.0, 0.3), (0.02, 1.0, 0.6))], "plate")
        sc = syn.render_scene([], extra_boxes=[s.Box((1.6, 0.0, 1.0), (0.02, 8.0, 2.0))], noise=None)
        T0 = g.T_from_xyz_yaw(1.59, 0.1, 0.0, 0.0)
    else:
        v = s.v_dock_boxes()
        t = tp.template_from_boxes(v, "v_dock")
        sc = syn.render_scene([(v, 1.2, 0.0, 0.0)], noise=s.DepthNoise(), rng=np.random.default_rng(3))
        T0 = g.T_from_xyz_yaw(1.2, 0.0, 0.0, 0.0)
    S = _prep(sc["points_base"])
    r = reg.icp_point_to_plane(S, t, T0, camera_pos_base=sc["camera_pos_base"])
    p = reg.IcpParams()
    T_ob = g.inv_T(r.T_base_obj)
    lo, hi = t.bounds
    Q = g.transform_points(T_ob, S)
    q = Q[c.crop_box_mask(Q, lo - p.metric_margin, hi + p.metric_margin)]
    cc = p.coverage_radius
    f0 = reg._profile_cost(q, t, t.kdtree(), None, cc)
    dys = np.linspace(-0.06, 0.06, 25)
    prof = np.array([(reg._profile_cost(q - [0.0, dy, 0.0], t, t.kdtree(), None, cc) - f0) / cc ** 2
                     for dy in dys])
    dts = np.radians(np.linspace(-4.0, 4.0, 25))
    profr = np.array([(reg._profile_cost(q @ g.rot_z(a).T, t, t.kdtree(), None, cc) - f0) / cc ** 2
                      for a in dts])
    _cache[key] = (dys, prof, r, dts, profr)
    return _cache[key]


def f07(th, theme):
    fig = plt.figure(figsize=(11.6, 4.5))
    gsx = fig.add_gridspec(1, 3, width_ratios=[1, 1, 1.25], wspace=0.32)
    names = {"pallet": "pallet EUR", "vdock": "dock chữ V", "plate": "tấm phẳng sát tường"}
    cols = {"pallet": th["tpl"], "vdock": th["ok"], "plate": th["bad"]}
    obs = {}
    thr = reg.IcpParams().min_observability
    axA = fig.add_subplot(gsx[0, 0])
    axB = fig.add_subplot(gsx[0, 1], sharey=axA)
    for case in ("pallet", "vdock", "plate"):
        dys, prof, r, dts, profr = _profile(case)
        obs[case] = r.observability
        lab = f"{names[case]}: obs {r.observability:.3f}" + (" ✗" if r.degenerate else " ✓")
        axA.plot(100 * dys, prof, color=cols[case], lw=2, label=lab)
        axB.plot(np.degrees(dts), profr, color=cols[case], lw=2)
    for ax, xv in ((axA, 3.0), (axB, 1.5)):
        ax.axhline(thr, color=th["muted"], ls="--", lw=1)
        ax.axvline(-xv, color=th["muted"], lw=0.8, ls=":")
        ax.axvline(xv, color=th["muted"], lw=0.8, ls=":")
    axA.set_ylim(-0.02, 0.2)
    axA.set_xlabel("dịch ngang thử Δy (cm)")
    axB.set_xlabel("xoay thử Δyaw (°)")
    axA.set_ylabel("chi phí tăng thêm (chuẩn hóa)")
    axA.set_title("A. Dịch ngang template")
    axB.set_title("B. Xoay template")
    axA.text(-5.8, thr + 0.004, f"ngưỡng {thr}", color=th["muted"], fontsize=7.5)
    axA.legend(loc="upper center", fontsize=7.5, bbox_to_anchor=(1.1, -0.18), ncol=3)

    ax = fig.add_subplot(gsx[0, 2])
    if "amb" not in _cache:
        est = es.DockPoseEstimator(pallet_template(), es.EstimatorParams())
        sc = syn.render_scene([(PALLET, 2.0, 0.65, 0.0), (PALLET, 2.0, -0.65, 0.0)],
                              noise=s.DepthNoise(), rng=np.random.default_rng(4))
        det = est.process(sc["points_base"], sc["camera_pos_base"])
        S = est.preprocess(sc["points_base"])
        p = est.params
        hyps = est.searcher.search(S, p.roi_min[:2], p.roi_max[:2])
        cam = sc["camera_pos_base"]
        res = sorted((est._icp(S, g.T_from_xyz_yaw(h.x, h.y, 0.0, h.yaw), cam) for h in hyps),
                     key=lambda r: r.score, reverse=True)
        left = es.DockPoseEstimator(pallet_template(), es.EstimatorParams(roi_min=(0.25, 0.0, 0.03)))
        det2 = left.process(sc["points_base"], sc["camera_pos_base"])
        _cache["amb"] = (sc, S, det, res, det2)
    sc, S, det, res, det2 = _cache["amb"]
    ax.scatter(S[:, 1], S[:, 0], s=1.6, color=th["scene"], linewidths=0)
    shown = [res[0]]
    for r in res[1:]:
        if np.hypot(*(r.T_base_obj[:2, 3] - res[0].T_base_obj[:2, 3])) > 0.3:
            shown.append(r)
            break
    for i, r in enumerate(shown):
        fp = footprint(r.T_base_obj)
        ax.add_patch(Polygon(fp, closed=True, fill=False, ec=th["tpl"] if i == 0 else th["bad"],
                             lw=1.6, ls="-" if i == 0 else "--"))
        x, y, _ = g.pose2d(r.T_base_obj)
        ax.text(y, x + 1.3, f"#{i + 1}: score {r.score:.2f}", ha="center", fontsize=8.5,
                color=th["tpl"] if i == 0 else th["bad"], weight="bold")
    ratio = shown[1].score / shown[0].score if len(shown) > 1 else float("nan")
    ax.add_patch(Rectangle((1.5, 0.0), -1.5, 3.5, fill=True, fc=th["ok"], alpha=0.07, ec=th["ok"],
                           lw=1.2, ls=":"))
    ax.text(0.75, 0.32, "ROI thu hẹp y > 0\n→ status OK", ha="center", color=th["ok"], fontsize=8)
    topview(ax, xlim=(-0.4, 3.6), ylim=(-1.6, 1.6))
    draw_robot(ax, th, label=False)
    ax.set_title(f"C. Hai pallet giống nhau: #2/#1 = {ratio:.2f}\n≥ 0.9 → {det.status.value}")
    META["f07"] = dict(obs=obs, thr=thr, amb_status=det.status.value, amb_ratio=ratio,
                       amb_msg=det.message, left_status=det2.status.value,
                       left_y=float(det2.T_base_obj[1, 3]) if det2.T_base_obj is not None else None)
    return save(fig, "f07-quality", theme)


# ============================================================================ F10 thời gian
def f10(th, theme):
    if "timing" not in _cache:
        est = es.DockPoseEstimator(pallet_template(), es.EstimatorParams())
        sc = demo_scene()
        cam = sc["camera_pos_base"]
        ts, tt = [], []
        for _ in range(8):
            est.reset()
            d = est.process(sc["points_stride2"], cam)
            ts.append(d.timings)
        prior = d.T_base_obj
        for _ in range(25):
            d = est.process(sc["points_stride2"], cam, prior_T_base_obj=prior)
            tt.append(d.timings)
        med = lambda L, k: float(np.median([x.get(k, 0.0) for x in L]))  # noqa: E731
        _cache["timing"] = {"search": {k: med(ts, k) for k in ("preprocess", "search", "icp", "total")},
                            "track": {k: med(tt, k) for k in ("preprocess", "icp", "total")}}
    T = _cache["timing"]
    fig, ax = plt.subplots(figsize=(10.0, 2.9))
    rows = [("Bám (track)", T["track"], ("preprocess", "icp")),
            ("Tìm kiếm (mất dấu)", T["search"], ("preprocess", "search", "icp"))]
    colors = {"preprocess": th["scene"], "search": th["init"], "icp": th["tpl"]}
    names = {"preprocess": "tiền xử lý", "search": "tìm kiếm 2.5D", "icp": "ICP (top-5)"}
    for yi, (lbl, d, keys) in enumerate(rows):
        left = 0.0
        for k in keys:
            ax.barh(yi, d[k], left=left, color=colors[k], height=0.5,
                    label=names[k] if yi == 1 or k != "icp" else None)
            if d[k] > 6:
                ax.text(left + d[k] / 2, yi, f"{d[k]:.0f}", ha="center", va="center", fontsize=8,
                        color="#FFFFFF" if theme == "light" else "#161D24", weight="bold")
            left += d[k]
        ax.text(left + 2, yi, f"tổng {d['total']:.0f} ms", va="center", fontsize=8.5, color=th["ink"])
    ax.set_yticks([0, 1], [r[0] for r in rows])
    ax.set_xlabel("thời gian xử lý 1 khung (ms, trung vị, máy dev i7-14700F)")
    ax.axvline(40, color=th["muted"], ls="--", lw=1)
    ax.text(41, -0.42, "ngân sách 40 ms/khung ở chế độ bám (CPU cỡ RPi 5)", fontsize=7.5,
            color=th["muted"], va="center")
    ax.set_ylim(-0.65, 1.45)
    handles, labels = ax.get_legend_handles_labels()
    uniq = dict(zip(labels, handles))
    ax.legend(uniq.values(), uniq.keys(), loc="upper right", ncol=3, bbox_to_anchor=(1.0, 1.12))
    ax.grid(axis="y", visible=False)
    META["f10"] = T
    return save(fig, "f10-timing", theme)


# ============================================================================ F08 Kalman + vùng mù
from camera3d_docking.core import control as ctl  # noqa: E402
from camera3d_docking.core import kalman as kf  # noqa: E402

LAW = ctl.ControlLawParams(k_phi=3.0, k_delta=2.0, beta=0.4, lam=2.0, v_max=0.2, v_min=0.02,
                           w_max=0.6, slowdown_radius=0.25)          # = config/controller.yaml
FSMP = ctl.FsmParams(goal_tol_x=0.01, blind_zone=0.6, blind_speed=0.08, detection_timeout=5.0,
                     lost_timeout=1.0, approach_timeout=60.0, max_retries=2, retreat_distance=0.5,
                     retreat_speed=0.1, acc_v=0.5, acc_w=1.5)
DOCK_OFFSET = -0.35                                                   # dock_offset_x


def simulate_dock(x0, y0, yaw0, dt=0.05, t_max=60.0):
    """Robot một bánh (unicycle) chạy DockingFSM thật; đích = gốc (0,0,0) hướng +x."""
    fsm = ctl.DockingFSM(LAW, FSMP)
    fsm.start(0.0)
    x, y, th = x0, y0, yaw0
    rows = []
    t = 0.0
    while t < t_max:
        dx, dy = -x, -y
        tx = math.cos(th) * dx + math.sin(th) * dy
        ty = -math.sin(th) * dx + math.cos(th) * dy
        tyaw = g.wrap_angle(-th)
        v, w = fsm.step(t, (tx, ty, float(tyaw)), 0.0)
        rows.append((t, x, y, th, v, w, fsm.state.value))
        if fsm.state in (ctl.DockState.DOCKED, ctl.DockState.FAILED):
            break
        x += v * math.cos(th) * dt
        y += v * math.sin(th) * dt
        th += w * dt
        t += dt
    return rows


def f08(th, theme):
    if "kf" not in _cache:
        rows = simulate_dock(-2.0, 0.06, math.radians(4.0))
        face_x = -DOCK_OFFSET                                   # mặt pallet tại x = +0.35
        truth = np.array([face_x, 0.0, 0.0])
        rng = np.random.default_rng(11)
        filt = kf.StaticPoseKF()
        out = []
        next_meas = 0.0
        for (t, x, y, yaw, v, w, st) in rows:
            if t + 1e-9 < next_meas:
                continue
            next_meas = t + 0.1                                 # estimator 10 Hz
            d = face_x - x
            occluded = 3.0 <= t < 4.2                            # bị che 1.2 s (người đi ngang)
            blind = d < 0.6                                      # blind_zone_distance
            z = None
            kind = ""
            if not occluded and not blind:
                sxy = 0.002 + 0.002 * d * d
                syaw = math.radians(0.15 + 0.15 * d * d)
                z = truth + rng.normal(0, 1, 3) * np.array([sxy, sxy, syaw])
                R = np.diag([max(sxy, 0.005) ** 2, max(sxy, 0.005) ** 2, max(syaw, math.radians(0.3)) ** 2])
                if abs(t - 2.0) < 0.06:
                    z = truth + np.array([0.02, 0.32, math.radians(1.0)])   # nhận nhầm 1 khung
                    kind = "outlier"
                acc = filt.update(z, R, t)
                kind = kind or ("ok" if acc else "rejected")
                if kind == "outlier" and not acc:
                    kind = "rejected"
            else:
                filt.predict(t)
                kind = "blind" if blind else "occluded"
            sd = math.sqrt(filt.P[1, 1]) if filt.P is not None else float("nan")
            out.append((t, d, None if z is None else z[1], filt.x[1] if filt.x is not None else np.nan, sd,
                        kind))
        _cache["kf"] = (rows, out)
    rows, out = _cache["kf"]
    t = np.array([o[0] for o in out])
    d = np.array([o[1] for o in out])
    est = 100 * np.array([o[3] for o in out])
    sd = 100 * np.array([o[4] for o in out])
    kinds = [o[5] for o in out]
    fig, axs = plt.subplots(2, 1, figsize=(11.0, 5.6), sharex=True,
                            gridspec_kw=dict(height_ratios=[1, 1.5], hspace=0.28))
    tb = t[np.array([k == "blind" for k in kinds])]
    t_blind = tb[0] if len(tb) else t[-1]
    for ax in axs:
        ax.axvspan(3.0, 4.2, color=th["init"], alpha=0.10, lw=0)
        ax.axvspan(t_blind, t[-1] + 0.1, color=th["tpl"], alpha=0.13, lw=0)
    ax = axs[0]
    ax.plot(t, d, color=th["ink"], lw=1.8)
    ax.axhline(0.6, color=th["tpl"], ls="--", lw=1)
    ax.text(0.1, 0.66, "blind_zone_distance = 0.6 m", color=th["tpl"], fontsize=8)
    ax.set_ylabel("robot → mặt vật (m)")
    ax.set_ylim(0, 2.6)
    ax.text(3.6, 2.35, "bị che 1.2 s", ha="center", color=th["init"], fontsize=8.5)
    ax.text((t_blind + t[-1]) / 2, 2.35, "vùng mù: không đo nữa", ha="center", color=th["tpl"], fontsize=8.5)
    ax.set_title("A. Robot tiến vào (mô phỏng bằng DockingFSM thật)")
    ax = axs[1]
    ok = np.array([k == "ok" for k in kinds])
    rej = np.array([k == "rejected" for k in kinds])
    zs = np.array([np.nan if o[2] is None else 100 * o[2] for o in out])
    ax.fill_between(t, est - 2 * sd, est + 2 * sd, color=th["scene"], alpha=0.18, lw=0,
                    label="±2σ của Kalman")
    ax.plot(t[ok], zs[ok], "o", ms=3, color=th["muted"], label="phép đo được nhận")
    if rej.any():
        ax.plot(t[rej], np.clip(zs[rej], -4, 4.6), "x", ms=9, mew=2, color=th["bad"],
                label="phép đo bị cổng Mahalanobis loại")
        ax.annotate(f"nhận nhầm: lệch {zs[rej][0]:.0f} cm\n→ bị loại, pose không nhảy",
                    xy=(t[rej][0], 4.5), xytext=(t[rej][0] + 0.5, 3.0), fontsize=8, color=th["bad"],
                    arrowprops=dict(arrowstyle="->", color=th["bad"], lw=0.9))
    ax.plot(t, est, color=th["scene"], lw=2, label="pose Kalman (được phát 10 Hz)")
    ax.axhline(0.0, color=th["ink"], ls=":", lw=1, label="vị trí thật")
    ax.set_ylim(-4.2, 4.9)
    ax.set_ylabel("y của vật trong odom (cm)")
    ax.set_xlabel("thời gian (s)")
    ax.set_title("B. Ước lượng ngang của vật: vẫn có pose khi bị che và khi ở vùng mù")
    ax.legend(loc="lower left", ncol=3, fontsize=7.8)
    META["f08"] = dict(t_blind=float(t_blind), t_end=float(t[-1]), final_sd_cm=float(sd[-1]),
                       final_err_cm=float(est[-1]), n_rej=int(rej.sum()))
    return save(fig, "f08-kalman", theme)


# ============================================================================ F09 điều khiển
STARTS = [(-1.5, 0.30, 0.0), (-1.5, -0.30, 0.0), (-1.4, 0.0, 20.0), (-1.3, -0.30, -20.0),
          (-1.5, 0.25, 15.0)]


def f09(th, theme):
    if "ctl" not in _cache:
        _cache["ctl"] = [simulate_dock(x, y, math.radians(a)) for (x, y, a) in STARTS]
    runs = _cache["ctl"]
    fig, axs = plt.subplots(1, 2, figsize=(11.2, 4.9), gridspec_kw=dict(width_ratios=[0.85, 1.3], wspace=0.22))
    ax = axs[0]
    face = -DOCK_OFFSET
    ax.add_patch(Rectangle((0.4, face), -0.8, 1.2, fill=True, fc=th["faint"], ec=th["muted"], lw=1))
    ax.text(0.0, face + 0.33, "pallet", ha="center", va="center", color=th["ink"], fontsize=9)
    ax.axhline(-0.6, color=th["tpl"], ls="--", lw=1)
    ax.text(-0.42, -0.57, "còn 0.6 m → BLIND", color=th["tpl"], fontsize=8, ha="right", va="bottom")
    finals = []
    for k, rows in enumerate(runs):
        R = np.array([(r[1], r[2]) for r in rows])
        st = [r[6] for r in rows]
        blind = np.array([s_ == "BLIND" for s_ in st])
        ax.plot(R[:, 1], R[:, 0], color=th["scene"], lw=1.8, label="APPROACH" if k == 0 else None)
        if blind.any():
            i0 = int(np.argmax(blind))
            ax.plot(R[i0:, 1], R[i0:, 0], color=th["tpl"], lw=2.4, label="BLIND" if k == 0 else None)
        x0, y0, a0 = rows[0][1], rows[0][2], rows[0][3]
        draw_robot(ax, th, x0, y0, a0, L=(-0.16, 0.10), W=0.22, label=False)
        last = rows[-1]
        finals.append((100 * abs(last[2]), abs(math.degrees(last[3])), last[0], last[6]))
    ax.plot([0], [0], marker="+", ms=14, mew=2, color=th["ok"])
    ax.text(-0.05, 0.05, "đích", color=th["ok"], fontsize=8.5)
    topview(ax, xlim=(-1.75, 1.05), ylim=(-0.8, 0.8))
    ax.set_xlabel("y (m)")
    ax.set_ylabel("x (m) theo trục dock ↑")
    ax.set_title("A. 5 điểm xuất phát → 1 đường vào thẳng")
    ax.legend(loc="center left", fontsize=8)
    ax = axs[1]
    rows = runs[3]
    T = np.array([r[0] for r in rows])
    V = np.array([r[4] for r in rows])
    W = np.array([r[5] for r in rows])
    st = [r[6] for r in rows]
    for name, col in (("APPROACH", th["scene"]), ("BLIND", th["tpl"])):
        m = np.array([s_ == name for s_ in st])
        if m.any():
            ax.axvspan(T[m][0], T[m][-1], color=col, alpha=0.12, lw=0)
            ax.text((T[m][0] + T[m][-1]) / 2, 0.97, name, transform=ax.get_xaxis_transform(),
                    ha="center", va="top", fontsize=8, color=col)
    ax.plot(T, V, color=th["ink"], lw=2, label="v (m/s)")
    ax.plot(T, W, color=th["init"], lw=1.6, label="ω (rad/s)")
    ax.axhline(0.2, color=th["muted"], ls=":", lw=1)
    ax.text(T[-1], 0.205, "v_max 0.2", ha="right", va="bottom", fontsize=7.5, color=th["muted"])
    ax.axhline(0.08, color=th["tpl"], ls=":", lw=1)
    ax.text(T[-1], 0.085, "blind_speed 0.08", ha="right", va="bottom", fontsize=7.5, color=th["tpl"])
    ax.set_xlabel("thời gian (s)")
    ax.set_title(f"B. Lệnh v, ω (xuất phát −1.3 m, −0.3 m, −20°)")
    ax.text(T[-1], -0.02, f"DOCKED\nsau {T[-1]:.1f} s", ha="right", va="top", fontsize=8, color=th["ok"], weight="bold")
    ax.legend(loc="lower right")
    META["f09"] = dict(finals=[dict(start=list(s0), lat_cm=f[0], yaw_deg=f[1], t=f[2], state=f[3])
                               for s0, f in zip(STARTS, finals)])
    return save(fig, "f09-control", theme)


# ============================================================================ F11 lần chạy thật trong Gazebo
SIMDIR = os.path.join(HERE, "data")                 # data/run1.jsonl ghi bằng record_run.py


def _robot_patch(ax, th, x, y, yaw, L=(-0.30, 0.20), W=0.40, ec=None, fc=None, ls="-", lw=1.2, z=5):
    c, s_ = math.cos(yaw), math.sin(yaw)
    P = np.array([[L[0], -W / 2], [L[1], -W / 2], [L[1], W / 2], [L[0], W / 2]])
    P = P @ np.array([[c, s_], [-s_, c]]) + [x, y]
    ax.add_patch(Polygon(P[:, ::-1], closed=True, fc=fc or th["faint"], ec=ec or th["muted"], lw=lw,
                         ls=ls, zorder=z))
    tip = np.array([x, y]) + 0.32 * np.array([c, s_])
    ax.annotate("", xy=(tip[1], tip[0]), xytext=(y, x),
                arrowprops=dict(arrowstyle="-|>", color=ec or th["ink"], lw=1.2), zorder=z + 1)


def f11(th, theme):
    rows = [json.loads(line) for line in open(os.path.join(SIMDIR, "run1.jsonl"))]
    gt = [r for r in rows if r["k"] == "gt"]
    ctl = [(r["t"], r["state"].split(":")[0]) for r in rows if r["k"] == "ctl"]
    st = [(r["t"], r.get("status", "")) for r in rows if r["k"] == "st"]
    t0 = next(t for t, s_ in ctl if s_ == "WAIT_DETECTION")
    tb = next(t for t, s_ in ctl if s_ == "BLIND")
    td = next(t for t, s_ in ctl if s_ == "DOCKED")
    T = np.array([r["t"] for r in gt]) - t0
    X = np.array([r["x"] for r in gt])
    Y = np.array([r["y"] for r in gt])
    YAW = np.array([r["yaw"] for r in gt])
    target = (2.65, 0.0)
    fig = plt.figure(figsize=(11.6, 5.4))
    gsx = fig.add_gridspec(3, 2, width_ratios=[1.0, 1.25], height_ratios=[1, 1, 0.22], hspace=0.35,
                           wspace=0.22)
    ax = fig.add_subplot(gsx[:, 0])
    # vật trong world (khớp generate_world.py)
    ax.add_patch(Rectangle((0.4, 3.0), -0.8, 1.2, fc=th["faint"], ec=th["tpl"], lw=1.4, zorder=2))
    ax.text(0.0, 3.62, "pallet", ha="center", va="center", fontsize=9, color=th["ink"], zorder=3)
    ax.add_patch(Rectangle((-1.2, 1.6), -0.4, 0.4, fc=th["faint"], ec=th["muted"], lw=1, zorder=2))
    ax.text(-1.4, 1.48, "thùng", ha="center", va="top", fontsize=8, color=th["muted"])
    # FOV camera lúc xuất phát
    x0, y0, a0 = X[0], Y[0], YAW[0]
    cam = np.array([x0, y0]) + 0.25 * np.array([math.cos(a0), math.sin(a0)])
    hf = math.radians(87 / 2)
    rng = 3.0
    wedge = [cam, cam + rng * np.array([math.cos(a0 + hf), math.sin(a0 + hf)]),
             cam + rng * np.array([math.cos(a0 - hf), math.sin(a0 - hf)])]
    ax.add_patch(Polygon(np.array(wedge)[:, ::-1], closed=True, fc=th["scene"], alpha=0.10, ec="none",
                         zorder=1))
    ax.text(0.95, 2.55, "tầm nhìn camera\nlúc xuất phát", fontsize=7.5, color=th["scene"], ha="center")
    blind = (np.array([r["t"] for r in gt]) >= tb)
    ax.plot(Y[~blind], X[~blind], color=th["scene"], lw=2.4, label="APPROACH", zorder=6)
    ax.plot(Y[blind], X[blind], color=th["tpl"], lw=3.0, label="BLIND", zorder=6)
    _robot_patch(ax, th, x0, y0, a0)
    _robot_patch(ax, th, X[-1], Y[-1], YAW[-1], ec=th["ok"], fc=th["bg"])
    ax.plot([target[1]], [target[0]], "+", ms=14, mew=2, color=th["ok"], zorder=8)
    ax.text(-0.12, 2.55, "đích", color=th["ok"], fontsize=8.5, zorder=8)
    ax.text(0.30, 0.66, "xuất phát (1,1; 0,35; −15°)", fontsize=8, color=th["muted"], ha="center", va="top")
    topview(ax, xlim=(0.5, 4.3), ylim=(-1.7, 1.1))
    ax.set_xlabel("y world (m)")
    ax.set_ylabel("x world (m)")
    ax.set_title("A. Quỹ đạo thật của robot (ground truth Gazebo)")
    ax.legend(loc="lower right")
    # chuỗi thời gian
    a1 = fig.add_subplot(gsx[0, 1])
    a2 = fig.add_subplot(gsx[1, 1], sharex=a1)
    a3 = fig.add_subplot(gsx[2, 1], sharex=a1)
    for a in (a1, a2):
        a.axvspan(0, tb - t0, color=th["scene"], alpha=0.10, lw=0)
        a.axvspan(tb - t0, td - t0, color=th["tpl"], alpha=0.14, lw=0)
    a1.plot(T, target[0] - X, color=th["ink"], lw=2)
    a1.set_ylabel("còn lại (m)")
    a1.set_title("B. Diễn biến theo thời gian (tính từ lúc gọi ~/start)")
    a1.text(0.15, 0.08, "APPROACH", transform=a1.get_xaxis_transform(), color=th["scene"], fontsize=8)
    a1.text(tb - t0 + 0.2, 0.08, "BLIND", transform=a1.get_xaxis_transform(), color=th["tpl"], fontsize=8)
    a1.annotate(f"DOCKED sau {td - t0:.1f} s", xy=(td - t0, target[0] - X[np.searchsorted(T, td - t0) - 1]),
                xytext=(td - t0 - 6.5, 0.9), fontsize=8.5, color=th["ok"],
                arrowprops=dict(arrowstyle="->", color=th["ok"], lw=1))
    a2.plot(T, 100 * Y, color=th["scene"], lw=2, label="lệch ngang y (cm)")
    a2.plot(T, np.degrees(YAW), color=th["init"], lw=1.6, label="lệch hướng (°)")
    a2.axhline(0, color=th["muted"], lw=0.8)
    a2.set_ylabel("sai số")
    a2.legend(loc="upper right", fontsize=7.8)
    a2.text(T[-1], -7, f"cuối: ngang {100 * Y[-1]:+.1f} cm, yaw {math.degrees(YAW[-1]):+.2f}°",
            ha="right", fontsize=8, color=th["ink"])
    # dải trạng thái estimator
    a3.set_yticks([])
    a3.grid(False)
    cols = {"OK": th["ok"], "BLIND_HOLD": th["tpl"]}
    for (ta, sa), (tb_, _) in zip(st, st[1:] + [(st[-1][0] + 0.1, "")]):
        if ta - t0 < -0.5:
            continue
        a3.axvspan(ta - t0, tb_ - t0, color=cols.get(sa, th["bad"]), lw=0)
    a3.set_ylabel("estimator", rotation=0, ha="right", va="center", fontsize=8)
    a3.text(0.2, 0.5, "OK (đo mới)", transform=a3.get_xaxis_transform(), va="center", fontsize=7.5,
            color="#FFFFFF" if theme == "light" else "#0F1419")
    a3.text(tb - t0 + 2.4, 0.5, "BLIND_HOLD (giữ pose)", transform=a3.get_xaxis_transform(),
            va="center", fontsize=7.5, color="#FFFFFF" if theme == "light" else "#0F1419")
    a3.set_xlim(-0.3, T[-1] + 0.2)
    a3.set_xlabel("thời gian (s)")
    plt.setp(a1.get_xticklabels(), visible=False)
    plt.setp(a2.get_xticklabels(), visible=False)
    n_ok = sum(1 for _, s_ in st if s_ == "OK")
    n_bh = sum(1 for _, s_ in st if s_ == "BLIND_HOLD")
    META["f11"] = dict(t_blind=tb - t0, t_dock=td - t0, final=[float(X[-1]), float(Y[-1]),
                       math.degrees(float(YAW[-1]))], n_ok=n_ok, n_blind_hold=n_bh)
    return save(fig, "f11-sim-run", theme)


# ============================================================================ F12 test E2E
E2E = [  # (x, y, yaw°) xuất phát trong world -> kết quả trong logs/e2e_run*.log
    ((1.6, 0.0, 0.0), (-1.1, 0.0, -0.05), 16),
    ((1.5, 0.25, 15.0), (-1.1, 0.2, 0.82), 19),
    ((1.7, -0.3, -20.0), (-1.1, -0.3, -1.77), 21),
    ((1.4, -0.3, -20.0), (-1.0, 0.0, -0.93), 21),
]


def f12(th, theme):
    fig, ax = plt.subplots(figsize=(10.6, 4.6))
    ax.add_patch(Rectangle((0.4, 3.0), -0.8, 1.2, fc=th["faint"], ec=th["tpl"], lw=1.4))
    ax.text(0.0, 3.6, "pallet\nmặt trước\nx = 3,0 m", ha="center", va="center", fontsize=8.5, color=th["ink"])
    ax.plot([0], [2.65], "+", ms=14, mew=2, color=th["ok"])
    ax.text(-0.05, 2.70, "đích x = 2,65 m\n(dock_offset −0,35)", fontsize=8, color=th["ok"], va="bottom")
    cols = [th["scene"], th["init"], th["tpl"], th["ok"]]
    for k, ((x, y, a), (al, lat, yw), tsec) in enumerate(E2E):
        _robot_patch(ax, th, x, y, math.radians(a), ec=cols[k], fc=th["bg"], lw=1.6)
        lbl = f"#{k + 1}  x={x:.1f}, y={y:+.2f}, yaw={a:+.0f}°  →  ngang {lat:+.1f} cm, yaw {yw:+.2f}°, {tsec} s"
        ax.plot([], [], color=cols[k], lw=3, label=lbl)
    topview(ax, xlim=(1.0, 4.3), ylim=(-0.9, 0.9))
    ax.set_xlabel("y world (m)")
    ax.set_ylabel("x world (m)")
    ax.set_title("4 lần chạy E2E headless: cả 4 DOCKED, PASS")
    ax.legend(loc="center left", bbox_to_anchor=(1.02, 0.5), fontsize=8.5, title="xuất phát → kết quả",
              title_fontsize=9)
    return save(fig, "f12-e2e", theme)


FIGS = {"f01": f01, "f02": f02, "f03": f03, "f04": f04, "f04b": f04b, "f05": f05, "f06": f06,
        "f07": f07, "f08": f08, "f09": f09, "f10": f10, "f11": f11, "f12": f12}


def main(argv):
    names = list(FIGS) if (not argv or argv == ["all"]) else argv
    if os.path.exists(META_PATH):
        try:
            META.update(json.load(open(META_PATH)))
        except Exception:
            pass
    for n in names:
        for theme, th in THEMES.items():
            style(th)
            t0 = time.time()
            size = FIGS[n](th, theme)
            print(f"{n:5s} {theme:5s} {size} {time.time() - t0:.1f}s", flush=True)
    with open(META_PATH, "w") as fh:
        json.dump(META, fh, ensure_ascii=False, indent=1, default=float)
    print(json.dumps({k: v for k, v in META.items() if k in names}, ensure_ascii=False, default=float)[:3000])


if __name__ == "__main__":
    main(sys.argv[1:])
