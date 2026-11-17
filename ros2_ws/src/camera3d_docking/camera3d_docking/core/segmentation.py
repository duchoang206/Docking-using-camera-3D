"""Mặt nạ segmentation 2D (ảnh) -> chọn điểm của vật trong đám mây điểm organized.

- Node ROS: nhận mặt nạ (khác 0 = vật) từ một mạng segmentation bất kỳ (YOLO-seg, SegFormer,
  ...) chạy trên ảnh màu ĐÃ CĂN với depth, rồi đưa mặt nạ về đúng lưới của cloud organized.
- Thí nghiệm: mô phỏng lỗi điển hình của mặt nạ thật để đo xem segmentation có làm docking
  chính xác hơn không (tools/benchmark.py --seg-study).
"""
from __future__ import annotations

import numpy as np
from scipy import ndimage

MODES = ("oracle", "dilate", "erode", "bbox", "shift")


def degrade_mask(mask, mode: str = "oracle", px: int = 3) -> np.ndarray:
    """Tạo mặt nạ "giống mạng thật" từ mặt nạ đúng (H, W) bool.

    oracle: giữ nguyên (mạng hoàn hảo) | dilate: lem ra nền px pixel | erode: hụt mép px pixel |
    bbox: chỉ có hình chữ nhật bao (detector, nới px) | shift: lệch ngang px pixel (ảnh màu
    không căn khớp depth, hoặc dùng mặt nạ của khung trước khi robot đang chạy).
    """
    m = np.asarray(mask, dtype=bool)
    px = max(0, int(px))
    if mode == "oracle":
        return m.copy()
    if mode == "dilate":
        return ndimage.binary_dilation(m, iterations=px) if px else m.copy()
    if mode == "erode":
        return ndimage.binary_erosion(m, iterations=px) if px else m.copy()
    if mode == "bbox":
        out = np.zeros_like(m)
        rows, cols = np.flatnonzero(m.any(axis=1)), np.flatnonzero(m.any(axis=0))
        if len(rows):
            out[max(0, rows[0] - px):rows[-1] + px + 1, max(0, cols[0] - px):cols[-1] + px + 1] = True
        return out
    if mode == "shift":
        out = np.zeros_like(m)
        if px < m.shape[1]:
            out[:, px:] = m[:, :m.shape[1] - px]
        return out
    raise ValueError(f"Kiểu mặt nạ không hỗ trợ: {mode!r} (chọn: {', '.join(MODES)})")


def mask_to_grid(mask, height: int, width: int, aspect_tol: float = 0.02) -> np.ndarray:
    """Đưa mặt nạ (h, w) về lưới (height, width) của cloud organized (lấy mẫu gần nhất).

    Mặt nạ và cloud phải cùng khung hình (vd. RealSense align_depth + ordered_pc). Khác tỉ lệ
    khung hình -> ValueError, vì ghép pixel khi đó chắc chắn sai.
    """
    m = np.asarray(mask, dtype=bool)
    if m.ndim != 2 or m.size == 0:
        raise ValueError(f"Mặt nạ phải là ảnh 2D, nhận shape {m.shape}")
    h, w = m.shape
    if (h, w) == (height, width):
        return m
    if abs(h / w - height / width) > aspect_tol * (height / width):
        raise ValueError(f"Tỉ lệ khung mặt nạ {w}x{h} khác cloud {width}x{height}")
    rows = (np.arange(height) * h) // height
    cols = (np.arange(width) * w) // width
    return m[np.ix_(rows, cols)]
