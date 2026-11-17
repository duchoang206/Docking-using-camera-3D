"""Bảo đảm tiến trình dùng numpy/scipy của hệ thống (apt) thay vì ~/.local.

Trên máy dev có numpy 2.x cài bằng pip trong ~/.local, làm hỏng scipy.spatial
(biên dịch với numpy 1.26 của apt) và các gói Python của ROS 2 Jazzy.
Gọi ensure_clean_python() ở ĐẦU mỗi entry point, TRƯỚC khi import numpy.
"""
import os
import sys


def ensure_clean_python() -> None:
    """Nếu chưa đặt PYTHONNOUSERSITE=1 thì đặt và khởi động lại chính tiến trình."""
    if os.environ.get("PYTHONNOUSERSITE") == "1":
        return
    os.environ["PYTHONNOUSERSITE"] = "1"
    # sys.orig_argv giữ nguyên dòng lệnh gốc (kể cả "-m module" / "-c code").
    argv = list(getattr(sys, "orig_argv", [sys.executable] + sys.argv))
    os.execv(sys.executable, [sys.executable] + argv[1:])
