"""Task 0 — môi trường Python: phải dùng numpy/scipy của apt (không lấy ~/.local)."""
import os
import subprocess
import sys


def test_env_flag_set_under_pytest():
    import numpy

    assert os.environ.get("PYTHONNOUSERSITE") == "1"
    assert numpy.__version__.startswith("1."), numpy.__version__


def test_scipy_spatial_importable():
    from scipy.spatial import cKDTree
    import numpy as np

    tree = cKDTree(np.zeros((3, 3)))
    assert tree.n == 3


def test_ensure_clean_python_reexecs_without_flag():
    env = {k: v for k, v in os.environ.items() if k != "PYTHONNOUSERSITE"}
    pkg_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    env["PYTHONPATH"] = pkg_root + os.pathsep + env.get("PYTHONPATH", "")
    code = (
        "from camera3d_docking._env import ensure_clean_python; "
        "ensure_clean_python(); import os, numpy; "
        "print(os.environ['PYTHONNOUSERSITE'], numpy.__version__[0])"
    )
    out = subprocess.run(
        [sys.executable, "-c", code], env=env, capture_output=True, text=True, timeout=60
    )
    assert out.returncode == 0, out.stderr
    assert out.stdout.split() == ["1", "1"]
