"""Task 14 — world Gazebo sinh từ CÙNG mô hình hộp với template"""
import shutil
import subprocess
import xml.etree.ElementTree as ET

import pytest

from camera3d_docking.core import shapes as s
from camera3d_docking_sim import generate_world as gw


def test_world_contains_models_from_shapes(tmp_path):
    out = tmp_path / "w.sdf"
    gw.write_world(out, pallet_pose=(3.0, 0.0, 0.0))
    world = ET.parse(out).getroot().find("world")
    models = {m.get("name"): m for m in world.findall("model")}
    assert {"ground_plane", "pallet", "rack", "wall", "clutter"} <= set(models)
    assert len(models["pallet"].findall(".//collision/geometry/box/size")) == len(s.euro_pallet_boxes())
    assert float(models["pallet"].find("pose").text.split()[0]) == 3.0
    plugins = {p.get("filename") for p in world.findall("plugin")}
    assert {"gz-sim-sensors-system", "gz-sim-physics-system", "gz-sim-scene-broadcaster-system"} <= plugins


def test_world_is_valid_sdf(tmp_path):
    if shutil.which("gz") is None:
        pytest.skip("cần Gazebo (gz) — source /opt/ros/jazzy/setup.bash")
    out = tmp_path / "w.sdf"
    gw.write_world(out)
    r = subprocess.run(["gz", "sdf", "-k", str(out)], capture_output=True, text=True, timeout=120)
    assert r.returncode == 0 and "Valid" in r.stdout, r.stdout + r.stderr
