"""Sinh world Gazebo (SDF 1.9) từ CÙNG mô hình hộp với template (camera3d_docking.core.shapes)
-> hình học trong mô phỏng và template khớp nhau tuyệt đối.

    python3 -m camera3d_docking_sim.generate_world            # ghi worlds/docking_world.sdf
    python3 -m camera3d_docking_sim.generate_world --pallet 3.0 0.0 0
"""
from __future__ import annotations

import argparse
import math
import os
import sys

from camera3d_docking.core import shapes

PALLET_POSE = (3.0, 0.0, 0.0)   # (x, y, yaw_deg) của mặt trước pallet trong world


def _boxes_xml(boxes, rgb) -> str:
    out = []
    color = f"{rgb[0]} {rgb[1]} {rgb[2]} 1"
    for i, b in enumerate(boxes):
        pose = f"{b.center[0]:.5f} {b.center[1]:.5f} {b.center[2]:.5f} 0 0 {b.yaw:.6f}"
        size = f"{b.size[0]:.5f} {b.size[1]:.5f} {b.size[2]:.5f}"
        geom = f"<geometry><box><size>{size}</size></box></geometry>"
        out.append(f'<collision name="c{i}"><pose>{pose}</pose>{geom}</collision>')
        out.append(f'<visual name="v{i}"><pose>{pose}</pose>{geom}'
                   f"<material><ambient>{color}</ambient><diffuse>{color}</diffuse></material></visual>")
    return "\n        ".join(out)


def _static_model(name, boxes, pose, rgb) -> str:
    x, y, yaw_deg = pose
    return (f'    <model name="{name}">\n      <static>true</static>\n'
            f"      <pose>{x} {y} 0 0 0 {math.radians(yaw_deg):.6f}</pose>\n"
            f'      <link name="link">\n        {_boxes_xml(boxes, rgb)}\n      </link>\n    </model>\n')


def world_sdf(pallet_pose=PALLET_POSE, rack_pose=(3.0, 2.6, 0.0)) -> str:
    plugins = "".join(
        f'    <plugin filename="gz-sim-{p}-system" name="gz::sim::systems::{n}"/>\n'
        for p, n in (("physics", "Physics"), ("user-commands", "UserCommands"),
                     ("scene-broadcaster", "SceneBroadcaster")))
    plugins += ('    <plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors">'
                "<render_engine>ogre2</render_engine></plugin>\n")
    ground = ('    <model name="ground_plane">\n      <static>true</static>\n      <link name="link">\n'
              '        <collision name="c"><geometry><plane><normal>0 0 1</normal><size>40 40</size>'
              "</plane></geometry></collision>\n"
              '        <visual name="v"><geometry><plane><normal>0 0 1</normal><size>40 40</size>'
              "</plane></geometry><material><ambient>0.6 0.6 0.6 1</ambient>"
              "<diffuse>0.6 0.6 0.6 1</diffuse></material></visual>\n      </link>\n    </model>\n")
    wall = [shapes.Box((0.1, 0.0, 1.0), (0.2, 8.0, 2.0))]
    clutter = [shapes.Box((0.0, 0.0, 0.25), (0.4, 0.4, 0.5))]
    return (
        '<?xml version="1.0" ?>\n<sdf version="1.9">\n  <world name="docking_world">\n'
        '    <physics name="4ms" type="ignored"><max_step_size>0.004</max_step_size>'
        "<real_time_factor>1.0</real_time_factor></physics>\n"
        f"{plugins}"
        '    <light type="directional" name="sun"><cast_shadows>true</cast_shadows>'
        "<pose>0 0 10 0 0 0</pose><diffuse>0.9 0.9 0.9 1</diffuse><specular>0.2 0.2 0.2 1</specular>"
        "<direction>-0.4 0.2 -0.9</direction></light>\n"
        f"{ground}"
        + _static_model("pallet", shapes.euro_pallet_boxes(), pallet_pose, (0.76, 0.6, 0.42))
        + _static_model("rack", shapes.rack_boxes(), rack_pose, (0.2, 0.35, 0.8))
        + _static_model("wall", wall, (5.2, 0.0, 0.0), (0.85, 0.85, 0.8))
        + _static_model("clutter", clutter, (1.8, -1.4, 0.0), (0.3, 0.3, 0.3))
        + "  </world>\n</sdf>\n")


def write_world(path, pallet_pose=PALLET_POSE, rack_pose=(3.0, 2.6, 0.0)) -> None:
    with open(path, "w", encoding="utf-8") as f:
        f.write(world_sdf(pallet_pose, rack_pose))


def main(argv=None) -> int:
    default_out = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                               "worlds", "docking_world.sdf")
    ap = argparse.ArgumentParser(description="Sinh world Gazebo cho mô phỏng docking")
    ap.add_argument("-o", "--output", default=default_out)
    ap.add_argument("--pallet", type=float, nargs=3, default=list(PALLET_POSE),
                    metavar=("X", "Y", "YAW_DEG"))
    a = ap.parse_args(argv if argv is not None else sys.argv[1:])
    write_world(a.output, tuple(a.pallet))
    print(f"Đã ghi {a.output}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
