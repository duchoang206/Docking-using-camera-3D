"""Docking ĐỘC LẬP (không Nav2): pose_estimator + docking_controller.

ros2 launch camera3d_docking docking.launch.py points_topic:=/camera/points template:=euro_pallet.npz
ros2 service call /docking_controller/start std_srvs/srv/Trigger
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory("camera3d_docking")
    cfg = os.path.join(share, "config")
    lc = LaunchConfiguration
    args = [
        DeclareLaunchArgument("points_topic", default_value="/camera/points"),
        DeclareLaunchArgument("template", default_value="euro_pallet.npz"),
        DeclareLaunchArgument("estimator_params", default_value=os.path.join(cfg, "estimator.yaml")),
        DeclareLaunchArgument("controller_params", default_value=os.path.join(cfg, "controller.yaml")),
        DeclareLaunchArgument("cmd_vel_topic", default_value="/cmd_vel"),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
    ]
    env = {"PYTHONNOUSERSITE": "1"}
    estimator = Node(
        package="camera3d_docking", executable="pose_estimator", name="pose_estimator",
        output="screen", additional_env=env,
        parameters=[lc("estimator_params"), {"template_path": lc("template"),
                                             "use_sim_time": lc("use_sim_time")}],
        remappings=[("points", lc("points_topic"))])
    controller = Node(
        package="camera3d_docking", executable="docking_controller", name="docking_controller",
        output="screen", additional_env=env,
        parameters=[lc("controller_params"), {"use_sim_time": lc("use_sim_time")}],
        remappings=[("cmd_vel", lc("cmd_vel_topic"))])
    return LaunchDescription(args + [estimator, controller])
