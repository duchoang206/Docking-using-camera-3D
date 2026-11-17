"""Docking qua Nav2: pose_estimator + docking_server (opennav_docking) + lifecycle manager.

Yêu cầu robot đã chạy Nav2 (map, AMCL/SLAM, navigate_to_pose) để tới staging pose.
ros2 launch camera3d_docking nav2_docking.launch.py points_topic:=/camera/points
ros2 action send_goal /dock_robot opennav_docking_msgs/action/DockRobot "{dock_id: pallet_1}"
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
        DeclareLaunchArgument("docking_params", default_value=os.path.join(cfg, "nav2_docking.yaml")),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
    ]
    estimator = Node(
        package="camera3d_docking", executable="pose_estimator", name="pose_estimator",
        output="screen", additional_env={"PYTHONNOUSERSITE": "1"},
        parameters=[lc("estimator_params"), {"template_path": lc("template"),
                                             "use_sim_time": lc("use_sim_time")}],
        remappings=[("points", lc("points_topic"))])
    docking_server = Node(
        package="opennav_docking", executable="opennav_docking", name="docking_server",
        output="screen",
        parameters=[lc("docking_params"), {"use_sim_time": lc("use_sim_time")}])
    lifecycle = Node(
        package="nav2_lifecycle_manager", executable="lifecycle_manager",
        name="lifecycle_manager_docking", output="screen",
        parameters=[{"autostart": True, "node_names": ["docking_server"],
                     "use_sim_time": lc("use_sim_time")}])
    return LaunchDescription(args + [estimator, docking_server, lifecycle])
