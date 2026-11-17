"""ros2 launch camera3d_docking estimator.launch.py points_topic:=/camera/points template:=euro_pallet.npz"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory("camera3d_docking")
    args = [
        DeclareLaunchArgument("points_topic", default_value="/camera/points"),
        DeclareLaunchArgument("template", default_value="euro_pallet.npz"),
        DeclareLaunchArgument("params_file", default_value=os.path.join(share, "config", "estimator.yaml")),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
    ]
    node = Node(
        package="camera3d_docking", executable="pose_estimator", name="pose_estimator",
        output="screen",
        parameters=[LaunchConfiguration("params_file"),
                    {"template_path": LaunchConfiguration("template"),
                     "use_sim_time": LaunchConfiguration("use_sim_time")}],
        remappings=[("points", LaunchConfiguration("points_topic"))],
        additional_env={"PYTHONNOUSERSITE": "1"},
    )
    return LaunchDescription(args + [node])
