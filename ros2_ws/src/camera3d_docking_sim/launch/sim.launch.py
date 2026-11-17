"""Mô phỏng docking: Gazebo Harmonic + robot + bridge + (tùy chọn) estimator/controller + RViz.

ros2 launch camera3d_docking_sim sim.launch.py                       # có GUI Gazebo + RViz
ros2 launch camera3d_docking_sim sim.launch.py headless:=true rviz:=false x:=1.2 y:=0.2 yaw:=10
ros2 service call /docking_controller/start std_srvs/srv/Trigger
"""
import math
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction,
                            SetEnvironmentVariable)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

CAMERA_TF = ["--x", "0.25", "--y", "0", "--z", "0.30", "--roll", "0", "--pitch", "0.1745",
             "--yaw", "0", "--frame-id", "base_link", "--child-frame-id", "camera_link"]


def _setup(context):
    def arg(name):
        return LaunchConfiguration(name).perform(context)

    pkg = get_package_share_directory("camera3d_docking_sim")
    headless = arg("headless") == "true"
    # Chỉ truyền TÊN world: gz tách gz_args theo dấu cách -> đường dẫn có dấu cách
    # ("Docking using Camera 3D") bị vỡ. gz tìm file trong GZ_SIM_RESOURCE_PATH.
    gz_args = f"-r {'-s --headless-rendering ' if headless else ''}docking_world.sdf"
    resource = os.pathsep.join([os.path.join(pkg, "worlds"), os.path.join(pkg, "models"),
                                os.environ.get("GZ_SIM_RESOURCE_PATH", "")])
    actions = [
        SetEnvironmentVariable("GZ_SIM_RESOURCE_PATH", resource),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                get_package_share_directory("ros_gz_sim"), "launch", "gz_sim.launch.py")),
            launch_arguments={"gz_args": gz_args, "on_exit_shutdown": "true"}.items()),
        Node(package="ros_gz_sim", executable="create", output="screen",
             arguments=["-world", "docking_world", "-name", "docking_bot",
                        "-file", os.path.join(pkg, "models", "docking_bot", "model.sdf"),
                        "-x", arg("x"), "-y", arg("y"), "-z", "0.01",
                        "-Y", str(math.radians(float(arg("yaw"))))]),
        Node(package="ros_gz_bridge", executable="parameter_bridge", output="screen",
             parameters=[{"config_file": os.path.join(pkg, "config", "bridge.yaml"),
                          "use_sim_time": True}]),
        Node(package="tf2_ros", executable="static_transform_publisher", arguments=CAMERA_TF,
             parameters=[{"use_sim_time": True}]),
    ]
    if arg("docking") == "true":
        actions.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                get_package_share_directory("camera3d_docking"), "launch", "docking.launch.py")),
            launch_arguments={"points_topic": "/camera/points", "use_sim_time": "true",
                              "template": arg("template")}.items()))
    if arg("rviz") == "true":
        actions.append(Node(
            package="rviz2", executable="rviz2", output="log",
            arguments=["-d", os.path.join(get_package_share_directory("camera3d_docking"),
                                          "rviz", "docking.rviz")],
            parameters=[{"use_sim_time": True}]))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("headless", default_value="false"),
        DeclareLaunchArgument("rviz", default_value="true"),
        DeclareLaunchArgument("docking", default_value="true",
                              description="chạy pose_estimator + docking_controller"),
        DeclareLaunchArgument("template", default_value="euro_pallet.npz"),
        DeclareLaunchArgument("x", default_value="1.0", description="vị trí xuất phát robot (m)"),
        DeclareLaunchArgument("y", default_value="0.0"),
        DeclareLaunchArgument("yaw", default_value="0.0", description="độ"),
        OpaqueFunction(function=_setup),
    ])
