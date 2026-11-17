"""Cô lập các test ROS khỏi mọi tiến trình ROS khác trên máy (vd mô phỏng Gazebo đang chạy).

Không cô lập: /tf của mô phỏng (sim time) trộn với TF tĩnh của test -> test điều khiển fail
ngẫu nhiên. Đặt domain riêng + chỉ discovery trong localhost, TRƯỚC khi rclpy.init().
"""
import os

os.environ["ROS_DOMAIN_ID"] = str(50 + os.getpid() % 50)
os.environ["ROS_AUTOMATIC_DISCOVERY_RANGE"] = "LOCALHOST"
