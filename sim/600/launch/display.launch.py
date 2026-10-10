"""Rosie 600 in RViz: robot_state_publisher + joint sliders.

    ros2 launch rosie_600_description display.launch.py
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory("rosie_600_description")
    with open(os.path.join(share, "urdf", "rosie_600.urdf")) as f:
        # The URDF's mesh paths are relative to it; RViz needs package:// URIs.
        description = f.read().replace('filename="../meshes/', 'filename="package://rosie_600_description/meshes/')
    return LaunchDescription([
        Node(package="robot_state_publisher", executable="robot_state_publisher",
             parameters=[{"robot_description": description}]),
        Node(package="joint_state_publisher_gui", executable="joint_state_publisher_gui"),
        Node(package="rviz2", executable="rviz2", arguments=["-d", os.path.join(share, "rviz", "display.rviz")]),
    ])
