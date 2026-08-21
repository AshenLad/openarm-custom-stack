import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    package_share = get_package_share_directory("openarm_aruco_vision")
    return LaunchDescription([
        Node(
            package="openarm_aruco_vision",
            executable="box_detector",
            name="box_detector",
            output="screen",
            parameters=[os.path.join(package_share, "config", "box.yaml")],
        ),
        Node(
            package="openarm_aruco_vision",
            executable="blue_cube_detector",
            name="blue_cube_detector",
            output="screen",
            parameters=[os.path.join(package_share, "config", "blue_cube.yaml")],
        ),
    ])
