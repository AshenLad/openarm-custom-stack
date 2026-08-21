import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory("openarm_aruco_vision"),
        "config",
        "blue_cube.yaml",
    )
    return LaunchDescription([
        Node(
            package="openarm_aruco_vision",
            executable="blue_cube_detector",
            name="blue_cube_detector",
            output="screen",
            parameters=[config],
        )
    ])
