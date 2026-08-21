import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    config_file = LaunchConfiguration("config_file")
    return LaunchDescription([
        DeclareLaunchArgument(
            "config_file",
            default_value=os.path.join(
                get_package_share_directory("openarm_mtc_pick"), "config", "pick.yaml"
            ),
        ),
        Node(
            package="openarm_mtc_pick",
            executable="scene_manager",
            output="screen",
            parameters=[config_file],
        ),
    ])
