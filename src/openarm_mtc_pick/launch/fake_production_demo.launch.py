"""End-to-end test of the production orchestrator on fake controllers."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def include(package, launch_file, arguments=None):
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory(package), "launch", launch_file
            )
        ),
        launch_arguments=(arguments or {}).items(),
    )


def generate_launch_description():
    package_share = get_package_share_directory("openarm_mtc_pick")
    config_file = os.path.join(package_share, "config", "fake_pick_place.yaml")
    fake_moveit = include(
        "openarm_bimanual_moveit_config",
        "demo.launch.py",
        {
            "arm_type": "openarm_v2.0",
            "use_fake_hardware": "true",
            "robot_controller": "joint_trajectory_controller",
            "launch_rviz": "false",
        },
    )
    fake_geometry = Node(
        package="openarm_mtc_pick",
        executable="fake_blue_cube_pose_publisher.py",
        output="screen",
        parameters=[config_file],
    )
    production_demo = include(
        "openarm_mtc_pick",
        "pick.launch.py",
        {
            "config_file": config_file,
            "start_perception": "false",
            "shutdown_on_completion": "true",
        },
    )
    return LaunchDescription([
        fake_moveit,
        fake_geometry,
        TimerAction(period=5.0, actions=[production_demo]),
    ])
