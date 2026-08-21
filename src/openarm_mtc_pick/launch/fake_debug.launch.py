import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource


def include(package, launch_file, arguments=None):
    source = PythonLaunchDescriptionSource(
        os.path.join(get_package_share_directory(package), "launch", launch_file)
    )
    return IncludeLaunchDescription(
        source,
        launch_arguments=(arguments or {}).items(),
    )


def generate_launch_description():
    package_share = get_package_share_directory("openarm_mtc_pick")
    fake_config = os.path.join(package_share, "config", "fake_debug.yaml")

    # This launch file intentionally hard-codes fake hardware. It must not accept
    # a use_real_hardware override.
    openarm_fake = include(
        "openarm_bimanual_moveit_config",
        "demo.launch.py",
        {
            "arm_type": "openarm_v2.0",
            "use_fake_hardware": "true",
            "robot_controller": "joint_trajectory_controller",
        },
    )
    scene = include(
        "openarm_mtc_pick",
        "scene_manager.launch.py",
        {"config_file": fake_config},
    )
    check = include(
        "openarm_mtc_pick",
        "environment_check.launch.py",
        {"config_file": fake_config},
    )

    return LaunchDescription([
        openarm_fake,
        TimerAction(period=5.0, actions=[scene]),
        TimerAction(period=6.0, actions=[check]),
    ])
