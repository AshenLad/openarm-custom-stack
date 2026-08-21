"""Developer entry point exposing stage/plan-only controls.

Real hardware operators should use ``pick.launch.py``.  This launch is kept
only for deterministic fake regression and focused development tests.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    package_share = get_package_share_directory("openarm_mtc_pick")
    return LaunchDescription([
        DeclareLaunchArgument("execute", default_value="false"),
        DeclareLaunchArgument("allow_execution", default_value="false"),
        DeclareLaunchArgument("allow_full_execution", default_value="false"),
        DeclareLaunchArgument(
            "run_mode",
            default_value="plan_only",
            description=(
                "DEVELOPMENT ONLY: plan_only | open_only | pregrasp_test | "
                "approach_test | close_test | lift_test | place_test | full"
            ),
        ),
        DeclareLaunchArgument(
            "use_receipt_time_for_freshness", default_value="false"
        ),
        DeclareLaunchArgument("shutdown_on_exit", default_value="false"),
        DeclareLaunchArgument("production_demo", default_value="false"),
        DeclareLaunchArgument(
            "config_file",
            default_value=os.path.join(package_share, "config", "pick.yaml"),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(package_share, "launch", "pregrasp.launch.py")
            ),
            launch_arguments={
                "config_file": LaunchConfiguration("config_file"),
                "execute": LaunchConfiguration("execute"),
                "allow_execution": LaunchConfiguration("allow_execution"),
                "allow_full_execution": LaunchConfiguration(
                    "allow_full_execution"
                ),
                "use_receipt_time_for_freshness": LaunchConfiguration(
                    "use_receipt_time_for_freshness"
                ),
                "run_mode": LaunchConfiguration("run_mode"),
                "shutdown_on_exit": LaunchConfiguration("shutdown_on_exit"),
                "production_demo": LaunchConfiguration("production_demo"),
            }.items(),
        ),
    ])
