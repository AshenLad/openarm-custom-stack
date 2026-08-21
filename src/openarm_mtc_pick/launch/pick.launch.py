"""Production one-shot visual pick/place entry point.

The camera, calibrated TF and real MoveIt/ros2_control stack are expected to
already be running.  This launch owns perception for one transaction, waits
for a stable visual box, applies that exact PlanningScene, then starts one MTC
process which plans and immediately executes its own selected full solution.
No plan-only or stage execution controls are exposed here.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
    RegisterEventHandler,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def include(package, launch_file, arguments=None, condition=None):
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory(package), "launch", launch_file
            )
        ),
        launch_arguments=(arguments or {}).items(),
        condition=condition,
    )


def generate_launch_description():
    package_share = get_package_share_directory("openarm_mtc_pick")

    def launch_setup(context):
        # Resolve every public argument while this included launch's scope is
        # active. The scene process exits later, after that scope is gone.
        resolved_config = context.perform_substitution(
            LaunchConfiguration("config_file")
        )
        resolved_start_perception = context.perform_substitution(
            LaunchConfiguration("start_perception")
        )
        resolved_shutdown = context.perform_substitution(
            LaunchConfiguration("shutdown_on_completion")
        )
        perception = include(
            "openarm_aruco_vision",
            "rgbd_scene_detection.launch.py",
            condition=IfCondition(resolved_start_perception),
        )
        scene = Node(
            package="openarm_mtc_pick",
            executable="scene_manager",
            # Keep the configured node name: ROS 2 parameter-file sections are
            # keyed by it (openarm_mtc_scene_manager in pick.yaml).
            name="openarm_mtc_scene_manager",
            output="screen",
            parameters=[resolved_config],
        )

        def start_full_if_scene_succeeded(event, _event_context):
            if event.returncode == 0:
                full_task = include(
                    "openarm_mtc_pick",
                    "pregrasp.launch.py",
                    {
                        "config_file": resolved_config,
                        "run_mode": "full",
                        "production_demo": "true",
                        "execute": "true",
                        "allow_execution": "true",
                        "allow_full_execution": "true",
                        "use_receipt_time_for_freshness": "false",
                        "shutdown_on_exit": resolved_shutdown,
                    },
                )
                return [
                    LogInfo(
                        msg=(
                            "Stable visual box committed to PlanningScene; "
                            "starting one-shot full plan-and-execute"
                        )
                    ),
                    TimerAction(period=0.5, actions=[full_task]),
                ]
            reason = (
                "visual PlanningScene transaction failed; "
                "full execution not started"
            )
            return [
                LogInfo(msg=f"ERROR: {reason}"),
                EmitEvent(event=Shutdown(reason=reason)),
            ]

        start_full_after_scene = RegisterEventHandler(
            OnProcessExit(
                target_action=scene,
                on_exit=start_full_if_scene_succeeded,
            )
        )
        return [perception, start_full_after_scene, scene]

    return LaunchDescription([
        DeclareLaunchArgument(
            "config_file",
            default_value=os.path.join(package_share, "config", "pick.yaml"),
        ),
        DeclareLaunchArgument(
            "start_perception",
            default_value="true",
            description=(
                "Start the production box/object detectors for this one-shot run"
            ),
        ),
        DeclareLaunchArgument(
            "shutdown_on_completion",
            default_value="true",
            description="Stop this one-shot perception/task launch after MTC exits",
        ),
        OpaqueFunction(function=launch_setup),
    ])
