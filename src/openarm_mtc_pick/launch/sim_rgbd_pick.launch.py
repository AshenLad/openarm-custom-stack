import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    IncludeLaunchDescription,
    LogInfo,
    RegisterEventHandler,
    TimerAction,
)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
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
    config_file = os.path.join(package_share, "config", "pick.yaml")

    # Gazebo publishes only RGB-D sensor data. The existing perception nodes
    # must recover /box/* and /blue_cube/*; no fake geometry topic is injected.
    simulated_perception = include(
        "openarm_aruco_vision",
        "sim_rgbd_scene_detection.launch.py",
        {
            "gui": LaunchConfiguration("gazebo_gui"),
            "world_file": LaunchConfiguration("world_file"),
        },
    )
    fake_moveit = include(
        "openarm_bimanual_moveit_config",
        "demo.launch.py",
        {
            "arm_type": "openarm_v2.0",
            "use_fake_hardware": "true",
            "robot_controller": "joint_trajectory_controller",
        },
    )

    scene = Node(
        package="openarm_mtc_pick",
        executable="scene_manager",
        name="openarm_mtc_scene_manager",
        output="screen",
        parameters=[config_file],
    )
    pick = include(
        "openarm_mtc_pick",
        "dev_pick.launch.py",
        {
            "config_file": config_file,
            "execute": LaunchConfiguration("execute_fake"),
            "allow_execution": LaunchConfiguration("execute_fake"),
            # Gazebo RGB-D -> detector -> fake controllers is the only place
            # allowed to satisfy the full-execution latch automatically.
            "allow_full_execution": LaunchConfiguration("execute_fake"),
            "run_mode": "full",
            # Gazebo image headers use simulation time while fake ros2_control
            # uses wall time. Local monotonic receipt age remains meaningful.
            "use_receipt_time_for_freshness": "true",
        },
    )

    # scene_manager waits indefinitely for a synchronized visual box and exits
    # after applying it. Only then start MTC, eliminating the old timer race in
    # which object perception could finish before the table collision existed.
    def start_pick_if_scene_succeeded(event, _context):
        if event.returncode == 0:
            return [TimerAction(period=1.0, actions=[pick])]
        reason = (
            "scene_manager failed before applying the visually measured box; "
            "MTC will not start"
        )
        return [LogInfo(msg=f"ERROR: {reason}"), EmitEvent(event=Shutdown(reason=reason))]

    start_pick_after_scene = RegisterEventHandler(
        OnProcessExit(
            target_action=scene,
            on_exit=start_pick_if_scene_succeeded,
        )
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "gazebo_gui", default_value="true", description="Open Gazebo GUI"
        ),
        DeclareLaunchArgument(
            "execute_fake",
            default_value="false",
            description="Execute only on the fake controllers started here",
        ),
        DeclareLaunchArgument(
            "world_file",
            default_value="rgbd_pick_scene_reachable.sdf",
            description=(
                "Gazebo RGB-D world; no object or box truth is injected into ROS"
            ),
        ),
        fake_moveit,
        simulated_perception,
        start_pick_after_scene,
        TimerAction(period=5.0, actions=[scene]),
    ])
