import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


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
    config_file = os.path.join(package_share, "config", "fake_pick_place.yaml")
    execute_fake = LaunchConfiguration("execute_fake")
    cube_position = ParameterValue(
        [
            "[",
            LaunchConfiguration("cube_x"),
            ",",
            LaunchConfiguration("cube_y"),
            ",",
            LaunchConfiguration("cube_z"),
            "]",
        ],
        value_type=list[float],
    )
    cube_size = ParameterValue(
        [
            "[",
            LaunchConfiguration("cube_size_x"),
            ",",
            LaunchConfiguration("cube_size_y"),
            ",",
            LaunchConfiguration("cube_size_z"),
            "]",
        ],
        value_type=list[float],
    )
    box_position = ParameterValue(
        [
            "[",
            LaunchConfiguration("box_x"),
            ",",
            LaunchConfiguration("box_y"),
            ",",
            LaunchConfiguration("box_z"),
            "]",
        ],
        value_type=list[float],
    )
    box_size = ParameterValue(
        [
            "[",
            LaunchConfiguration("box_size_x"),
            ",",
            LaunchConfiguration("box_size_y"),
            ",",
            LaunchConfiguration("box_size_z"),
            "]",
        ],
        value_type=list[float],
    )

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
    fake_pose = Node(
        package="openarm_mtc_pick",
        executable="fake_blue_cube_pose_publisher.py",
        output="screen",
        parameters=[
            config_file,
            {
                "position": cube_position,
                "size": cube_size,
                "yaw": ParameterValue(
                    LaunchConfiguration("cube_yaw"), value_type=float
                ),
            },
        ],
    )
    scene = Node(
        package="openarm_mtc_pick",
        executable="scene_manager",
        output="screen",
        parameters=[
            config_file,
            {
                "table.position": box_position,
                "table.size": box_size,
                "table.yaw": ParameterValue(
                    LaunchConfiguration("box_yaw"), value_type=float
                ),
            },
        ],
    )
    pick_place = include(
        "openarm_mtc_pick",
        "dev_pick.launch.py",
        {
            "config_file": config_file,
            "execute": execute_fake,
            "allow_execution": execute_fake,
            # The full fake pipeline requires the same latch as real hardware;
            # execute_fake is the only permitted way to satisfy it here.
            "allow_full_execution": execute_fake,
            "run_mode": LaunchConfiguration("run_mode"),
            "production_demo": LaunchConfiguration("production_demo"),
            "shutdown_on_exit": "true",
        },
    )

    # This launch hard-codes fake hardware. Even execute_fake:=true can only
    # command the mock controllers created above.
    return LaunchDescription([
        DeclareLaunchArgument("execute_fake", default_value="false"),
        DeclareLaunchArgument(
            "production_demo",
            default_value="false",
            description="Exercise the production one-shot MTC path on fake hardware",
        ),
        DeclareLaunchArgument(
            "run_mode",
            default_value="full",
            description=(
                "plan_only | open_only | pregrasp_test | approach_test | "
                "close_test | lift_test | place_test | full"
            ),
        ),
        # Defaults mirror the 08-21 full hardware log, including the current
        # rotated box and the visually measured generic object. They are only
        # a regression fixture; production pick.launch.py consumes live topics.
        DeclareLaunchArgument("cube_x", default_value="0.3024"),
        DeclareLaunchArgument("cube_y", default_value="-0.0448"),
        DeclareLaunchArgument("cube_z", default_value="0.270"),
        DeclareLaunchArgument("cube_yaw", default_value="-0.0647"),
        DeclareLaunchArgument("cube_size_x", default_value="0.035"),
        DeclareLaunchArgument("cube_size_y", default_value="0.033"),
        DeclareLaunchArgument("cube_size_z", default_value="0.042"),
        DeclareLaunchArgument("box_x", default_value="0.324"),
        DeclareLaunchArgument("box_y", default_value="-0.034"),
        DeclareLaunchArgument("box_z", default_value="0.123"),
        DeclareLaunchArgument("box_size_x", default_value="0.253"),
        DeclareLaunchArgument("box_size_y", default_value="0.169"),
        DeclareLaunchArgument("box_size_z", default_value="0.242"),
        DeclareLaunchArgument("box_yaw", default_value="1.490"),
        fake_moveit,
        fake_pose,
        TimerAction(period=5.0, actions=[scene]),
        TimerAction(period=7.0, actions=[pick_place]),
    ])
