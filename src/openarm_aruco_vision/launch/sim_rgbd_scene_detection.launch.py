import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    package_share = get_package_share_directory("openarm_aruco_vision")
    ros_gz_share = get_package_share_directory("ros_gz_sim")
    world = PathJoinSubstitution(
        [package_share, "worlds", LaunchConfiguration("world_file")]
    )
    bridge_config = os.path.join(package_share, "config", "sim_rgbd_bridge.yaml")

    gazebo_gui = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_share, "launch", "gz_sim.launch.py")
        ),
        launch_arguments={"gz_args": ["-r ", world]}.items(),
        condition=IfCondition(LaunchConfiguration("gui")),
    )
    gazebo_headless = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_share, "launch", "gz_sim.launch.py")
        ),
        launch_arguments={"gz_args": ["-s -r ", world]}.items(),
        condition=UnlessCondition(LaunchConfiguration("gui")),
    )

    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="sim_rgbd_bridge",
        output="screen",
        parameters=[{"config_file": bridge_config}],
    )
    # Gazebo camera frame (+X forward) converted to ROS optical convention
    # (+Z forward, +X right, +Y down) for the overhead camera pose in the SDF.
    camera_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="sim_rgbd_camera_tf",
        output="screen",
        arguments=[
            "--x", "0.431", "--y", "0.0", "--z", "0.85",
            "--roll", "3.141592653589793", "--pitch", "0.0",
            "--yaw", "-1.5707963267948966",
            "--frame-id", "world",
            "--child-frame-id", "global_camera_color_optical_frame",
        ],
    )
    box_detector = Node(
        package="openarm_aruco_vision",
        executable="box_detector",
        name="box_detector",
        output="screen",
        parameters=[os.path.join(package_share, "config", "box.yaml")],
    )
    object_detector = Node(
        package="openarm_aruco_vision",
        executable="blue_cube_detector",
        name="blue_cube_detector",
        output="screen",
        parameters=[os.path.join(package_share, "config", "blue_cube.yaml")],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "gui", default_value="true", description="Open the Gazebo GUI"
        ),
        DeclareLaunchArgument(
            "world_file",
            default_value="rgbd_pick_scene_reachable.sdf",
            description=(
                "RGB-D-only Gazebo world. Use rgbd_pick_scene.sdf to reproduce "
                "the current measured but vertically unreachable placement."
            ),
        ),
        gazebo_gui,
        gazebo_headless,
        bridge,
        camera_tf,
        box_detector,
        object_detector,
    ])
