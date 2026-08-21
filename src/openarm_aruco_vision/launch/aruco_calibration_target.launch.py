from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
import os


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory("openarm_aruco_vision"),
        "config",
        "aruco_calibration.yaml",
    )
    return LaunchDescription([
        DeclareLaunchArgument("dictionary", default_value="DICT_6X6_50"),
        DeclareLaunchArgument("marker_id", default_value="0"),
        DeclareLaunchArgument("marker_size_m", default_value="0.080"),
        Node(
            package="openarm_aruco_vision",
            executable="aruco_detector",
            name="aruco_detector",
            output="screen",
            parameters=[
                config,
                {
                    "dictionary": ParameterValue(
                        LaunchConfiguration("dictionary"), value_type=str
                    ),
                    "marker_id": ParameterValue(
                        LaunchConfiguration("marker_id"), value_type=int
                    ),
                    "marker_size_m": ParameterValue(
                        LaunchConfiguration("marker_size_m"), value_type=float
                    ),
                },
            ],
        )
    ])
