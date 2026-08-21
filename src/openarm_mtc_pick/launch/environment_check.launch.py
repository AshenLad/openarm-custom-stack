import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    config_file = LaunchConfiguration("config_file")
    description_path = os.path.join(
        get_package_share_directory("openarm_description"),
        "assets", "robot", "openarm_v2.0", "urdf", "openarm_v20.urdf.xacro"
    )
    moveit_config = (
        MoveItConfigsBuilder(
            "openarm", package_name="openarm_bimanual_moveit_config"
        )
        .robot_description(
            file_path=description_path,
            mappings={
                "arm_type": "openarm_v2.0",
                "bimanual": "true",
                "use_fake_hardware": "true",
                "ros2_control": "true",
            },
        )
        .robot_description_semantic(
            file_path="config/openarm_v2.0/openarm_bimanual.srdf"
        )
        .robot_description_kinematics(
            file_path="config/openarm_v2.0/kinematics.yaml"
        )
        .joint_limits(file_path="config/openarm_v2.0/joint_limits.yaml")
        .trajectory_execution(
            file_path="config/openarm_v2.0/moveit_controllers.yaml"
        )
        .planning_pipelines(pipelines=["ompl"], default_planning_pipeline="ompl")
        .to_dict()
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "config_file",
            default_value=os.path.join(
                get_package_share_directory("openarm_mtc_pick"), "config", "pick.yaml"
            ),
        ),
        Node(
            package="openarm_mtc_pick",
            executable="mtc_environment_check",
            output="screen",
            parameters=[moveit_config, config_file],
        ),
    ])
