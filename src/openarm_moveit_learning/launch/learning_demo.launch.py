from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    description_share = Path(
        get_package_share_directory("openarm_description")
    )
    urdf_path = (
        description_share
        / "assets/robot/openarm_v2.0/urdf/openarm_v20.urdf.xacro"
    )
    config_dir = "config/openarm_v2.0"

    config = (
        MoveItConfigsBuilder(
            "openarm", package_name="openarm_bimanual_moveit_config"
        )
        .robot_description(
            file_path=str(urdf_path),
            mappings={
                "arm_type": "openarm_v2.0",
                "bimanual": "true",
                "emit_grasp_frame": "true",
                "use_fake_hardware": "true",
                "use_sim_hardware": "false",
                "ros2_control": "true",
            },
        )
        .robot_description_semantic(
            file_path=f"{config_dir}/openarm_bimanual.srdf"
        )
        .robot_description_kinematics(
            file_path=f"{config_dir}/kinematics.yaml"
        )
        .joint_limits(file_path=f"{config_dir}/joint_limits.yaml")
        .trajectory_execution(
            file_path=f"{config_dir}/moveit_controllers.yaml"
        )
        .planning_pipelines(
            pipelines=["ompl"], default_planning_pipeline="ompl"
        )
        .to_moveit_configs()
    )

    return LaunchDescription([
        Node(
            package="openarm_moveit_learning",
            executable="minimal_moveit_demo",
            name="minimal_moveit_demo",
            output="screen",
            parameters=[
                config.robot_description,
                config.robot_description_semantic,
                config.robot_description_kinematics,
            ],
        )
    ])