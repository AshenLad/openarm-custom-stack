import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    config_file = LaunchConfiguration("config_file")
    execute = LaunchConfiguration("execute")
    run_mode = LaunchConfiguration("run_mode")
    allow_execution = LaunchConfiguration("allow_execution")
    allow_full_execution = LaunchConfiguration("allow_full_execution")
    use_receipt_time_for_freshness = LaunchConfiguration(
        "use_receipt_time_for_freshness"
    )
    shutdown_on_exit = LaunchConfiguration("shutdown_on_exit")
    production_demo = LaunchConfiguration("production_demo")

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
                # Expose the grasp helper link generated from the same shifted
                # v2.0 model as the fingers.  mtc_pregrasp reads its transform
                # from RobotModel instead of duplicating stale CAD constants.
                "emit_grasp_frame": "true",
                "use_fake_hardware": "false",
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
    moveit_config["ompl"]["right_arm"] = {
        "planner_configs": ["RRTConnect"],
        "longest_valid_segment_fraction": 0.001,
    }
    response_adapters = moveit_config["ompl"].get("response_adapters", [])
    ruckig = "default_planning_response_adapters/AddRuckigTrajectorySmoothing"
    if ruckig not in response_adapters:
        response_adapters.insert(1 if response_adapters else 0, ruckig)
        moveit_config["ompl"]["response_adapters"] = response_adapters

    mtc_node = Node(
        package="openarm_mtc_pick",
        executable="mtc_pregrasp",
        name="openarm_mtc_pregrasp",
        output="screen",
        parameters=[
            moveit_config,
            config_file,
            {
                "execution.execute": ParameterValue(execute, value_type=bool),
                "execution.allow_execution": ParameterValue(
                    allow_execution, value_type=bool
                ),
                "execution.allow_full_execution": ParameterValue(
                    allow_full_execution, value_type=bool
                ),
                "execution.use_receipt_time_for_freshness": ParameterValue(
                    use_receipt_time_for_freshness, value_type=bool
                ),
                "task.run_mode": run_mode,
                "task.production_demo": ParameterValue(
                    production_demo, value_type=bool
                ),
            },
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "config_file",
            default_value=os.path.join(
                get_package_share_directory("openarm_mtc_pick"), "config", "pick.yaml"
            ),
        ),
        DeclareLaunchArgument("execute", default_value="false"),
        DeclareLaunchArgument(
            "run_mode",
            default_value="plan_only",
            description=(
                "plan_only | open_only | pregrasp_test | approach_test | "
                "close_test | lift_test | place_test | full"
            ),
        ),
        DeclareLaunchArgument("allow_execution", default_value="false"),
        DeclareLaunchArgument("allow_full_execution", default_value="false"),
        DeclareLaunchArgument(
            "use_receipt_time_for_freshness",
            default_value="false",
            description=(
                "Use local monotonic receipt time for freshness checks; intended "
                "only for mixed-clock Gazebo/fake-hardware simulation"
            ),
        ),
        DeclareLaunchArgument("shutdown_on_exit", default_value="false"),
        DeclareLaunchArgument("production_demo", default_value="false"),
        mtc_node,
        RegisterEventHandler(
            OnProcessExit(
                target_action=mtc_node,
                on_exit=[EmitEvent(event=Shutdown(reason="MTC run completed"))],
            ),
            condition=IfCondition(shutdown_on_exit),
        ),
    ])
