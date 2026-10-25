#include <chrono>
#include <memory>
#include <thread>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <moveit/robot_state/robot_state.hpp>
#include <moveit/robot_model/joint_model_group.hpp>

namespace
{

int run_demo(const rclcpp::Node::SharedPtr & node)
{
    auto logger = node->get_logger();

    moveit::planning_interface::MoveGroupInterface move_group(node, "left_arm");
    RCLCPP_INFO(logger, "Planning Group: %s", move_group.getName().c_str());

    auto current_state = move_group.getCurrentState(2.0);
    if (!current_state) {
        RCLCPP_ERROR(logger, "Failed to get current robot state");
        return 1;
    }

    const auto eef_link = move_group.getEndEffectorLink();
    RCLCPP_INFO(logger, "Planning frame: %s",
                move_group.getPlanningFrame().c_str());

    RCLCPP_INFO(logger, "End effector link: %s",
                eef_link.c_str());

    if (eef_link.empty()) {
        RCLCPP_ERROR(logger, "No end effector link configured");
        return 1;
    }

    const auto pose = move_group.getCurrentPose(eef_link);

    RCLCPP_INFO(logger, "Pose frame: %s",
                pose.header.frame_id.c_str());

    RCLCPP_INFO(logger, "Position: x=%.4f, y=%.4f, z=%.4f",
                pose.pose.position.x,
                pose.pose.position.y,
                pose.pose.position.z);

    RCLCPP_INFO(logger, "Orientation: x=%.4f, y=%.4f, z=%.4f, w=%.4f",
                pose.pose.orientation.x,
                pose.pose.orientation.y,
                pose.pose.orientation.z,
                pose.pose.orientation.w);

    auto target_pose = pose;
    target_pose.pose.position.z += 0.01;

    move_group.setStartStateToCurrentState();

    if (!move_group.setPoseTarget(target_pose, eef_link)) {
        RCLCPP_ERROR(logger, "Failed to set pose target");
        return 1;
    }

    
    const auto * joint_group =
        current_state->getRobotModel()->getJointModelGroup("left_arm");
    if (!joint_group) {
        RCLCPP_ERROR(logger, "left_arm group not found");
        return 1;
    }

    RCLCPP_INFO(
        logger,
        "Target position: x=%.4f, y=%.4f, z=%.4f",
        target_pose.pose.position.x,
        target_pose.pose.position.y,
        target_pose.pose.position.z
    );

    moveit::planning_interface::MoveGroupInterface::Plan pose_plan;
    auto pose_result = move_group.plan(pose_plan);

    if (!pose_result) {
        RCLCPP_ERROR(logger, "Pose planning failed: %d", pose_result.val);
        return 1;
    }

    RCLCPP_INFO(logger, "Pose planning succeeded!");

    //正运动学验证
    const auto& pose_traj = pose_plan.trajectory.joint_trajectory;
    if (pose_traj.points.empty() ||
        pose_traj.joint_names.size() != pose_traj.points.back().positions.size()) {
        RCLCPP_ERROR(logger, "Invalid planned trajectory");
        return 1;
    }

    moveit::core::RobotState final_state(*current_state);

    final_state.setVariablePositions(
        pose_traj.joint_names,
        pose_traj.points.back().positions
    );
    final_state.update();

    const auto& final_tf = final_state.getGlobalLinkTransform(eef_link);

    RCLCPP_INFO(
        logger,
        "FK final position: x=%.4f, y=%.4f, z=%.4f",
        final_tf.translation().x(),
        final_tf.translation().y(),
        final_tf.translation().z()
    );

    return 0; // 本次只读取位姿，不再执行下面旧的规划代码
//////////////////////////////////////////////////////////

    std::vector<double> positions;
    current_state->copyJointGroupPositions(joint_group, positions);
    if (positions.size() != 7) {
        RCLCPP_ERROR(logger, "Unexpected joint count: %zu", positions.size());
        return 1;
    }

    positions[0] += 0.1;
    RCLCPP_INFO(logger, "Target joint1: %.6f rad", positions[0]);

    move_group.setStartStateToCurrentState();
    if (!move_group.setJointValueTarget(positions)) {
        RCLCPP_ERROR(logger, "Joint target is invalid or out of bounds");
        return 1;
    }

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    auto result = move_group.plan(plan);
    if (!result) {
        RCLCPP_ERROR(logger, "Planning failed, error code: %d", result.val);
        return 1;
    }

    const auto & traj = plan.trajectory.joint_trajectory;
    if (traj.points.empty()) {
        RCLCPP_ERROR(logger, "Plan succeeded but trajectory is empty");
        return 1;
    }

    const auto & first = traj.points.front();
    const auto & last = traj.points.back();
    const auto & names = traj.joint_names;

    RCLCPP_INFO(logger, "Planning succeeded!");
    RCLCPP_INFO(logger, "Trajectory points: %zu", traj.points.size());
    RCLCPP_INFO(logger, "Planning computation time: %.3f s", plan.planning_time);
    RCLCPP_INFO(logger, "Trajectory duration: %.3f s",
                rclcpp::Duration(last.time_from_start).seconds());

    if (first.positions.size() == names.size() &&
        last.positions.size() == names.size()) {
        for (std::size_t i = 0; i < names.size(); ++i) {
            RCLCPP_INFO(logger, "%s: start=%.6f, goal=%.6f",
                        names[i].c_str(), first.positions[i], last.positions[i]);
        }
    }

    if (!first.velocities.empty() && !last.velocities.empty() &&
        !first.accelerations.empty() && !last.accelerations.empty()) {
        RCLCPP_INFO(logger, "Joint1 velocity: start=%.6f, end=%.6f rad/s",
                    first.velocities[0], last.velocities[0]);
        RCLCPP_INFO(logger, "Joint1 acceleration: start=%.6f, end=%.6f rad/s^2",
                    first.accelerations[0], last.accelerations[0]);
    }

    RCLCPP_INFO(logger, "Starting trajectory execution");
    auto exec_result = move_group.execute(plan);

    if (!exec_result) {
        RCLCPP_ERROR(
            logger,
            "Execution failed, error code: %d",
            exec_result.val
        );
        return 1;
    }
    RCLCPP_INFO(logger, "Execution succeeded!");

    return 0;
}

}  // namespace

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);

    rclcpp::NodeOptions options;
    options.automatically_declare_parameters_from_overrides(true);
    auto node = std::make_shared<rclcpp::Node>("minimal_moveit_demo", options);

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    std::thread executor_thread([&executor]() { executor.spin(); });

    int ret = 1;
    try {
        ret = run_demo(node);
    } catch (const std::exception & e) {
        RCLCPP_ERROR(node->get_logger(), "Demo failed: %s", e.what());
    }

    executor.cancel();
    executor_thread.join();
    rclcpp::shutdown();
    return ret;
}