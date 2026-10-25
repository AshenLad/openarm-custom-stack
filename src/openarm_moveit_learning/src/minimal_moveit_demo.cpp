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

    const auto * joint_group =
        current_state->getRobotModel()->getJointModelGroup("left_arm");
    if (!joint_group) {
        RCLCPP_ERROR(logger, "left_arm group not found");
        return 1;
    }

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