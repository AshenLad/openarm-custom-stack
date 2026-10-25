#include <iostream>
#include <thread>
#include <chrono>
#include <memory>
#include <vector>

#include <moveit/robot_state/robot_state.hpp>
#include <moveit/robot_model/joint_model_group.hpp>
#include "rclcpp/rclcpp.hpp"
#include <moveit/move_group_interface/move_group_interface.hpp>

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);

    rclcpp::NodeOptions options;
    options.automatically_declare_parameters_from_overrides(true);

    auto node = std::make_shared<rclcpp::Node>("minimal_moveit_demo", options);

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);

    std::thread executor_thread([&executor]() {
        executor.spin();
    });

    {
        moveit::planning_interface::MoveGroupInterface move_group(node, "left_arm");

        auto current_state = move_group.getCurrentState(2.0);

        if (current_state == nullptr) {
            RCLCPP_ERROR(node->get_logger(), "Failed to get current robot state");
        } else {
            RCLCPP_INFO(node->get_logger(), "Current robot state received successfully");

            const auto * joint_group =
                current_state->getRobotModel()->getJointModelGroup("left_arm");

            if (joint_group == nullptr) {
                RCLCPP_ERROR(node->get_logger(), "left_arm group not found");
            } else {
                std::vector<double> positions;
                current_state->copyJointGroupPositions(joint_group, positions);

                if(positions.size() == 7){
                    move_group.setStartStateToCurrentState();
                    positions[0] += 0.1;

                    bool valid = move_group.setJointValueTarget(positions);

                    if(valid){
                        moveit::planning_interface::MoveGroupInterface::Plan plan;

                        auto result = move_group.plan(plan);

                        if(result == moveit::core::MoveItErrorCode::SUCCESS){
                            RCLCPP_INFO(
                                node->get_logger(),
                                "Planning succeeded!"
                            );
                            RCLCPP_INFO(
                                node->get_logger(),
                                "Trajectory points: %zu",
                                plan.trajectory.joint_trajectory.points.size()
                            );
                            RCLCPP_INFO(
                                node->get_logger(),
                                "Planning computation time: %.3f s",
                                plan.planning_time
                            );

                            const auto& trajectory = plan.trajectory.joint_trajectory;

                            if (!trajectory.points.empty()) {
                                const auto& last_point = trajectory.points.back();

                                const auto& duration = last_point.time_from_start;

                                double total_time =
                                    static_cast<double>(duration.sec) +
                                    static_cast<double>(duration.nanosec) * 1e-9;

                                RCLCPP_INFO(
                                    node->get_logger(),
                                    "Trajectory duration: %.3f s",
                                    total_time
                                );
                            }

                            const auto& first = trajectory.points.front();
                            const auto& last  = trajectory.points.back();
                            const auto& names = trajectory.joint_names;

                            for (std::size_t i = 0; i < names.size(); ++i){
                                if(first.positions.size() == names.size() &&
                                    last.positions.size() == names.size()){
                                    RCLCPP_INFO(
                                        node->get_logger(),
                                        "%s: start=%.6f, goal=%.6f",
                                        names[i].c_str(),
                                        first.positions[i],
                                        last.positions[i]
                                    );
                                }
                            }

                            if (!first.velocities.empty() &&
                                !last.velocities.empty() &&
                                !first.accelerations.empty() &&
                                !last.accelerations.empty()){
                                    RCLCPP_INFO(
                                        node->get_logger(),
                                        "Joint1 velocity: start=%.6f, end=%.6f rad/s",
                                        first.velocities[0],
                                        last.velocities[0]
                                    );

                                    RCLCPP_INFO(
                                        node->get_logger(),
                                        "Joint1 acceleration: start=%.6f, end=%.6f rad/s^2",
                                        first.accelerations[0],
                                        last.accelerations[0]
                                    );
                                }
                        }
                    }else{
                        RCLCPP_ERROR(
                            node->get_logger(),
                            "Planning failed!"
                        );
                    }

                    RCLCPP_INFO(
                        node->get_logger(),
                        "Target joint1: %.6f rad",
                        positions[0]
                    );

                    RCLCPP_INFO(
                        node->get_logger(),
                        "Joint target within bounds: %s",
                        valid ? "true" : "false"
                    );
                }else{
                    RCLCPP_ERROR(
                        node->get_logger(),
                        "Unexpected joint count: %zu",
                        positions.size()
                    );

            }
        }          

        RCLCPP_INFO(
            node->get_logger(),
            "Planning Group: %s",
            move_group.getName().c_str()
        );
    }   // move_group 作用域

    RCLCPP_INFO(node->get_logger(), "Main thread continues");
    std::this_thread::sleep_for(std::chrono::seconds(2));

    executor.cancel();
    executor_thread.join();

    rclcpp::shutdown();
    return 0;
}}