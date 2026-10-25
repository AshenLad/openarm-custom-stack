#include<iostream>
#include<thread>
#include<chrono>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include <moveit/move_group_interface/move_group_interface.hpp>


int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);

    rclcpp::NodeOptions options;
    options.automatically_declare_parameters_from_overrides(true);

    auto node = std::make_shared<rclcpp::Node>(
        "minimal_moveit_demo", options
    );

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);

    std::thread executor_thread([&executor](){
        executor.spin();
    });

    {
        moveit::planning_interface::MoveGroupInterface move_group(
            node, "left_arm"
        );

        auto current_state = move_group.getCurrentState(2.0);

        if (current_state == nullptr) {
            RCLCPP_ERROR(
                node->get_logger(),
                "Failed to get current robot state"
            );
        } else {
            RCLCPP_INFO(
                node->get_logger(),
                "Current robot state received successfully"
            );
        }

        RCLCPP_INFO(
            node->get_logger(),
            "Planning Group: %s",
            move_group.getName().c_str()
        );
    }

    RCLCPP_INFO(node->get_logger(), "Main thread continues");
    std::this_thread::sleep_for(std::chrono::seconds(2));

    executor.cancel();
    executor_thread.join();

    rclcpp::shutdown();
    return 0;
}