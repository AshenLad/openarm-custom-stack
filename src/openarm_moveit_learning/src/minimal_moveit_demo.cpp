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

    {
        moveit::planning_interface::MoveGroupInterface move_group(
            node, "left_arm"
        );

        RCLCPP_INFO(
            node->get_logger(),
            "Planning Group: %s",
            move_group.getName().c_str()
        );
    }


    rclcpp::shutdown();
    return 0;
}