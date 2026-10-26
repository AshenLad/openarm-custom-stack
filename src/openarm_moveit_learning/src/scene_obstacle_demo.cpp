
#include "rclcpp/rclcpp.hpp"
#include <moveit/planning_scene_interface/planning_scene_interface.hpp>
#include <moveit_msgs/msg/collision_object.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <geometry_msgs/msg/pose.hpp>

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    auto logger = rclcpp::get_logger("scene_obstacle_demo");

    moveit::planning_interface::PlanningSceneInterface scene;

    moveit_msgs::msg::CollisionObject box;
    box.id = "stage4_demo_box";
    box.header.frame_id = "world";

    shape_msgs::msg::SolidPrimitive shape;
    shape.type = shape_msgs::msg::SolidPrimitive::BOX;
    shape.dimensions = {0.1, 0.1, 0.1};

    geometry_msgs::msg::Pose pose;
    pose.orientation.w = 1.0;
    pose.position.x = 0.35;
    pose.position.y = 0.0;
    pose.position.z = 0.25;

    box.primitives.push_back(shape);
    box.primitive_poses.push_back(pose);
    box.operation = moveit_msgs::msg::CollisionObject::ADD;

    bool success = scene.applyCollisionObject(box);

    if (success) {
        RCLCPP_INFO(logger, "Collision box added successfully");
    } else {
        RCLCPP_ERROR(logger, "Failed to add collision box");
    }

    rclcpp::shutdown();
    return success ? 0 : 1;
}
