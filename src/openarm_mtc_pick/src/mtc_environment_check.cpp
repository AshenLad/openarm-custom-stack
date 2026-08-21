#include <memory>
#include <stdexcept>
#include <string>
#include <thread>

#include <moveit/task_constructor/stages/current_state.h>
#include <moveit/task_constructor/task.h>
#include <rclcpp/rclcpp.hpp>

namespace mtc = moveit::task_constructor;

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  options.automatically_declare_parameters_from_overrides(true);
  auto node = rclcpp::Node::make_shared("openarm_mtc_environment_check", options);
  std::thread spinner([node] { rclcpp::spin(node); });

  int result = 0;
  try {
    const std::string arm_group = node->get_parameter("robot.arm_group").as_string();
    const std::string gripper_group = node->get_parameter("robot.gripper_group").as_string();
    const std::string ee_frame = node->get_parameter("robot.ee_frame").as_string();
    const std::string gripper_joint = node->get_parameter("gripper.joint").as_string();

    mtc::Task task;
    task.stages()->setName("OpenArm MTC environment smoke test");
    task.loadRobotModel(node);
    const auto model = task.getRobotModel();

    const auto* arm = model->getJointModelGroup(arm_group);
    const auto* gripper = model->getJointModelGroup(gripper_group);
    if (!arm) {
      throw std::runtime_error("missing planning group: " + arm_group);
    }
    if (!gripper) {
      throw std::runtime_error("missing planning group: " + gripper_group);
    }
    if (!model->hasLinkModel(ee_frame)) {
      throw std::runtime_error("missing end-effector link: " + ee_frame);
    }
    if (!model->hasJointModel(gripper_joint)) {
      throw std::runtime_error("missing gripper joint: " + gripper_joint);
    }
    if (gripper->getLinkModelNamesWithCollisionGeometry().empty()) {
      throw std::runtime_error("gripper group has no collision geometry links");
    }

    RCLCPP_INFO(node->get_logger(),
                "Robot model OK: frame=%s arm=%s (%u variables), gripper=%s (%u "
                "variables), ee=%s, joint=%s",
                model->getModelFrame().c_str(), arm_group.c_str(),
                arm->getVariableCount(), gripper_group.c_str(),
                gripper->getVariableCount(), ee_frame.c_str(), gripper_joint.c_str());

    task.add(std::make_unique<mtc::stages::CurrentState>("current state"));
    task.init();
    if (!task.plan(1)) {
      throw std::runtime_error(
          "CurrentState produced no solution; check move_group/get_planning_scene and joint_states");
    }
    task.introspection().publishSolution(*task.solutions().front());
    RCLCPP_INFO(node->get_logger(),
                "MTC plan-only smoke test passed; no trajectory was executed");
  } catch (const std::exception& error) {
    RCLCPP_ERROR(node->get_logger(), "Environment check failed: %s", error.what());
    result = 1;
  }

  rclcpp::shutdown();
  spinner.join();
  return result;
}
