#include <algorithm>
#include <array>
#include <atomic>
#include <cmath>
#include <chrono>
#include <condition_variable>
#include <iomanip>
#include <limits>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include <Eigen/Geometry>
#include <control_msgs/msg/joint_trajectory_controller_state.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.hpp>
#include <moveit/robot_state/robot_state.hpp>
#include <moveit/task_constructor/container.h>
#include <moveit/task_constructor/cost_terms.h>
#include <moveit/task_constructor/solvers/cartesian_path.h>
#include <moveit/task_constructor/solvers/joint_interpolation.h>
#include <moveit/task_constructor/solvers/pipeline_planner.h>
#include <moveit/task_constructor/stages/compute_ik.h>
#include <moveit/task_constructor/stages/connect.h>
#include <moveit/task_constructor/stages/current_state.h>
#include <moveit/task_constructor/stages/generate_grasp_pose.h>
#include <moveit/task_constructor/stages/generate_place_pose.h>
#include <moveit/task_constructor/stages/modify_planning_scene.h>
#include <moveit/task_constructor/stages/move_relative.h>
#include <moveit/task_constructor/stages/move_to.h>
#include <moveit/task_constructor/task.h>
#include <moveit_task_constructor_msgs/action/execute_task_solution.hpp>
#include <moveit_task_constructor_msgs/msg/solution.hpp>
#include <moveit_msgs/msg/collision_object.hpp>
#include <moveit_msgs/msg/move_it_error_codes.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>

#include "run_mode.hpp"
#include "tcp_geometry.hpp"

namespace mtc = moveit::task_constructor;
namespace {

moveit_msgs::msg::MoveItErrorCodes execute_sanitized_solution(
    mtc::Task& task, const mtc::SolutionBase& solution,
    const rclcpp::Logger& logger) {
  using ExecuteTaskSolution =
      moveit_task_constructor_msgs::action::ExecuteTaskSolution;

  ExecuteTaskSolution::Goal goal;
  solution.toMsg(goal.solution, &task.introspection());

  // Unique per call so repeated executions in one process can never collide
  // with a previously registered node name (FastDDS "Publisher already
  // registered for node name" warning).
  static std::atomic<std::size_t> executor_instance{0};
  auto action_node = rclcpp::Node::make_shared(
      "openarm_mtc_sanitized_solution_executor" +
      std::to_string(executor_instance.fetch_add(1)));

  std::size_t sanitized_boundaries = 0;
  double largest_terminal_velocity = 0.0;
  double largest_planned_velocity = 0.0;
  double largest_planned_acceleration = 0.0;
  double total_duration = 0.0;
  std::vector<std::size_t> waypoints_per_segment;
  std::vector<std::string> controller_names;
  for (auto& sub_trajectory : goal.solution.sub_trajectory) {
    auto& trajectory = sub_trajectory.trajectory.joint_trajectory;
    if (trajectory.points.empty()) {
      continue;
    }
    for (const auto& controller : sub_trajectory.execution_info.controller_names) {
      if (std::find(controller_names.begin(), controller_names.end(),
                    controller) == controller_names.end()) {
        controller_names.push_back(controller);
      }
    }
    const auto& last_point = trajectory.points.back();
    const double segment_duration =
        static_cast<double>(last_point.time_from_start.sec) +
        1.0e-9 * static_cast<double>(last_point.time_from_start.nanosec);
    total_duration += std::max(0.0, segment_duration);
    waypoints_per_segment.push_back(trajectory.points.size());
    for (const auto& point : trajectory.points) {
      for (const double velocity : point.velocities) {
        largest_planned_velocity =
            std::max(largest_planned_velocity, std::abs(velocity));
      }
      for (const double acceleration : point.accelerations) {
        largest_planned_acceleration =
            std::max(largest_planned_acceleration, std::abs(acceleration));
      }
    }
    for (const double velocity : last_point.velocities) {
      largest_terminal_velocity =
          std::max(largest_terminal_velocity, std::abs(velocity));
    }
    const auto zero_boundary_derivatives = [&trajectory](auto& point) {
      const std::size_t joint_count = trajectory.joint_names.size();
      if (!point.velocities.empty()) {
        point.velocities.assign(joint_count, 0.0);
      }
      if (!point.accelerations.empty()) {
        point.accelerations.assign(joint_count, 0.0);
      }
    };
    zero_boundary_derivatives(trajectory.points.front());
    if (trajectory.points.size() > 1) {
      zero_boundary_derivatives(trajectory.points.back());
    }
    ++sanitized_boundaries;
  }
  RCLCPP_INFO(logger,
              "Execution trajectory boundary sanitization: segments=%zu, "
              "largest original terminal velocity=%.9g rad/s",
              sanitized_boundaries, largest_terminal_velocity);
  {
    std::ostringstream summary;
    summary << "EXECUTION_TRAJECTORY_SUMMARY\n";
    summary << "  segments=" << sanitized_boundaries << "\n";
    summary << "  total_duration=" << std::fixed << std::setprecision(3)
            << total_duration << " s\n";
    for (std::size_t segment = 0; segment < waypoints_per_segment.size();
         ++segment) {
      summary << "  segment[" << segment << "].waypoints="
              << waypoints_per_segment[segment] << "\n";
    }
    summary << "  max_planned_joint_velocity=" << std::fixed
            << std::setprecision(6) << largest_planned_velocity << " rad/s\n";
    summary << "  max_planned_joint_acceleration=" << std::fixed
            << std::setprecision(6) << largest_planned_acceleration
            << " rad/s^2\n";
    summary << "  largest_terminal_velocity=" << std::fixed
            << std::setprecision(6) << largest_terminal_velocity << " rad/s\n";
    summary << "  controllers=[";
    for (std::size_t index = 0; index < controller_names.size(); ++index) {
      if (index > 0) {
        summary << ", ";
      }
      summary << controller_names[index];
    }
    summary << "]";
    RCLCPP_INFO(logger, "%s", summary.str().c_str());
  }

  moveit_msgs::msg::MoveItErrorCodes failure;
  failure.val = moveit_msgs::msg::MoveItErrorCodes::FAILURE;
  auto action_client = rclcpp_action::create_client<ExecuteTaskSolution>(
      action_node, "execute_task_solution");
  if (!action_client->wait_for_action_server(std::chrono::seconds(5))) {
    RCLCPP_ERROR(logger,
                 "Failed to connect to the execute_task_solution action server");
    return failure;
  }

  auto goal_future = action_client->async_send_goal(goal);
  if (rclcpp::spin_until_future_complete(action_node, goal_future) !=
      rclcpp::FutureReturnCode::SUCCESS) {
    RCLCPP_ERROR(logger, "Sending sanitized execution goal failed");
    return failure;
  }
  const auto goal_handle = goal_future.get();
  if (!goal_handle) {
    RCLCPP_ERROR(logger, "Sanitized execution goal was rejected by server");
    return failure;
  }

  auto result_future = action_client->async_get_result(goal_handle);
  if (rclcpp::spin_until_future_complete(action_node, result_future) !=
      rclcpp::FutureReturnCode::SUCCESS) {
    RCLCPP_ERROR(logger, "Waiting for sanitized execution result failed");
    return failure;
  }
  const auto wrapped_result = result_future.get();
  if (wrapped_result.code != rclcpp_action::ResultCode::SUCCEEDED ||
      !wrapped_result.result) {
    RCLCPP_ERROR(logger, "Sanitized execution goal was aborted or canceled");
    return failure;
  }
  return wrapped_result.result->error_code;
}

template <typename T>
T parameter_or(const rclcpp::Node::SharedPtr& node, const std::string& name,
               const T& fallback) {
  if (node->has_parameter(name)) {
    return node->get_parameter(name).get_value<T>();
  }
  return node->declare_parameter<T>(name, fallback);
}

void require_size(const std::string& name, const std::vector<double>& value,
                  std::size_t expected) {
  if (value.size() != expected) {
    throw std::invalid_argument(name + " must contain " +
                                std::to_string(expected) + " values");
  }
}

geometry_msgs::msg::Quaternion quaternion_from_rpy(const double roll,
                                                   const double pitch,
                                                   const double yaw) {
  const Eigen::Quaterniond quaternion(
      Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) *
      Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
      Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX()));
  geometry_msgs::msg::Quaternion message;
  message.x = quaternion.x();
  message.y = quaternion.y();
  message.z = quaternion.z();
  message.w = quaternion.w();
  return message;
}

double yaw_from_pose(const geometry_msgs::msg::Pose& pose) {
  const auto& q = pose.orientation;
  return std::atan2(2.0 * (q.w * q.z + q.x * q.y),
                    1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}

double angle_distance(const double first, const double second) {
  return std::abs(std::atan2(std::sin(first - second),
                            std::cos(first - second)));
}

double symmetric_angle_distance(const double first, const double second,
                                const int symmetry_order) {
  const double scaled =
      static_cast<double>(symmetry_order) * (first - second);
  return std::abs(std::atan2(std::sin(scaled), std::cos(scaled))) /
         static_cast<double>(symmetry_order);
}

geometry_msgs::msg::Pose compose_poses(const geometry_msgs::msg::Pose& parent,
                                       const geometry_msgs::msg::Pose& child) {
  const Eigen::Quaterniond parent_rotation(
      parent.orientation.w, parent.orientation.x, parent.orientation.y,
      parent.orientation.z);
  const Eigen::Quaterniond child_rotation(
      child.orientation.w, child.orientation.x, child.orientation.y,
      child.orientation.z);
  const Eigen::Vector3d parent_translation(
      parent.position.x, parent.position.y, parent.position.z);
  const Eigen::Vector3d child_translation(
      child.position.x, child.position.y, child.position.z);
  const Eigen::Vector3d translation =
      parent_translation + parent_rotation.normalized() * child_translation;
  const Eigen::Quaterniond rotation =
      (parent_rotation.normalized() * child_rotation.normalized()).normalized();
  geometry_msgs::msg::Pose result;
  result.position.x = translation.x();
  result.position.y = translation.y();
  result.position.z = translation.z();
  result.orientation.x = rotation.x();
  result.orientation.y = rotation.y();
  result.orientation.z = rotation.z();
  result.orientation.w = rotation.w();
  return result;
}

Eigen::Isometry3d pose_to_eigen(const geometry_msgs::msg::Pose& pose) {
  Eigen::Isometry3d result = Eigen::Isometry3d::Identity();
  result.translation() = Eigen::Vector3d(
      pose.position.x, pose.position.y, pose.position.z);
  result.linear() = Eigen::Quaterniond(
                        pose.orientation.w, pose.orientation.x,
                        pose.orientation.y, pose.orientation.z)
                        .normalized()
                        .toRotationMatrix();
  return result;
}

// Try to find a raw IK solution for the world->TCP target. Returns the joint
// positions of the first successful seed attempt, or nullopt. This is the
// shared primitive behind both the string diagnostic and the boolean
// pre-filter, so the printed YES/NO and the candidate filtering can never
// disagree about a candidate.
std::optional<std::vector<double>> try_raw_ik(
    const moveit::core::RobotState& seed,
    const moveit::core::JointModelGroup* group,
    const Eigen::Isometry3d& world_from_tcp,
    const Eigen::Isometry3d& ee_to_tcp,
    const std::string& ee_frame,
    const std::size_t attempts = 16) {
  const Eigen::Isometry3d world_from_ee = world_from_tcp * ee_to_tcp.inverse();
  for (std::size_t attempt = 0; attempt < attempts; ++attempt) {
    moveit::core::RobotState candidate(seed);
    if (attempt > 0) {
      candidate.setToRandomPositions(group);
    }
    if (!candidate.setFromIK(group, world_from_ee, ee_frame, 0.02)) {
      continue;
    }
    std::vector<double> positions;
    candidate.copyJointGroupPositions(group, positions);
    return positions;
  }
  return std::nullopt;
}

bool raw_ik_available(const moveit::core::RobotState& seed,
                      const moveit::core::JointModelGroup* group,
                      const Eigen::Isometry3d& world_from_tcp,
                      const Eigen::Isometry3d& ee_to_tcp,
                      const std::string& ee_frame,
                      const std::size_t attempts = 16) {
  return try_raw_ik(seed, group, world_from_tcp, ee_to_tcp, ee_frame, attempts)
      .has_value();
}

std::string format_raw_ik_result(
    const std::optional<std::vector<double>>& positions) {
  if (!positions.has_value()) {
    return "NO";
  }
  std::ostringstream stream;
  stream.setf(std::ios::fixed);
  stream.precision(3);
  stream << "YES joints=[";
  for (std::size_t index = 0; index < positions->size(); ++index) {
    if (index > 0) {
      stream << ",";
    }
    stream << (*positions)[index];
  }
  stream << "]";
  return stream.str();
}

std::string raw_ik_diagnostic(
    const moveit::core::RobotState& seed,
    const moveit::core::JointModelGroup* group,
    const Eigen::Isometry3d& world_from_tcp,
    const Eigen::Isometry3d& ee_to_tcp,
    const std::string& ee_frame,
    const std::size_t attempts = 16) {
  return format_raw_ik_result(
      try_raw_ik(seed, group, world_from_tcp, ee_to_tcp, ee_frame, attempts));
}

bool has_joint_limit_margin(
    const mtc::SolutionBase& solution,
    const moveit::core::RobotModel& model,
    const std::vector<std::string>& checked_joints, const double margin,
    std::string& reason) {
  moveit_task_constructor_msgs::msg::Solution message;
  solution.toMsg(message);
  const auto& model_variables = model.getVariableNames();
  for (const auto& checked_joint : checked_joints) {
    if (std::find(model_variables.begin(), model_variables.end(), checked_joint) ==
        model_variables.end()) {
      reason = "unknown margin joint '" + checked_joint + "'";
      return false;
    }
    const auto& bounds = model.getVariableBounds(checked_joint);
    if (!bounds.position_bounded_) {
      continue;
    }
    for (const auto& subtrajectory : message.sub_trajectory) {
      const auto& trajectory = subtrajectory.trajectory.joint_trajectory;
      const auto found = std::find(trajectory.joint_names.begin(),
                                   trajectory.joint_names.end(), checked_joint);
      if (found == trajectory.joint_names.end()) {
        continue;
      }
      const auto index = static_cast<std::size_t>(
          std::distance(trajectory.joint_names.begin(), found));
      bool first_point = true;
      for (const auto& point : trajectory.points) {
        if (first_point) {
          // The first point of each segment is the physical current state (or
          // the previous segment's end), which the arm has already reached
          // safely; it is not a planned approach toward the limit.
          first_point = false;
          continue;
        }
        if (index >= point.positions.size()) {
          reason = "trajectory point is missing '" + checked_joint + "'";
          return false;
        }
        const double position = point.positions[index];
        if (position < bounds.min_position_ + margin ||
            position > bounds.max_position_ - margin) {
          reason = checked_joint + " reaches " + std::to_string(position) +
                   " rad, inside the configured " + std::to_string(margin) +
                   " rad limit margin";
          return false;
        }
      }
    }
  }
  return true;
}

// Per-yaw-candidate raw-IK verdict, collected while the IK_DIAG block runs and
// consumed by GRASP CANDIDATE SUMMARY and (for pregrasp_test) the pre-filter.
// Kept in one place so the printed status always matches what was filtered.
struct CandidateDiagnostic {
  std::size_t index;
  double yaw_offset_rad;
  double world_yaw_rad;
  bool raw_pregrasp_ik;
  bool raw_approach_ik;
  bool raw_cube_center_ik;
};

std::string candidate_status(const CandidateDiagnostic& candidate,
                             const bool uses_pick) {
  if (!candidate.raw_pregrasp_ik) {
    return "REJECTED_RAW_IK";
  }
  if (uses_pick && !candidate.raw_approach_ik) {
    return "REJECTED_APPROACH_IK";
  }
  return "ACCEPTED_RAW_IK";
}

void print_grasp_candidate_summary(const std::vector<CandidateDiagnostic>& candidates,
                                   const bool uses_pick,
                                   const rclcpp::Logger& logger) {
  std::ostringstream stream;
  stream << "====================================\n";
  stream << "GRASP CANDIDATE SUMMARY\n";
  stream << "====================================\n";
  std::size_t accepted = 0;
  std::size_t rejected = 0;
  for (const auto& candidate : candidates) {
    const std::string status = candidate_status(candidate, uses_pick);
    if (status == "ACCEPTED_RAW_IK") {
      ++accepted;
    } else {
      ++rejected;
    }
    stream << "candidate " << candidate.index << "\n";
    stream << "  yaw_offset=" << candidate.yaw_offset_rad * 180.0 / M_PI
           << " deg, world_yaw=" << candidate.world_yaw_rad * 180.0 / M_PI
           << " deg\n";
    stream << "  raw pregrasp IK = "
           << (candidate.raw_pregrasp_ik ? "YES" : "NO") << "\n";
    if (uses_pick) {
      stream << "  raw approach IK = "
             << (candidate.raw_approach_ik ? "YES" : "NO") << "\n";
    }
    stream << "  raw cube-center IK = "
           << (candidate.raw_cube_center_ik ? "YES" : "NO") << "\n";
    stream << "  status = " << status << "\n";
  }
  stream << "accepted (raw IK): " << accepted << "\n";
  stream << "rejected (raw IK): " << rejected << "\n";
  stream << "(raw IK only; MTC/planning additionally filters by collision, "
            "joint limits and solver success)\n";
  stream << "NOTE: individual candidate failures are CANDIDATE_REJECTED; "
            "planning that still\n";
  stream << "      produces no solution is TASK_FAILED (see MTC STAGE SUMMARY "
            "for the first\n";
  stream << "      zero-solution stage that dropped the whole branch)\n";
  RCLCPP_INFO(logger, "%s", stream.str().c_str());
}

// MTC_STAGE_SUMMARY: one line per stage with its final solution count, plus
// the first stage that dropped to zero solutions. This is what separates a
// normal candidate rejection (some stages keep solutions) from a real task
// failure (the first stage whose input branch never produced an output).
void print_mtc_stage_summary(const mtc::Task& task, const rclcpp::Logger& logger) {
  std::ostringstream stream;
  stream << "=====================================\n";
  stream << "MTC STAGE SUMMARY\n";
  stream << "=====================================\n";
  std::string first_zero_solution_stage;
  task.stages()->traverseRecursively(
      [&](const mtc::Stage& stage, const unsigned int depth) {
        const std::size_t solutions = stage.solutions().size();
        const std::size_t failures = stage.numFailures();
        stream << std::string(depth * 2, ' ') << stage.name()
               << ": solutions=" << solutions << " failures=" << failures
               << "\n";
        if (first_zero_solution_stage.empty() && solutions == 0 &&
            failures > 0) {
          first_zero_solution_stage = stage.name();
        }
        return true;
      });
  stream << "Final: solutions = " << task.numSolutions() << "\n";
  if (task.numSolutions() > 0) {
    // Zero-solution child stages can be ordinary rejected candidate branches
    // when another branch completes the task. Calling one of them the "first
    // failed stage" after an overall success is actively misleading.
    first_zero_solution_stage.clear();
  } else if (first_zero_solution_stage.empty()) {
    first_zero_solution_stage = "unknown (no failures recorded)";
  }
  stream << "FIRST FAILED STAGE: "
         << (first_zero_solution_stage.empty() ? "(none)" : first_zero_solution_stage)
         << "\n";
  RCLCPP_INFO(logger, "%s", stream.str().c_str());
}

// Optional diagnostics.controller_tracking report. Reads the latest
// /<controller>/controller_state after execution and prints the per-joint
// reference-feedback error plus the worst joint. Purely diagnostic: a nonzero
// residual here is expected and must not be "fixed" with a vision offset.
void print_controller_tracking(const std::string& topic,
                               const rclcpp::Logger& logger) {
  static std::atomic<std::size_t> tracking_instance{0};
  auto tracking_node = rclcpp::Node::make_shared(
      "openarm_mtc_controller_tracking" +
      std::to_string(tracking_instance.fetch_add(1)));
  std::optional<control_msgs::msg::JointTrajectoryControllerState> latest;
  auto subscription =
      tracking_node->create_subscription<control_msgs::msg::JointTrajectoryControllerState>(
          topic, 10,
          [&latest](const control_msgs::msg::JointTrajectoryControllerState::SharedPtr message) {
            latest = *message;
          });
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(tracking_node);
  const auto deadline =
      std::chrono::steady_clock::now() + std::chrono::milliseconds(800);
  while (rclcpp::ok() && !latest.has_value() &&
         std::chrono::steady_clock::now() < deadline) {
    executor.spin_some();
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }
  executor.remove_node(tracking_node);
  if (!latest.has_value()) {
    RCLCPP_WARN(logger,
                "CONTROLLER_TRACKING: no state received on %s within 0.8 s; "
                "check that the joint trajectory controller is running",
                topic.c_str());
    return;
  }
  double max_abs_error = 0.0;
  std::string max_joint;
  std::ostringstream stream;
  stream << "CONTROLLER_TRACKING (" << topic << "): per-joint final error\n";
  for (std::size_t index = 0; index < latest->joint_names.size(); ++index) {
    const double error =
        index < latest->error.positions.size() ? latest->error.positions[index] : 0.0;
    const double abs_error = std::abs(error);
    stream << "  " << latest->joint_names[index] << ": error="
           << std::fixed << std::setprecision(6) << error << " rad\n";
    if (abs_error > max_abs_error) {
      max_abs_error = abs_error;
      max_joint = latest->joint_names[index];
    }
  }
  stream << "max abs joint error = " << std::fixed << std::setprecision(6)
         << max_abs_error << " rad (" << max_joint << ")";
  RCLCPP_INFO(logger, "%s", stream.str().c_str());
}

}  // namespace

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  options.automatically_declare_parameters_from_overrides(true);
  auto node = rclcpp::Node::make_shared("openarm_mtc_pregrasp", options);
  std::thread spinner([node] { rclcpp::spin(node); });

  int result = 0;
  try {
    const std::string arm_group =
        parameter_or<std::string>(node, "robot.arm_group", "right_arm");
    const std::string gripper_group =
        parameter_or<std::string>(node, "robot.gripper_group", "right_gripper");
    const std::string ee_frame = parameter_or<std::string>(
        node, "robot.ee_frame", "openarm_right_ee_base_link");
    const std::string eef_name =
        parameter_or<std::string>(node, "robot.eef", "right_ee");
    const std::string joint_states_topic =
        parameter_or<std::string>(node, "robot.joint_states_topic", "/joint_states");
    const double joint_state_wait_timeout =
        parameter_or<double>(node, "robot.joint_state_wait_timeout", 5.0);
    const std::string gripper_joint = parameter_or<std::string>(
        node, "gripper.joint", "openarm_right_finger_joint1");
    const double open_position =
        parameter_or<double>(node, "gripper.open_position", -0.7854);
    const double post_release_position =
        parameter_or<double>(node, "gripper.post_release_position", 0.0);
    const double reference_joint_position = parameter_or<double>(
        node, "gripper.reference_joint_position", -0.10);
    const double reference_object_width = parameter_or<double>(
        node, "gripper.reference_object_width", 0.040);
    const double aperture_preload =
        parameter_or<double>(node, "gripper.aperture_preload", 0.002);
    const double finger_contact_radius = parameter_or<double>(
        node, "gripper.finger_contact_radius", 0.070);
    const double tip_below_grasp_frame = parameter_or<double>(
        node, "gripper.tip_below_grasp_frame", 0.032);
    const double support_clearance = parameter_or<double>(
        node, "gripper.support_clearance", 0.003);
    const auto gripper_touch_links = parameter_or<std::vector<std::string>>(
        node, "gripper.touch_links",
        {"openarm_right_ee_link1", "openarm_right_ee_link2"});

    const std::string planning_frame =
        parameter_or<std::string>(node, "robot.planning_frame", "world");
    const std::string object_id =
        parameter_or<std::string>(node, "object.id", "blue_cube");
    const std::string table_id =
        parameter_or<std::string>(node, "table.id", "table");
    const std::string object_pose_topic = parameter_or<std::string>(
        node, "object.pose_topic", "/blue_cube/pose");
    const std::string object_dimensions_topic = parameter_or<std::string>(
        node, "object.dimensions_topic", "/blue_cube/dimensions");
    const double pose_wait_timeout =
        parameter_or<double>(node, "object.pose_wait_timeout", 0.0);
    const double object_max_message_time_delta = parameter_or<double>(
        node, "object.max_message_time_delta", 0.20);
    const double object_max_pose_age =
        parameter_or<double>(node, "object.max_pose_age", 1.0);
    const auto object_max_motion_xyz = parameter_or<std::vector<double>>(
        node, "object.max_motion_xyz", {0.01, 0.01, 0.01});
    const auto object_max_size_change = parameter_or<std::vector<double>>(
        node, "object.max_size_change", {0.01, 0.01, 0.01});
    const double object_max_motion_yaw =
        parameter_or<double>(node, "object.max_motion_yaw", 0.20);
    const double object_max_support_height_error = parameter_or<double>(
        node, "object.max_support_height_error", 0.015);
    const int configured_object_yaw_symmetry_order =
        parameter_or<int>(node, "object.yaw_symmetry_order", 4);
    const double square_aspect_ratio_max =
        parameter_or<double>(node, "object.square_aspect_ratio_max", 1.30);
    const double pregrasp_offset =
        parameter_or<double>(node, "motion.pregrasp_offset", 0.05);
    const double approach_min_distance = parameter_or<double>(
        node, "motion.approach_min_distance", pregrasp_offset);
    const double grasp_roll =
        parameter_or<double>(node, "motion.grasp_roll", 0.0);
    const double grasp_pitch =
        parameter_or<double>(node, "motion.grasp_pitch", 0.0);
    const std::string grasp_yaw_reference = parameter_or<std::string>(
        node, "motion.grasp_yaw_reference", "object");
    const auto grasp_yaw_refinements = parameter_or<std::vector<double>>(
        node, "motion.grasp_yaw_refinements", {0.0});
    const double max_vertical_tilt = parameter_or<double>(
        node, "motion.max_vertical_tilt_rad", 0.05);
    const auto diagnostic_pitch_offsets = parameter_or<std::vector<double>>(
        node, "diagnostics.pitch_offsets_rad", {0.0});
    const bool enable_tilt_scan = parameter_or<bool>(
        node, "diagnostics.enable_tilt_scan", false);
    const bool controller_tracking = parameter_or<bool>(
        node, "diagnostics.controller_tracking", false);
    const std::string controller_state_topic = parameter_or<std::string>(
        node, "diagnostics.controller_state_topic",
        "/right_joint_trajectory_controller/controller_state");
    const double velocity_scaling =
        parameter_or<double>(node, "motion.velocity_scaling", 0.10);
    const double acceleration_scaling =
        parameter_or<double>(node, "motion.acceleration_scaling", 0.10);
    const double planning_timeout =
        parameter_or<double>(node, "motion.planning_timeout", 10.0);
    const int max_solutions =
        parameter_or<int>(node, "motion.max_solutions", 5);
    const double joint_limit_margin =
        parameter_or<double>(node, "motion.joint_limit_margin", 0.02);
    const auto margin_joint_names = parameter_or<std::vector<std::string>>(
        node, "motion.margin_joint_names",
        {"openarm_right_joint6", "openarm_right_joint7"});
    const double lift_min_distance =
        parameter_or<double>(node, "motion.lift_min_distance", 0.05);
    const double lift_max_distance =
        parameter_or<double>(node, "motion.lift_max_distance", 0.08);
    const double cartesian_step =
        parameter_or<double>(node, "motion.cartesian_step", 0.005);
    const double cartesian_min_fraction =
        parameter_or<double>(node, "motion.cartesian_min_fraction", 0.95);
    const auto place_local_xy_fractions_flat =
        parameter_or<std::vector<double>>(
            node, "place.local_xy_fractions", {0.0, 0.0});
    const auto place_yaw_candidates = parameter_or<std::vector<double>>(
        node, "place.yaw_candidates", {0.0});
    const std::string place_yaw_reference = parameter_or<std::string>(
        node, "place.yaw_reference", "table");
    const double preplace_offset =
        parameter_or<double>(node, "place.preplace_offset", 0.07);
    // Likewise, lower from the pre-place offset to the dynamically computed
    // landing center instead of stopping at a fixed residual height.
    const double lower_distance = preplace_offset;
    const double retreat_distance =
        parameter_or<double>(node, "place.retreat_distance", 0.07);
    const double place_edge_clearance =
        parameter_or<double>(node, "place.edge_clearance", 0.01);
    const double place_surface_clearance =
        parameter_or<double>(node, "place.surface_clearance", 0.005);
    const double table_collision_padding_top =
        parameter_or<double>(node, "table.collision_padding_top", 0.0);
    const double table_collision_padding_xy =
        parameter_or<double>(node, "table.collision_padding_xy", 0.0);
    const bool table_recheck_before_execute = parameter_or<bool>(
        node, "table.recheck_before_execute", true);
    const std::string table_pose_topic = parameter_or<std::string>(
        node, "table.pose_topic", "/box/pose");
    const std::string table_dimensions_topic = parameter_or<std::string>(
        node, "table.dimensions_topic", "/box/dimensions");
    const double table_max_pose_age =
        parameter_or<double>(node, "table.max_pose_age", 1.0);
    const auto table_max_motion_xyz = parameter_or<std::vector<double>>(
        node, "table.max_motion_xyz", {0.01, 0.01, 0.01});
    const auto table_max_size_change = parameter_or<std::vector<double>>(
        node, "table.max_size_change", {0.01, 0.01, 0.01});
    const double table_max_message_time_delta = parameter_or<double>(
        node, "table.max_message_time_delta", 0.20);
    const double table_max_motion_yaw =
        parameter_or<double>(node, "table.max_motion_yaw", 0.10);

    const std::string tcp_frame = parameter_or<std::string>(
        node, "tcp.frame", "openarm_right_grasp_frame");

    const std::string run_mode_string =
        parameter_or<std::string>(node, "task.run_mode", "plan_only");
    const openarm_mtc::RunMode run_mode =
        openarm_mtc::parse_run_mode(run_mode_string);
    const bool production_demo =
        parameter_or<bool>(node, "task.production_demo", false);
    const bool execute = parameter_or<bool>(node, "execution.execute", false);
    const bool allow_execution =
        parameter_or<bool>(node, "execution.allow_execution", false);
    const bool allow_full_execution = parameter_or<bool>(
        node, "execution.allow_full_execution", false);
    const bool use_receipt_time_for_freshness = parameter_or<bool>(
        node, "execution.use_receipt_time_for_freshness", false);
    const double visual_recheck_wait_timeout = parameter_or<double>(
        node, "execution.visual_recheck_wait_timeout", 2.0);

    require_size("object.max_motion_xyz", object_max_motion_xyz, 3);
    require_size("object.max_size_change", object_max_size_change, 3);
    require_size("table.max_motion_xyz", table_max_motion_xyz, 3);
    require_size("table.max_size_change", table_max_size_change, 3);
    if (place_local_xy_fractions_flat.empty() ||
        place_local_xy_fractions_flat.size() % 2 != 0) {
      throw std::invalid_argument(
          "place.local_xy_fractions must contain one or more XY pairs");
    }
    std::vector<std::array<double, 2>> place_local_xy_fractions;
    for (std::size_t index = 0;
         index < place_local_xy_fractions_flat.size(); index += 2) {
      const std::array<double, 2> fraction{
          place_local_xy_fractions_flat[index],
          place_local_xy_fractions_flat[index + 1]};
      if (!std::isfinite(fraction[0]) || !std::isfinite(fraction[1]) ||
          std::abs(fraction[0]) > 1.0 || std::abs(fraction[1]) > 1.0) {
        throw std::invalid_argument(
            "place.local_xy_fractions values must be finite in [-1, 1]");
      }
      place_local_xy_fractions.push_back(fraction);
    }
    if (place_yaw_candidates.empty()) {
      throw std::invalid_argument("place yaw candidate list must not be empty");
    }
    if (diagnostic_pitch_offsets.empty() ||
        std::any_of(diagnostic_pitch_offsets.begin(),
                    diagnostic_pitch_offsets.end(),
                    [](const double value) { return !std::isfinite(value); })) {
      throw std::invalid_argument(
          "diagnostics.pitch_offsets_rad must contain finite values");
    }
    if (grasp_yaw_refinements.empty() ||
        std::any_of(grasp_yaw_refinements.begin(),
                    grasp_yaw_refinements.end(), [](const double value) {
                      return !std::isfinite(value) || std::abs(value) > 0.10;
                    })) {
      throw std::invalid_argument(
          "motion.grasp_yaw_refinements must contain finite values within "
          "+/-0.10 rad");
    }
    if (pregrasp_offset <= 0.0 || approach_min_distance <= 0.0 ||
        approach_min_distance > pregrasp_offset) {
      throw std::invalid_argument(
          "motion approach distances must satisfy 0 < min <= pregrasp_offset");
    }
    if (!std::isfinite(pose_wait_timeout) || pose_wait_timeout < 0.0) {
      throw std::invalid_argument(
          "object.pose_wait_timeout must be finite and non-negative; 0 waits "
          "until synchronized visual data arrives");
    }
    if (lift_min_distance <= 0.0 || lift_max_distance < lift_min_distance ||
        cartesian_step <= 0.0 || cartesian_min_fraction <= 0.0 ||
        cartesian_min_fraction > 1.0) {
      throw std::invalid_argument("Cartesian approach/lift parameters are invalid");
    }
    if (preplace_offset <= 0.0 || lower_distance <= 0.0 ||
        retreat_distance <= 0.0 || place_surface_clearance < 0.0 ||
        !std::isfinite(aperture_preload) || aperture_preload < 0.0 ||
        !std::isfinite(post_release_position) ||
        place_edge_clearance < 0.0 || table_collision_padding_top < 0.0 ||
        table_collision_padding_xy < 0.0 ||
        joint_state_wait_timeout <= 0.0 || object_max_pose_age <= 0.0 ||
        table_max_pose_age <= 0.0 ||
        !std::isfinite(visual_recheck_wait_timeout) ||
        visual_recheck_wait_timeout <= 0.0 ||
        table_max_motion_yaw < 0.0 || object_max_motion_yaw < 0.0 ||
        !std::isfinite(object_max_support_height_error) ||
        object_max_support_height_error < 0.0 ||
        object_max_message_time_delta <= 0.0 ||
        table_max_message_time_delta <= 0.0 ||
        !std::isfinite(reference_joint_position) ||
        !std::isfinite(reference_object_width) || reference_object_width <= 0.0 ||
        !std::isfinite(finger_contact_radius) || finger_contact_radius <= 0.0 ||
        !std::isfinite(tip_below_grasp_frame) ||
        !std::isfinite(support_clearance) || tip_below_grasp_frame < 0.0 ||
        support_clearance < 0.0 ||
        joint_limit_margin < 0.0 || margin_joint_names.empty() ||
        configured_object_yaw_symmetry_order < 0 ||
        !std::isfinite(square_aspect_ratio_max) ||
        square_aspect_ratio_max < 1.0 ||
        max_vertical_tilt <= 0.0 || max_vertical_tilt >= M_PI_2) {
      throw std::invalid_argument("place/return-home parameters are invalid");
    }
    const Eigen::Quaterniond vertical_test(
        Eigen::AngleAxisd(grasp_pitch, Eigen::Vector3d::UnitY()) *
        Eigen::AngleAxisd(grasp_roll, Eigen::Vector3d::UnitX()));
    const double downward_alignment =
        (vertical_test * Eigen::Vector3d::UnitX()).dot(-Eigen::Vector3d::UnitZ());
    if (downward_alignment < std::cos(max_vertical_tilt)) {
      throw std::invalid_argument(
          "configured grasp frame is not vertical: its +X approach axis must "
          "align with world -Z");
    }
    if ((grasp_yaw_reference != "world" &&
         grasp_yaw_reference != "object") ||
        (place_yaw_reference != "world" && place_yaw_reference != "table")) {
      throw std::invalid_argument(
          "yaw references must be grasp: world/object and place: world/table");
    }
    for (const double tolerance : object_max_motion_xyz) {
      if (!std::isfinite(tolerance) || tolerance < 0.0) {
        throw std::invalid_argument(
            "object.max_motion_xyz values must be finite and non-negative");
      }
    }
    for (const double tolerance : object_max_size_change) {
      if (!std::isfinite(tolerance) || tolerance < 0.0) {
        throw std::invalid_argument(
            "object.max_size_change values must be finite and non-negative");
      }
    }
    for (const double tolerance : table_max_motion_xyz) {
      if (!std::isfinite(tolerance) || tolerance < 0.0) {
        throw std::invalid_argument(
            "table.max_motion_xyz values must be finite and non-negative");
      }
    }
    for (const double tolerance : table_max_size_change) {
      if (!std::isfinite(tolerance) || tolerance < 0.0) {
        throw std::invalid_argument(
            "table.max_size_change values must be finite and non-negative");
      }
    }
    const bool will_execute = openarm_mtc::can_execute(
        run_mode, execute, allow_execution, allow_full_execution);
    if (production_demo &&
        (run_mode != openarm_mtc::RunMode::Full || !will_execute)) {
      throw std::runtime_error(
          "production demo requires one-shot run_mode=full with all execution "
          "latches open; use dev_pick.launch.py for plan-only/stage tests");
    }
    if (execute && !will_execute) {
      if (run_mode == openarm_mtc::RunMode::PlanOnly) {
        throw std::runtime_error(
            "run_mode=plan_only cannot execute a trajectory; select a stage "
            "mode (open_only ... full) and set execute:=true with "
            "allow_execution:=true");
      }
      if (run_mode == openarm_mtc::RunMode::Full) {
        throw std::runtime_error(
            "full execution latch is closed; run_mode=full additionally "
            "requires allow_full_execution:=true");
      }
      throw std::runtime_error(
          "execution latch is closed; set execute:=true and "
          "allow_execution:=true for the selected stage mode");
    }
    RCLCPP_INFO(node->get_logger(),
                "run_mode=%s execute=%d allow_execution=%d "
                "allow_full_execution=%d -> will_execute=%d",
                openarm_mtc::run_mode_name(run_mode).c_str(), execute,
                allow_execution, allow_full_execution, will_execute);
    if (production_demo) {
      RCLCPP_INFO(
          node->get_logger(),
          "PRODUCTION ONE-SHOT: one stable visual scene -> one complete plan "
          "-> immediate execution of that same selected solution");
    }
    if (use_receipt_time_for_freshness) {
      RCLCPP_WARN(node->get_logger(),
                  "Using local monotonic receipt time for freshness checks; "
                  "this mode is intended only for mixed-clock Gazebo/fake "
                  "hardware simulation");
    }

    std::mutex pose_mutex;
    std::condition_variable pose_condition;
    std::optional<geometry_msgs::msg::PoseStamped> detected_object_pose;
    std::optional<geometry_msgs::msg::Vector3Stamped> detected_object_dimensions;
    std::optional<geometry_msgs::msg::PoseStamped> latest_table_pose;
    std::optional<geometry_msgs::msg::Vector3Stamped> latest_table_dimensions;
    std::optional<std::chrono::steady_clock::time_point>
        detected_object_pose_received_at;
    std::optional<std::chrono::steady_clock::time_point>
        detected_object_dimensions_received_at;
    std::optional<std::chrono::steady_clock::time_point> table_pose_received_at;
    std::optional<std::chrono::steady_clock::time_point>
        table_dimensions_received_at;
    std::optional<sensor_msgs::msg::JointState> latest_joint_state;
    auto pose_subscription = node->create_subscription<geometry_msgs::msg::PoseStamped>(
        object_pose_topic, 10,
        [&](const geometry_msgs::msg::PoseStamped::SharedPtr message) {
          if (message->header.frame_id != planning_frame) {
            RCLCPP_ERROR(node->get_logger(),
                         "Ignoring %s pose in frame '%s'; expected '%s'",
                         object_id.c_str(), message->header.frame_id.c_str(),
                         planning_frame.c_str());
            return;
          }
          {
            std::lock_guard<std::mutex> lock(pose_mutex);
            detected_object_pose = *message;
            detected_object_pose_received_at =
                std::chrono::steady_clock::now();
          }
          pose_condition.notify_one();
        });
    auto table_pose_subscription =
        node->create_subscription<geometry_msgs::msg::PoseStamped>(
            table_pose_topic, 10,
            [&](const geometry_msgs::msg::PoseStamped::SharedPtr message) {
              if (message->header.frame_id != planning_frame) {
                return;
              }
              {
                std::lock_guard<std::mutex> lock(pose_mutex);
                latest_table_pose = *message;
                table_pose_received_at = std::chrono::steady_clock::now();
              }
              pose_condition.notify_one();
            });
    auto dimensions_subscription =
        node->create_subscription<geometry_msgs::msg::Vector3Stamped>(
            object_dimensions_topic, 10,
            [&](const geometry_msgs::msg::Vector3Stamped::SharedPtr message) {
              if (message->header.frame_id != planning_frame) {
                RCLCPP_ERROR(
                    node->get_logger(),
                    "Ignoring %s dimensions in frame '%s'; expected '%s'",
                    object_id.c_str(), message->header.frame_id.c_str(),
                    planning_frame.c_str());
                return;
              }
              {
                std::lock_guard<std::mutex> lock(pose_mutex);
                detected_object_dimensions = *message;
                detected_object_dimensions_received_at =
                    std::chrono::steady_clock::now();
              }
              pose_condition.notify_one();
            });
    auto table_dimensions_subscription =
        node->create_subscription<geometry_msgs::msg::Vector3Stamped>(
            table_dimensions_topic, 10,
            [&](const geometry_msgs::msg::Vector3Stamped::SharedPtr message) {
              if (message->header.frame_id != planning_frame) {
                return;
              }
              {
                std::lock_guard<std::mutex> lock(pose_mutex);
                latest_table_dimensions = *message;
                table_dimensions_received_at =
                    std::chrono::steady_clock::now();
              }
              pose_condition.notify_one();
            });
    auto joint_state_subscription =
        node->create_subscription<sensor_msgs::msg::JointState>(
            joint_states_topic, 10,
            [&](const sensor_msgs::msg::JointState::SharedPtr message) {
              {
                std::lock_guard<std::mutex> lock(pose_mutex);
                latest_joint_state = *message;
              }
              pose_condition.notify_one();
            });

    if (pose_wait_timeout == 0.0) {
      RCLCPP_INFO(node->get_logger(),
                  "Waiting without timeout for synchronized visual object "
                  "pose on %s and dimensions on %s",
                  object_pose_topic.c_str(), object_dimensions_topic.c_str());
    } else {
      RCLCPP_INFO(node->get_logger(),
                  "Waiting up to %.1f s for synchronized visual object pose "
                  "on %s and dimensions on %s",
                  pose_wait_timeout, object_pose_topic.c_str(),
                  object_dimensions_topic.c_str());
    }
    geometry_msgs::msg::PoseStamped object_pose;
    geometry_msgs::msg::Vector3Stamped object_dimensions;
    {
      std::unique_lock<std::mutex> lock(pose_mutex);
      const auto synchronized_visual_data_ready = [&] {
        if (!detected_object_pose.has_value() ||
            !detected_object_dimensions.has_value()) {
          return false;
        }
        const double delta = std::abs(
            (rclcpp::Time(detected_object_pose->header.stamp) -
             rclcpp::Time(detected_object_dimensions->header.stamp))
                .seconds());
        return std::isfinite(delta) &&
               delta <= object_max_message_time_delta;
      };
      bool visual_data_ready = synchronized_visual_data_ready();
      if (pose_wait_timeout == 0.0) {
        // Poll periodically so Ctrl+C can stop an otherwise unbounded wait even
        // when no publisher has ever produced a message to notify the condition.
        while (rclcpp::ok() && !visual_data_ready) {
          pose_condition.wait_for(lock, std::chrono::milliseconds(500));
          visual_data_ready = synchronized_visual_data_ready();
        }
      } else {
        visual_data_ready = pose_condition.wait_for(
            lock, std::chrono::duration<double>(pose_wait_timeout),
            synchronized_visual_data_ready);
      }
      if (!visual_data_ready) {
        if (!rclcpp::ok()) {
          throw std::runtime_error(
              "interrupted while waiting for synchronized visual object data");
        }
        throw std::runtime_error(
            "timed out waiting for synchronized visual object pose/dimensions on " +
            object_pose_topic + " and " + object_dimensions_topic);
      }
      object_pose = *detected_object_pose;
      object_dimensions = *detected_object_dimensions;
    }
    const double object_message_delta = std::abs(
        (rclcpp::Time(object_pose.header.stamp) -
         rclcpp::Time(object_dimensions.header.stamp))
            .seconds());
    if (!std::isfinite(object_message_delta) ||
        object_message_delta > object_max_message_time_delta) {
      throw std::runtime_error(
          "visual object pose and dimensions are not synchronized (delta=" +
          std::to_string(object_message_delta) + " s)");
    }
    const std::vector<double> object_size{
        object_dimensions.vector.x,
        object_dimensions.vector.y,
        object_dimensions.vector.z,
    };
    for (const double dimension : object_size) {
      if (!std::isfinite(dimension) || dimension <= 0.0) {
        throw std::runtime_error(
            "visual object dimensions must be finite and positive");
      }
    }
    const int object_yaw_symmetry_order =
        configured_object_yaw_symmetry_order > 0
            ? configured_object_yaw_symmetry_order
            : openarm_mtc::planar_symmetry_order(
                  object_size[0], object_size[1], square_aspect_ratio_max);
    std::vector<double> grasp_yaw_candidates;
    std::vector<double> grasp_yaw_candidate_costs;
    grasp_yaw_candidates.reserve(
        static_cast<std::size_t>(object_yaw_symmetry_order) *
        grasp_yaw_refinements.size());
    for (int index = 0; index < object_yaw_symmetry_order; ++index) {
      const double symmetry_yaw =
          2.0 * M_PI * static_cast<double>(index) /
          static_cast<double>(object_yaw_symmetry_order);
      for (const double refinement : grasp_yaw_refinements) {
        grasp_yaw_candidates.push_back(symmetry_yaw + refinement);
        // A 5 degree fallback costs about 87, deliberately above normal full
        // path costs (~30): an available exact-yaw solution always wins.
        grasp_yaw_candidate_costs.push_back(1000.0 * std::abs(refinement));
      }
    }
    const double measured_grasp_width = openarm_mtc::measured_grasp_width(
        object_size[0], object_size[1], object_yaw_symmetry_order);
    const double commanded_grasp_width = openarm_mtc::preloaded_grasp_width(
        measured_grasp_width, aperture_preload);
    // Symmetric angular fingers change their aperture by
    // 2 * contact_radius * sin(-joint_position). Anchor that CAD relation at
    // one measured reference pair instead of assuming every object is 4 cm.
    const double aperture_argument =
        std::sin(-reference_joint_position) +
        (commanded_grasp_width - reference_object_width) /
            (2.0 * finger_contact_radius);
    if (!std::isfinite(aperture_argument) || aperture_argument < -1.0 ||
        aperture_argument > 1.0) {
      throw std::runtime_error(
          "measured object width is outside the angular gripper geometry");
    }
    const double close_position = -std::asin(aperture_argument);
    RCLCPP_INFO(
        node->get_logger(),
        "Dynamic gripper target: measured width=%.3f m, preload=%.3f m, "
        "commanded aperture=%.3f m -> %s=%.3f rad (reference %.3f m at "
        "%.3f rad, CAD contact radius %.3f m)",
        measured_grasp_width, aperture_preload, commanded_grasp_width,
        gripper_joint.c_str(), close_position,
        reference_object_width, reference_joint_position,
        finger_contact_radius);
    if (!std::isfinite(object_pose.pose.position.x) ||
        !std::isfinite(object_pose.pose.position.y) ||
        !std::isfinite(object_pose.pose.position.z)) {
      throw std::runtime_error("visual object pose contains a non-finite position");
    }
    const auto& object_orientation = object_pose.pose.orientation;
    const double object_quaternion_norm = std::sqrt(
        object_orientation.x * object_orientation.x +
        object_orientation.y * object_orientation.y +
        object_orientation.z * object_orientation.z +
        object_orientation.w * object_orientation.w);
    if (!std::isfinite(object_quaternion_norm) ||
        object_quaternion_norm < 0.99 || object_quaternion_norm > 1.01 ||
        std::abs(object_orientation.x) > 0.05 ||
        std::abs(object_orientation.y) > 0.05) {
      throw std::runtime_error(
          "visual object orientation must be a normalized horizontal yaw");
    }
    (void)pose_subscription;
    (void)dimensions_subscription;
    (void)table_pose_subscription;
    (void)table_dimensions_subscription;
    (void)joint_state_subscription;

    moveit_msgs::msg::CollisionObject collision_object;
    collision_object.header = object_pose.header;
    collision_object.id = object_id;
    collision_object.operation = moveit_msgs::msg::CollisionObject::ADD;
    shape_msgs::msg::SolidPrimitive object_primitive;
    object_primitive.type = shape_msgs::msg::SolidPrimitive::BOX;
    object_primitive.dimensions.assign(object_size.begin(), object_size.end());
    collision_object.primitives.push_back(object_primitive);
    geometry_msgs::msg::Pose collision_pose = object_pose.pose;
    collision_pose.orientation.x /= object_quaternion_norm;
    collision_pose.orientation.y /= object_quaternion_norm;
    collision_pose.orientation.z /= object_quaternion_norm;
    collision_pose.orientation.w /= object_quaternion_norm;
    collision_object.primitive_poses.push_back(collision_pose);
    moveit::planning_interface::PlanningSceneInterface planning_scene;
    const auto table_objects = planning_scene.getObjects({table_id});
    const auto table_iterator = table_objects.find(table_id);
    if (table_iterator == table_objects.end()) {
      throw std::runtime_error(
          "dynamic box collision object '" + table_id +
          "' is missing; run scene_manager after the latest visual LOCK");
    }
    const auto& table_object = table_iterator->second;
    if (table_object.primitives.empty() || table_object.primitive_poses.empty() ||
        table_object.primitives.front().type != shape_msgs::msg::SolidPrimitive::BOX ||
        table_object.primitives.front().dimensions.size() < 3) {
      throw std::runtime_error("dynamic box collision object is not a valid box");
    }
    const auto table_collision_pose = compose_poses(
        table_object.pose, table_object.primitive_poses.front());
    const auto& table_dimensions = table_object.primitives.front().dimensions;
    const double table_yaw = yaw_from_pose(table_collision_pose);
    const double object_yaw = yaw_from_pose(object_pose.pose);
    const double grasp_yaw_base =
        grasp_yaw_reference == "object" ? object_yaw : 0.0;
    const double place_yaw_base =
        place_yaw_reference == "table" ? table_yaw : 0.0;
    if (!planning_scene.applyCollisionObject(collision_object)) {
      throw std::runtime_error("MoveIt rejected the visual blue-cube collision object");
    }
    RCLCPP_INFO(node->get_logger(),
                "Visual %s center in %s: [%.3f, %.3f, %.3f], "
                "dimensions=[%.3f, %.3f, %.3f], yaw=%.1f deg, "
                "yaw_symmetry_order=%d",
                object_id.c_str(), planning_frame.c_str(),
                object_pose.pose.position.x, object_pose.pose.position.y,
                object_pose.pose.position.z, object_size[0], object_size[1],
                object_size[2], object_yaw * 180.0 / M_PI,
                object_yaw_symmetry_order);

    mtc::Task task;
    task.stages()->setName(
        "OpenArm " + openarm_mtc::run_mode_name(run_mode) + " (pipeline " +
        openarm_mtc::pipeline_extent_name(openarm_mtc::pipeline_extent_for_mode(
            run_mode)) +
        ")");
    task.loadRobotModel(node);
    const auto model = task.getRobotModel();

    if (!model->getJointModelGroup(arm_group)) {
      throw std::runtime_error("missing planning group: " + arm_group);
    }
    if (!model->getJointModelGroup(gripper_group)) {
      throw std::runtime_error("missing planning group: " + gripper_group);
    }
    if (!model->hasLinkModel(ee_frame)) {
      throw std::runtime_error("missing end-effector link: " + ee_frame);
    }
    if (!model->hasLinkModel(tcp_frame)) {
      throw std::runtime_error(
          "missing generated TCP link '" + tcp_frame +
          "'; launch robot_description with emit_grasp_frame:=true");
    }
    if (!model->hasEndEffector(eef_name)) {
      throw std::runtime_error("missing semantic end effector: " + eef_name);
    }
    const auto* joint = model->getJointModel(gripper_joint);
    if (!joint) {
      throw std::runtime_error("missing gripper joint: " + gripper_joint);
    }
    const auto& bounds = joint->getVariableBounds(gripper_joint);
    if (!bounds.position_bounded_ || open_position < bounds.min_position_ ||
        open_position > bounds.max_position_) {
      throw std::runtime_error("gripper.open_position is outside the robot-model limits");
    }
    if (!bounds.position_bounded_ || close_position < bounds.min_position_ ||
        close_position > bounds.max_position_) {
      throw std::runtime_error(
          "visual-width gripper target is outside the robot-model limits");
    }
    if (close_position < open_position) {
      throw std::runtime_error(
          "measured object is wider than the configured gripper opening");
    }
    if (gripper_touch_links.empty()) {
      throw std::runtime_error("gripper.touch_links must not be empty");
    }
    for (const auto& link_name : gripper_touch_links) {
      if (!model->hasLinkModel(link_name)) {
        throw std::runtime_error("gripper touch link is missing: " + link_name);
      }
    }

    std::map<std::string, double> initial_arm_positions;
    const openarm_mtc::PipelineExtent pipeline_extent =
        openarm_mtc::pipeline_extent_for_mode(run_mode);
    if (pipeline_extent != openarm_mtc::PipelineExtent::OpenOnly) {
      RCLCPP_INFO(node->get_logger(),
                  "Recording initial right-arm joints from %s",
                  joint_states_topic.c_str());
      sensor_msgs::msg::JointState initial_joint_state;
      {
        std::unique_lock<std::mutex> lock(pose_mutex);
        if (!pose_condition.wait_for(
                lock, std::chrono::duration<double>(joint_state_wait_timeout),
                [&] { return latest_joint_state.has_value(); })) {
          throw std::runtime_error("timed out waiting for initial joint state on " +
                                   joint_states_topic);
        }
        initial_joint_state = *latest_joint_state;
      }
      const auto* arm_joint_group = model->getJointModelGroup(arm_group);
      for (const auto& variable : arm_joint_group->getVariableNames()) {
        const auto found = std::find(initial_joint_state.name.begin(),
                                     initial_joint_state.name.end(), variable);
        if (found == initial_joint_state.name.end()) {
          throw std::runtime_error("initial joint state is missing '" + variable + "'");
        }
        const auto index = static_cast<std::size_t>(
            std::distance(initial_joint_state.name.begin(), found));
        if (index >= initial_joint_state.position.size() ||
            !std::isfinite(initial_joint_state.position[index])) {
          throw std::runtime_error("initial joint state for '" + variable +
                                   "' is invalid");
        }
        const double raw_position = initial_joint_state.position[index];
        double clamped_position = raw_position;
        const moveit::core::VariableBounds& variable_bounds =
            model->getVariableBounds(variable);
        if (variable_bounds.position_bounded_) {
          clamped_position = std::min(
              std::max(raw_position, variable_bounds.min_position_),
              variable_bounds.max_position_);
          if (clamped_position != raw_position) {
            RCLCPP_WARN(node->get_logger(),
                        "Clamped start joint '%s': %.6f -> %.6f rad (model "
                        "limits [%.4f, %.4f]); verify the physical arm pose "
                        "or adjust the URDF limits",
                        variable.c_str(), raw_position, clamped_position,
                        variable_bounds.min_position_,
                        variable_bounds.max_position_);
          }
        }
        initial_arm_positions.emplace(variable, clamped_position);
      }
    }

    task.add(std::make_unique<mtc::stages::CurrentState>("current state"));

    auto gripper_planner = std::make_shared<mtc::solvers::JointInterpolationPlanner>();
    gripper_planner->setMaxVelocityScalingFactor(velocity_scaling);
    gripper_planner->setMaxAccelerationScalingFactor(acceleration_scaling);
    auto open = std::make_unique<mtc::stages::MoveTo>("open right gripper", gripper_planner);
    open->setGroup(gripper_group);
    open->setTimeout(planning_timeout);
    open->setGoal(std::map<std::string, double>{{gripper_joint, open_position}});
    mtc::Stage* open_stage_ptr = open.get();
    task.add(std::move(open));

    auto arm_planner =
        std::make_shared<mtc::solvers::PipelinePlanner>(node, "ompl");
    arm_planner->setMaxVelocityScalingFactor(velocity_scaling);
    arm_planner->setMaxAccelerationScalingFactor(acceleration_scaling);
    arm_planner->setProperty("num_planning_attempts", 10u);
    arm_planner->setProperty("goal_position_tolerance", 0.005);
    arm_planner->setProperty("goal_orientation_tolerance", 0.02);

    // Resolve T_ee_tcp from the generated robot model. The OpenArm v2.0 xacro
    // shifts ee_base_link by +20.5 mm; copying the unshifted reference_points
    // coordinates here previously created a yaw-dependent 20.5 mm XY miss.
    moveit::core::RobotState tcp_model_state(model);
    tcp_model_state.setToDefaultValues();
    tcp_model_state.update();
    const Eigen::Isometry3d ee_to_tcp =
        tcp_model_state.getGlobalLinkTransform(ee_frame).inverse() *
        tcp_model_state.getGlobalLinkTransform(tcp_frame);
    if (!ee_to_tcp.matrix().allFinite()) {
      throw std::runtime_error("generated ee->TCP transform contains non-finite values");
    }

    const double collision_table_top =
        table_collision_pose.position.z + 0.5 * table_dimensions[2];
    const double physical_table_top =
        collision_table_top - table_collision_padding_top;
    // support_clearance is a clearance above the collision safety plane, not
    // merely above the raw visual surface. Convert it to the equivalent
    // physical-top clearance for the geometry helpers below.
    const double collision_aware_support_clearance =
        support_clearance + table_collision_padding_top;
    const double object_bottom =
        object_pose.pose.position.z - 0.5 * object_size[2];
    const double object_top =
        object_pose.pose.position.z + 0.5 * object_size[2];
    // The official grasp frame is above the lowest finger collision geometry.
    // Raise the TCP only as much as needed to keep fingertips above the support;
    // taller objects naturally require no offset.
    const double object_support_height_error = object_bottom - physical_table_top;
    if (std::abs(object_support_height_error) >
        object_max_support_height_error) {
      throw std::runtime_error(
          "visual object/box Z snapshots are inconsistent: object bottom - "
          "physical box top = " + std::to_string(object_support_height_error) +
          " m exceeds object.max_support_height_error=" +
          std::to_string(object_max_support_height_error) +
          " m; refresh both visual LOCKs and scene_manager");
    }
    const double grasp_tcp_z = openarm_mtc::safe_grasp_tcp_z(
        object_pose.pose.position.z, object_size[2], physical_table_top,
        tip_below_grasp_frame, collision_aware_support_clearance);
    const double grasp_center_z_offset =
        grasp_tcp_z - object_pose.pose.position.z;
    RCLCPP_INFO(
        node->get_logger(),
        "SCENE_GEOMETRY: table collision center=[%.3f, %.3f, %.3f] "
        "size=[%.3f, %.3f, %.3f] yaw=%.1f deg; physical_top=%.3f, "
        "collision_top=%.3f (top_padding=%.3f); cube center=[%.3f, %.3f, "
        "%.3f] size=[%.3f, %.3f, %.3f] bottom=%.3f top=%.3f; "
        "cube_bottom-physical_top=%+.3f, cube_bottom-collision_top=%+.3f m",
        table_collision_pose.position.x, table_collision_pose.position.y,
        table_collision_pose.position.z, table_dimensions[0],
        table_dimensions[1], table_dimensions[2], table_yaw * 180.0 / M_PI,
        physical_table_top, collision_table_top,
        table_collision_padding_top, object_pose.pose.position.x,
        object_pose.pose.position.y, object_pose.pose.position.z,
        object_size[0], object_size[1], object_size[2], object_bottom,
        object_top, object_support_height_error,
        object_bottom - collision_table_top);

    moveit::core::RobotState ik_seed(model);
    ik_seed.setToDefaultValues();
    if (!initial_arm_positions.empty()) {
      ik_seed.setVariablePositions(initial_arm_positions);
    }
    ik_seed.update();
    const auto* arm_joint_group = model->getJointModelGroup(arm_group);
    const double approach_end_z = grasp_tcp_z;
    RCLCPP_INFO(
        node->get_logger(),
        "GRASP_GEOMETRY: official grasp_frame is the TCP; pregrasp_z=%.3f, "
        "configured approach_end_z=%.3f, object_center_z=%.3f, "
        "dynamic_tcp_z_offset=%+.3f, fingertip clearance: physical=%.3f, "
        "collision_plane=%.3f m",
        grasp_tcp_z + pregrasp_offset,
        approach_end_z,
        object_pose.pose.position.z,
        grasp_center_z_offset,
        grasp_tcp_z - tip_below_grasp_frame - physical_table_top,
        grasp_tcp_z - tip_below_grasp_frame - collision_table_top);
    std::vector<CandidateDiagnostic> candidate_diagnostics;
    candidate_diagnostics.reserve(grasp_yaw_candidates.size());
    if (!production_demo) {
      for (std::size_t index = 0; index < grasp_yaw_candidates.size(); ++index) {
      const double world_yaw = grasp_yaw_base + grasp_yaw_candidates[index];
      geometry_msgs::msg::Pose diagnostic_pose;
      diagnostic_pose.position.x = object_pose.pose.position.x;
      diagnostic_pose.position.y = object_pose.pose.position.y;
      diagnostic_pose.orientation =
          quaternion_from_rpy(grasp_roll, grasp_pitch, world_yaw);
      diagnostic_pose.position.z =
          object_pose.pose.position.z + grasp_center_z_offset + pregrasp_offset;
      const auto pregrasp_ik = try_raw_ik(
          ik_seed, arm_joint_group, pose_to_eigen(diagnostic_pose), ee_to_tcp,
          ee_frame);
      diagnostic_pose.position.z =
          object_pose.pose.position.z + grasp_center_z_offset +
          approach_min_distance;
      const auto minimum_pregrasp_ik = try_raw_ik(
          ik_seed, arm_joint_group, pose_to_eigen(diagnostic_pose), ee_to_tcp,
          ee_frame);
      diagnostic_pose.position.z = approach_end_z;
      const auto approach_end_ik = try_raw_ik(
          ik_seed, arm_joint_group, pose_to_eigen(diagnostic_pose), ee_to_tcp,
          ee_frame);
      diagnostic_pose.position.z =
          object_pose.pose.position.z + grasp_center_z_offset;
      const auto cube_center_ik = try_raw_ik(
          ik_seed, arm_joint_group, pose_to_eigen(diagnostic_pose), ee_to_tcp,
          ee_frame);
      RCLCPP_INFO(
          node->get_logger(),
          "IK_DIAG candidate=%zu yaw_offset=%.1f deg world_yaw=%.1f deg: "
          "pregrasp_max=%s; pregrasp_min=%s; configured_approach_end=%s; "
          "cube_center=%s "
          "(raw kinematic IK only, collision not checked)",
          index, grasp_yaw_candidates[index] * 180.0 / M_PI,
          world_yaw * 180.0 / M_PI,
          format_raw_ik_result(pregrasp_ik).c_str(),
          format_raw_ik_result(minimum_pregrasp_ik).c_str(),
          format_raw_ik_result(approach_end_ik).c_str(),
          format_raw_ik_result(cube_center_ik).c_str());
        candidate_diagnostics.push_back(CandidateDiagnostic{
            index, grasp_yaw_candidates[index], world_yaw,
            pregrasp_ik.has_value() || minimum_pregrasp_ik.has_value(),
            approach_end_ik.has_value(),
            cube_center_ik.has_value()});
      }
      print_grasp_candidate_summary(
          candidate_diagnostics,
          openarm_mtc::extent_uses_pick(pipeline_extent),
          node->get_logger());
    }
    if (!production_demo && enable_tilt_scan) {
      for (const double pitch_offset : diagnostic_pitch_offsets) {
        if (std::abs(pitch_offset) < 1.0e-9) {
          continue;  // The exact-vertical result was printed above.
        }
        for (std::size_t index = 0; index < grasp_yaw_candidates.size();
             ++index) {
          const double world_yaw = grasp_yaw_base + grasp_yaw_candidates[index];
          geometry_msgs::msg::Pose diagnostic_pose;
          diagnostic_pose.position.x = object_pose.pose.position.x;
          diagnostic_pose.position.y = object_pose.pose.position.y;
          diagnostic_pose.position.z =
              object_pose.pose.position.z + grasp_center_z_offset;
          diagnostic_pose.orientation = quaternion_from_rpy(
              grasp_roll, grasp_pitch + pitch_offset, world_yaw);
          const std::string cube_center_ik = raw_ik_diagnostic(
              ik_seed, arm_joint_group, pose_to_eigen(diagnostic_pose),
              ee_to_tcp, ee_frame, 8);
          RCLCPP_INFO(
              node->get_logger(),
              "IK_TILT_SCAN candidate=%zu yaw_offset=%.1f deg "
              "pitch_offset=%+.1f deg: cube_center=%s "
              "(diagnostic only; planning orientation unchanged)",
              index, grasp_yaw_candidates[index] * 180.0 / M_PI,
              pitch_offset * 180.0 / M_PI, cube_center_ik.c_str());
        }
      }
    }

    // §12-14 stage-latch construction. The pipeline is assembled from the same
    // predicates that define the tested abstract plan in run_mode.hpp so the
    // real task and the unit tests cannot drift apart.
    const auto stage_plan = openarm_mtc::stage_plan_for_mode(run_mode);
    {
      std::ostringstream plan_stream;
      for (std::size_t index = 0; index < stage_plan.size(); ++index) {
        if (index > 0) {
          plan_stream << " -> ";
        }
        plan_stream << openarm_mtc::stage_kind_name(stage_plan[index]);
      }
      RCLCPP_INFO(node->get_logger(), "RUN_MODE %s STAGE_PLAN: %s",
                  openarm_mtc::run_mode_name(run_mode).c_str(),
                  plan_stream.str().c_str());
    }

    if (pipeline_extent == openarm_mtc::PipelineExtent::PreGrasp) {
      auto pregrasp_alternatives = std::make_unique<mtc::Alternatives>(
          "vertical PreGrasp yaw candidates");
      std::size_t added_candidate_count = 0;
      for (std::size_t index = 0; index < grasp_yaw_candidates.size(); ++index) {
        // Raw IK is a bounded diagnostic sample, not proof of physical
        // unreachability. Always let the real planning pipeline evaluate every
        // configured yaw so a stochastic diagnostic miss cannot delete a valid
        // hardware pose.
        if (index >= candidate_diagnostics.size() ||
            !candidate_diagnostics[index].raw_pregrasp_ik) {
          RCLCPP_WARN(node->get_logger(),
                      "PreGrasp candidate %zu had no raw diagnostic IK; still "
                      "sending it to OMPL/MTC for authoritative planning",
                      index);
        }
        const std::array<double, 2> distance_candidates{
            pregrasp_offset, approach_min_distance};
        for (std::size_t distance_index = 0;
             distance_index < distance_candidates.size(); ++distance_index) {
          if (distance_index > 0 &&
              std::abs(pregrasp_offset - approach_min_distance) < 1.0e-9) {
            continue;
          }
          geometry_msgs::msg::PoseStamped target;
          target.header.frame_id = planning_frame;
          target.header.stamp = node->now();
          target.pose.position.x = object_pose.pose.position.x;
          target.pose.position.y = object_pose.pose.position.y;
          target.pose.position.z = object_pose.pose.position.z +
                                   grasp_center_z_offset +
                                   distance_candidates[distance_index];
          target.pose.orientation = quaternion_from_rpy(
              grasp_roll, grasp_pitch,
              grasp_yaw_base + grasp_yaw_candidates[index]);

          auto move = std::make_unique<mtc::stages::MoveTo>(
              "vertical PreGrasp yaw " + std::to_string(index) +
                  " distance " + std::to_string(distance_index),
              arm_planner);
          move->setGroup(arm_group);
          move->setTimeout(planning_timeout);
          move->setIKFrame(ee_to_tcp, ee_frame);
          move->setGoal(target);
          pregrasp_alternatives->add(std::move(move));
          ++added_candidate_count;
        }
      }
      RCLCPP_INFO(node->get_logger(),
                  "PreGrasp planning: all %zu/%zu configured yaw candidates "
                  "sent to OMPL (raw IK remains diagnostic only)",
                  added_candidate_count, grasp_yaw_candidates.size());
      task.add(std::move(pregrasp_alternatives));
    }

    std::shared_ptr<mtc::solvers::CartesianPath> cartesian;
    std::size_t valid_place_candidate_count = 0;
    if (openarm_mtc::extent_uses_pick(pipeline_extent)) {
      cartesian = std::make_shared<mtc::solvers::CartesianPath>();
      cartesian->setStepSize(cartesian_step);
      cartesian->setMinFraction(cartesian_min_fraction);
      cartesian->setMaxVelocityScalingFactor(velocity_scaling);
      cartesian->setMaxAccelerationScalingFactor(acceleration_scaling);

      // Connect is solved only after grasp IK and the reverse Cartesian
      // approach have produced compatible pregrasp states.
      auto connect_pick = std::make_unique<mtc::stages::Connect>(
          "connect current state to compatible PreGrasp",
          mtc::stages::Connect::GroupPlannerVector{{arm_group, arm_planner}});
      connect_pick->setTimeout(planning_timeout);
      task.add(std::move(connect_pick));

      auto pick = std::make_unique<mtc::SerialContainer>(
          "vision-driven pick from grasp IK");

      auto approach = std::make_unique<mtc::stages::MoveRelative>(
          "reverse-compute vertical approach", cartesian);
      approach->setGroup(arm_group);
      approach->setTimeout(planning_timeout);
      approach->setIKFrame(ee_to_tcp, ee_frame);
      approach->setMinMaxDistance(approach_min_distance, pregrasp_offset);
      geometry_msgs::msg::Vector3Stamped down;
      down.header.frame_id = planning_frame;
      down.vector.z = -1.0;
      approach->setDirection(down);
      pick->insert(std::move(approach));

      moveit_msgs::msg::RobotState open_gripper_state;
      open_gripper_state.is_diff = true;
      open_gripper_state.joint_state.name = {gripper_joint};
      open_gripper_state.joint_state.position = {open_position};
      moveit_msgs::msg::RobotState closed_gripper_state;
      closed_gripper_state.is_diff = true;
      closed_gripper_state.joint_state.name = {gripper_joint};
      closed_gripper_state.joint_state.position = {close_position};

      auto grasp_ik_alternatives = std::make_unique<mtc::Alternatives>(
          "compute explicit grasp yaw IK candidates");
      for (std::size_t candidate_index = 0;
           candidate_index < grasp_yaw_candidates.size(); ++candidate_index) {
        auto grasp_generator =
            std::make_unique<mtc::stages::GenerateGraspPose>(
                "generate grasp yaw candidate " +
                std::to_string(candidate_index));
        grasp_generator->setObject(object_id);
        grasp_generator->setEndEffector(eef_name);
        grasp_generator->setPreGraspPose(open_gripper_state);
        grasp_generator->setGraspPose(closed_gripper_state);
        // One explicit pose per generator. This supports small nonuniform yaw
        // refinements while retaining an exact candidate/cost identity.
        grasp_generator->setAngleDelta(2.0 * M_PI);
        grasp_generator->setMonitoredStage(open_stage_ptr);
        grasp_generator->setCostTerm(std::make_shared<mtc::cost::Constant>(
            grasp_yaw_candidate_costs[candidate_index]));

        const double desired_world_yaw =
            grasp_yaw_base + grasp_yaw_candidates[candidate_index];
        const double generator_object_yaw = desired_world_yaw - object_yaw;
        const Eigen::Isometry3d grasp_orientation_offset(
            Eigen::AngleAxisd(generator_object_yaw, Eigen::Vector3d::UnitZ()) *
            Eigen::AngleAxisd(grasp_pitch, Eigen::Vector3d::UnitY()) *
            Eigen::AngleAxisd(grasp_roll, Eigen::Vector3d::UnitX()));
        const Eigen::Isometry3d grasp_target_offset =
            Eigen::Translation3d(0.0, 0.0, grasp_center_z_offset) *
            grasp_orientation_offset;
        const Eigen::Isometry3d generator_ik_frame =
            ee_to_tcp * grasp_target_offset.inverse();
        auto grasp_ik = std::make_unique<mtc::stages::ComputeIK>(
            "compute grasp IK candidate " +
                std::to_string(candidate_index),
            std::move(grasp_generator));
        grasp_ik->setGroup(arm_group);
        grasp_ik->setEndEffector(eef_name);
        grasp_ik->setIKFrame(generator_ik_frame, ee_frame);
        grasp_ik->setMaxIKSolutions(static_cast<uint32_t>(max_solutions));
        grasp_ik->setMinSolutionDistance(0.15);
        grasp_ik->properties().configureInitFrom(
            mtc::Stage::INTERFACE, {"target_pose"});
        grasp_ik_alternatives->add(std::move(grasp_ik));
      }
      pick->insert(std::move(grasp_ik_alternatives));

      if (openarm_mtc::extent_uses_close(pipeline_extent)) {
        auto allow_hand_object =
            std::make_unique<mtc::stages::ModifyPlanningScene>(
                "allow collision (right gripper, visual object)");
        allow_hand_object->allowCollisions(object_id, gripper_touch_links, true);
        pick->insert(std::move(allow_hand_object));

        auto close = std::make_unique<mtc::stages::MoveTo>(
            "close right gripper around visual object", gripper_planner);
        close->setGroup(gripper_group);
        close->setTimeout(planning_timeout);
        close->setGoal(
            std::map<std::string, double>{{gripper_joint, close_position}});
        pick->insert(std::move(close));
      }

      if (openarm_mtc::extent_uses_attach_lift(pipeline_extent)) {
        auto allow_object_table =
            std::make_unique<mtc::stages::ModifyPlanningScene>(
                "allow supported object collision with box top");
        allow_object_table->allowCollisions({object_id}, {table_id}, true);
        pick->insert(std::move(allow_object_table));

        auto attach = std::make_unique<mtc::stages::ModifyPlanningScene>(
            "attach visual object to right gripper");
        attach->attachObject(object_id, ee_frame);
        pick->insert(std::move(attach));

        auto lift = std::make_unique<mtc::stages::MoveRelative>(
            "lift attached object along world +Z", cartesian);
        lift->setGroup(arm_group);
        lift->setTimeout(planning_timeout);
        lift->setIKFrame(ee_to_tcp, ee_frame);
        lift->setMinMaxDistance(lift_min_distance, lift_max_distance);
        geometry_msgs::msg::Vector3Stamped up;
        up.header.frame_id = planning_frame;
        up.vector.z = 1.0;
        lift->setDirection(up);
        pick->insert(std::move(lift));

        auto forbid_object_table =
            std::make_unique<mtc::stages::ModifyPlanningScene>(
                "restore object-box collision after lift");
        forbid_object_table->allowCollisions({object_id}, {table_id}, false);
        pick->insert(std::move(forbid_object_table));
      }

      task.add(std::move(pick));
    }

    if (openarm_mtc::extent_uses_place(pipeline_extent)) {
      const double cosine = std::cos(table_yaw);
      const double sine = std::sin(table_yaw);
      const double physical_table_x =
          table_dimensions[0] - 2.0 * table_collision_padding_xy;
      const double physical_table_y =
          table_dimensions[1] - 2.0 * table_collision_padding_xy;
      const double place_physical_table_top =
          table_collision_pose.position.z + 0.5 * table_dimensions[2] -
          table_collision_padding_top;
      const double place_center_z = openarm_mtc::landing_object_center_z(
          place_physical_table_top, object_size[2], place_surface_clearance,
          grasp_center_z_offset, tip_below_grasp_frame,
          collision_aware_support_clearance);
      const double place_tcp_z = place_center_z + grasp_center_z_offset;
      const double place_fingertip_z = place_tcp_z - tip_below_grasp_frame;
      const auto allowed_local_extent = [&](const double landing_world_yaw) {
        // Project the rotated object footprint into the detected box-local
        // axes. Using raw size_x/size_y for every yaw can put a rectangular
        // object's corners outside the real top surface.
        const double relative_yaw = landing_world_yaw - table_yaw;
        const double relative_cosine = std::abs(std::cos(relative_yaw));
        const double relative_sine = std::abs(std::sin(relative_yaw));
        const double projected_half_x =
            0.5 * (relative_cosine * object_size[0] +
                   relative_sine * object_size[1]);
        const double projected_half_y =
            0.5 * (relative_sine * object_size[0] +
                   relative_cosine * object_size[1]);
        return std::array<double, 2>{
            0.5 * physical_table_x - projected_half_x - place_edge_clearance,
            0.5 * physical_table_y - projected_half_y - place_edge_clearance};
      };

      // §12 lift_test lands back on the measured grasp XY (原路Lower) so the
      // object does not move; place_test/full/plan_only use the configured
      // dimensionless box-top fraction candidates.
      std::vector<std::array<double, 2>> place_local_xy;
      std::vector<double> landing_yaws;
      if (openarm_mtc::extent_is_lift_test(pipeline_extent)) {
        const double dx = object_pose.pose.position.x -
                          table_collision_pose.position.x;
        const double dy = object_pose.pose.position.y -
                          table_collision_pose.position.y;
        const double local_x = cosine * dx + sine * dy;
        const double local_y = -sine * dx + cosine * dy;
        const auto allowed = allowed_local_extent(object_yaw);
        if (allowed[0] <= 0.0 || allowed[1] <= 0.0) {
          throw std::runtime_error(
              "dynamic box top is too small for the rotated object and edge "
              "clearance");
        }
        const double fraction_x = local_x / allowed[0];
        const double fraction_y = local_y / allowed[1];
        if (std::abs(fraction_x) > 1.0 || std::abs(fraction_y) > 1.0) {
          throw std::runtime_error(
              "lift_test landing position (the measured grasp XY) is outside "
              "the feasible box-top region; move the object or enlarge edge "
              "clearance");
        }
        place_local_xy.push_back({fraction_x, fraction_y});
        landing_yaws.push_back(object_yaw);
      } else {
        place_local_xy = place_local_xy_fractions;
        landing_yaws.reserve(place_yaw_candidates.size());
        for (const double yaw_offset : place_yaw_candidates) {
          landing_yaws.push_back(place_yaw_base + yaw_offset);
        }
      }

      // The landing generator creates a state in which the object is exactly
      // supported by the measured box top. Put this ACM change in the parent
      // task before Connect/ComputeIK so every generated landing scene inherits
      // it; a stage inside the reverse-propagating place container is too late.
      auto allow_place_support =
          std::make_unique<mtc::stages::ModifyPlanningScene>(
              "allow object contact with box for place IK");
      allow_place_support->allowCollisions({object_id}, {table_id}, true);
      mtc::Stage* place_support_stage_ptr = allow_place_support.get();
      task.add(std::move(allow_place_support));

      auto connect_place = std::make_unique<mtc::stages::Connect>(
          "connect lifted object to compatible PrePlace",
          mtc::stages::Connect::GroupPlannerVector{{arm_group, arm_planner}});
      connect_place->setTimeout(planning_timeout);
      task.add(std::move(connect_place));

      auto place = std::make_unique<mtc::SerialContainer>(
          "vision-driven place from landing IK");

      auto lower = std::make_unique<mtc::stages::MoveRelative>(
          "reverse-compute vertical lower", cartesian);
      lower->setGroup(arm_group);
      lower->setTimeout(planning_timeout);
      lower->setIKFrame(ee_to_tcp, ee_frame);
      lower->setMinMaxDistance(std::max(0.005, lower_distance - 0.005),
                               lower_distance);
      geometry_msgs::msg::Vector3Stamped place_down;
      place_down.header.frame_id = planning_frame;
      place_down.vector.z = -1.0;
      lower->setDirection(place_down);
      place->insert(std::move(lower));

      auto place_ik_alternatives =
          std::make_unique<mtc::Alternatives>("dynamic landing IK candidates");
      for (std::size_t local_index = 0;
           local_index < place_local_xy.size(); ++local_index) {
        const auto& fraction = place_local_xy[local_index];
        for (std::size_t yaw_index = 0; yaw_index < landing_yaws.size();
             ++yaw_index) {
          const auto allowed = allowed_local_extent(landing_yaws[yaw_index]);
          if (allowed[0] <= 0.0 || allowed[1] <= 0.0) {
            RCLCPP_WARN(node->get_logger(),
                        "Skipping landing position %zu yaw %zu: rotated object "
                        "footprint plus %.3f m edge clearance does not fit",
                        local_index, yaw_index, place_edge_clearance);
            continue;
          }
          const std::array<double, 2> local_xy{
              fraction[0] * allowed[0], fraction[1] * allowed[1]};
          const double place_x = table_collision_pose.position.x +
                                 cosine * local_xy[0] - sine * local_xy[1];
          const double place_y = table_collision_pose.position.y +
                                 sine * local_xy[0] + cosine * local_xy[1];
          geometry_msgs::msg::PoseStamped landing_object_pose;
          landing_object_pose.header.frame_id = planning_frame;
          landing_object_pose.header.stamp = node->now();
          landing_object_pose.pose.position.x = place_x;
          landing_object_pose.pose.position.y = place_y;
          landing_object_pose.pose.position.z = place_center_z;
          landing_object_pose.pose.orientation = quaternion_from_rpy(
              0.0, 0.0, landing_yaws[yaw_index]);

          const std::string candidate_suffix =
              " position " + std::to_string(local_index) + " yaw " +
              std::to_string(yaw_index);
          auto place_generator =
              std::make_unique<mtc::stages::GeneratePlacePose>(
                  "generate landing object pose" + candidate_suffix);
          place_generator->setObject(object_id);
          place_generator->setPose(landing_object_pose);
          place_generator->setMonitoredStage(place_support_stage_ptr);

          auto place_ik = std::make_unique<mtc::stages::ComputeIK>(
              "compute landing IK" + candidate_suffix,
              std::move(place_generator));
          place_ik->setGroup(arm_group);
          place_ik->setEndEffector(eef_name);
          place_ik->setMaxIKSolutions(static_cast<uint32_t>(max_solutions));
          place_ik->setMinSolutionDistance(0.15);
          place_ik->properties().configureInitFrom(
              mtc::Stage::INTERFACE, {"target_pose", "ik_frame"});
          place_ik_alternatives->add(std::move(place_ik));
          ++valid_place_candidate_count;
        }
      }
      if (valid_place_candidate_count == 0) {
        throw std::runtime_error(
            "no configured place candidate lies on the current dynamic box top");
      }
      place->insert(std::move(place_ik_alternatives));

      auto open_at_place = std::make_unique<mtc::stages::MoveTo>(
          "open right gripper at place", gripper_planner);
      open_at_place->setGroup(gripper_group);
      open_at_place->setTimeout(planning_timeout);
      open_at_place->setGoal(
          std::map<std::string, double>{{gripper_joint, open_position}});
      place->insert(std::move(open_at_place));

      auto detach = std::make_unique<mtc::stages::ModifyPlanningScene>(
          "detach visual object at place");
      detach->detachObject(object_id, ee_frame);
      place->insert(std::move(detach));

      auto forbid_hand_object =
          std::make_unique<mtc::stages::ModifyPlanningScene>(
              "restore right gripper and cube collision");
      forbid_hand_object->allowCollisions(
          object_id, gripper_touch_links, false);
      place->insert(std::move(forbid_hand_object));

      auto forbid_place_support =
          std::make_unique<mtc::stages::ModifyPlanningScene>(
              "restore object-box collision after detach");
      forbid_place_support->allowCollisions({object_id}, {table_id}, false);
      place->insert(std::move(forbid_place_support));

      auto retreat = std::make_unique<mtc::stages::MoveRelative>(
          "retreat vertically after place", cartesian);
      retreat->setGroup(arm_group);
      retreat->setTimeout(planning_timeout);
      retreat->setIKFrame(ee_to_tcp, ee_frame);
      retreat->setMinMaxDistance(
          std::max(0.005, retreat_distance - 0.005), retreat_distance);
      geometry_msgs::msg::Vector3Stamped retreat_up;
      retreat_up.header.frame_id = planning_frame;
      retreat_up.vector.z = 1.0;
      retreat->setDirection(retreat_up);
      place->insert(std::move(retreat));

      task.add(std::move(place));

      RCLCPP_INFO(node->get_logger(),
                  "Generated %zu landing pose generators (%zu positions x %zu "
                  "yaws) on dynamic box center [%.3f, %.3f]; landing object "
                  "center z=%.3f, TCP z=%.3f, lowest fingertip z=%.3f "
                  "(physical top=%.3f, object clearance>=%.3f, gripper "
                  "physical clearance=%.3f, collision-plane clearance=%.3f m)",
                  valid_place_candidate_count,
                  place_local_xy.size(), landing_yaws.size(),
                  table_collision_pose.position.x,
                  table_collision_pose.position.y, place_center_z,
                  place_tcp_z, place_fingertip_z, place_physical_table_top,
                  place_surface_clearance,
                  place_fingertip_z - place_physical_table_top,
                  place_fingertip_z -
                      (place_physical_table_top + table_collision_padding_top));
    }

    if (openarm_mtc::extent_uses_return(pipeline_extent)) {
      auto stow_gripper = std::make_unique<mtc::stages::MoveTo>(
          "close empty right gripper after release", gripper_planner);
      stow_gripper->setGroup(gripper_group);
      stow_gripper->setTimeout(planning_timeout);
      stow_gripper->setGoal(std::map<std::string, double>{
          {gripper_joint, post_release_position}});
      task.add(std::move(stow_gripper));

      auto return_home = std::make_unique<mtc::stages::MoveTo>(
          "return right arm to recorded initial joints", arm_planner);
      return_home->setGroup(arm_group);
      return_home->setTimeout(planning_timeout);
      return_home->setGoal(initial_arm_positions);
      return_home->restrictDirection(mtc::stages::MoveTo::FORWARD);
      task.add(std::move(return_home));
    }

    RCLCPP_INFO(node->get_logger(),
                "Vertical PreGrasp candidates in %s: xyz=[%.3f, %.3f, %.3f], "
                "roll=%.3f pitch=%.3f, visual yaw base=%.1f deg; approach "
                "ends %.3f m above cube center",
                planning_frame.c_str(), object_pose.pose.position.x,
                object_pose.pose.position.y,
                object_pose.pose.position.z + grasp_center_z_offset +
                    pregrasp_offset,
                grasp_roll,
                grasp_pitch, grasp_yaw_base * 180.0 / M_PI,
                0.0);
    // §17 TCP diagnostics: make the whole model chain explicit so a real-robot
    // comparison (planned TCP vs post-execution FK vs physical gripper) does
    // not depend on hidden math. These prints match the exact expressions used
    // to build the stages below.
    {
      // Staged PreGrasp and full reverse approach share this exact target.
      const double pregrasp_tcp_x = object_pose.pose.position.x;
      const double pregrasp_tcp_y = object_pose.pose.position.y;
      const double pregrasp_tcp_z = object_pose.pose.position.z +
                                    grasp_center_z_offset + pregrasp_offset;
      RCLCPP_INFO(node->get_logger(),
                  "OBJECT POSE WORLD: frame=%s x=%.4f y=%.4f z=%.4f "
                  "yaw=%.1f deg size=[%.4f, %.4f, %.4f] m",
                  planning_frame.c_str(), object_pose.pose.position.x,
                  object_pose.pose.position.y, object_pose.pose.position.z,
                  object_yaw * 180.0 / M_PI, object_size[0], object_size[1],
                  object_size[2]);
      const Eigen::Quaterniond tcp_rotation(ee_to_tcp.linear());
      RCLCPP_INFO(node->get_logger(),
                  "TCP TRANSFORM FROM ROBOT MODEL (%s -> %s): "
                  "translation=[%.6f, %.6f, %.6f] m quaternion_xyzw="
                  "[%.6f, %.6f, %.6f, %.6f]",
                  ee_frame.c_str(), tcp_frame.c_str(),
                  ee_to_tcp.translation().x(), ee_to_tcp.translation().y(),
                  ee_to_tcp.translation().z(), tcp_rotation.x(),
                  tcp_rotation.y(), tcp_rotation.z(), tcp_rotation.w());
      RCLCPP_INFO(node->get_logger(),
                  "FINAL GRASP TCP TARGET: x=%.4f y=%.4f z=%.4f "
                  "rpy=(roll=%.4f pitch=%.4f yaw=%.4f) rad",
                  object_pose.pose.position.x, object_pose.pose.position.y,
                  object_pose.pose.position.z + grasp_center_z_offset,
                  grasp_roll, grasp_pitch, grasp_yaw_base);
      RCLCPP_INFO(node->get_logger(),
                  "FINAL PREGRASP TCP TARGET (VerticalPreGrasp MoveTo goal): "
                  "x=%.4f y=%.4f z=%.4f rpy=(roll=%.4f pitch=%.4f "
                  "yaw=%.4f) rad",
                  pregrasp_tcp_x, pregrasp_tcp_y, pregrasp_tcp_z, grasp_roll,
                  grasp_pitch, grasp_yaw_base);
      RCLCPP_INFO(node->get_logger(),
                  "pregrasp_dx = pregrasp_tcp.x - object.x = %.4f m, "
                  "pregrasp_dy = pregrasp_tcp.y - object.y = %.4f m "
                  "(expected ~0 for a strict vertical pregrasp)",
                  pregrasp_tcp_x - object_pose.pose.position.x,
                  pregrasp_tcp_y - object_pose.pose.position.y);
      RCLCPP_INFO(node->get_logger(),
                  "PreGrasp target is shared by staged/full execution; "
                  "grasp_center_z_offset=%+.3f m",
                  grasp_center_z_offset);
    }
    RCLCPP_INFO(node->get_logger(),
                "RUN_MODE %s pipeline %s: %zu abstract stages; stops after the "
                "last stage of that mode (no automatic next stage)",
                openarm_mtc::run_mode_name(run_mode).c_str(),
                openarm_mtc::pipeline_extent_name(pipeline_extent).c_str(),
                stage_plan.size());

    task.init();
    if (!task.plan(static_cast<std::size_t>(max_solutions))) {
      print_mtc_stage_summary(task, node->get_logger());
      std::ostringstream state_report;
      task.printState(state_report);
      std::ostringstream failure_report;
      task.explainFailure(failure_report);
      RCLCPP_ERROR(node->get_logger(), "MTC_STAGE_STATE:\n%s",
                   state_report.str().c_str());
      RCLCPP_ERROR(node->get_logger(), "MTC_FAILURE_EXPLANATION:\n%s",
                   failure_report.str().c_str());
      throw std::runtime_error(
          "run_mode=" + openarm_mtc::run_mode_name(run_mode) +
          " planning failed (TASK_FAILED: see MTC STAGE SUMMARY for the "
          "first zero-solution stage); inspect MTC failures and PlanningScene "
          "in RViz");
    }
    const mtc::SolutionBase* selected_solution = nullptr;
    for (const auto& candidate : task.solutions()) {
      std::string margin_reason;
      if (has_joint_limit_margin(*candidate, *model, margin_joint_names,
                                 joint_limit_margin, margin_reason)) {
        selected_solution = candidate.get();
        break;
      }
      RCLCPP_WARN(node->get_logger(),
                  "Rejecting planned solution without wrist-joint margin: %s",
                  margin_reason.c_str());
    }
    if (!selected_solution) {
      throw std::runtime_error(
          "all planned solutions approach a configured wrist joint limit; "
          "move the object/platform or add grasp candidates and replan");
    }
    const auto& solution = *selected_solution;
    task.introspection().publishSolution(solution);
    RCLCPP_INFO(node->get_logger(),
                "run_mode=%s plan succeeded with %zu solution(s)",
                openarm_mtc::run_mode_name(run_mode).c_str(),
                task.solutions().size());
    print_mtc_stage_summary(task, node->get_logger());
    if (production_demo) {
      RCLCPP_INFO(node->get_logger(),
                  "PRODUCTION PLAN READY: %zu complete solution(s), selected "
                  "cost=%.6f; executing immediately after scene recheck",
                  task.solutions().size(), solution.cost());
    } else {
      // § problem 6: explicitly identify the selected solution (candidate id,
      // yaw, final TCP, final joints, cost) before any execution happens.
      Eigen::Isometry3d selected_world_tcp = Eigen::Isometry3d::Identity();
      std::vector<double> selected_joints;
      if (solution.end() && solution.end()->scene()) {
        const moveit::core::RobotState& final_state =
            solution.end()->scene()->getCurrentState();
        selected_world_tcp =
            final_state.getGlobalLinkTransform(ee_frame) * ee_to_tcp;
        final_state.copyJointGroupPositions(arm_joint_group, selected_joints);
      }
      const double selected_world_yaw =
          openarm_mtc::vertical_grasp_yaw(selected_world_tcp.linear());
      const bool terminal_preserves_grasp_yaw =
          pipeline_extent == openarm_mtc::PipelineExtent::PreGrasp ||
          pipeline_extent == openarm_mtc::PipelineExtent::Approach ||
          pipeline_extent == openarm_mtc::PipelineExtent::Close ||
          pipeline_extent == openarm_mtc::PipelineExtent::Lift;
      std::size_t selected_candidate_index = 0;
      double best_yaw_distance = std::numeric_limits<double>::max();
      if (terminal_preserves_grasp_yaw) {
        for (const auto& diagnostic : candidate_diagnostics) {
          // Candidate identity must use the actual commanded world yaw. A
          // symmetry-reduced comparison makes 90/180-degree alternatives tie
          // and can report a candidate whose IK branch never succeeded.
          const double distance = angle_distance(
              selected_world_yaw, diagnostic.world_yaw_rad);
          if (distance < best_yaw_distance) {
            best_yaw_distance = distance;
            selected_candidate_index = diagnostic.index;
          }
        }
      }
      std::ostringstream stream;
      stream << "SELECTED SOLUTION\n";
      if (terminal_preserves_grasp_yaw) {
        stream << "  candidate_id=" << selected_candidate_index << "\n";
        stream << "  visual_yaw=" << std::fixed << std::setprecision(1)
               << (grasp_yaw_base +
                   grasp_yaw_candidates[selected_candidate_index]) * 180.0 /
                      M_PI
               << " deg\n";
        stream << "  yaw_offset=" << std::fixed << std::setprecision(1)
               << grasp_yaw_candidates[selected_candidate_index] * 180.0 /
                      M_PI
               << " deg\n";
      } else {
        stream << "  candidate_id=(not inferred from terminal "
               << openarm_mtc::pipeline_extent_name(pipeline_extent)
               << " state)\n";
      }
      stream << "  final_tcp_xyz=[" << std::fixed << std::setprecision(4)
             << selected_world_tcp.translation().x() << ", "
             << selected_world_tcp.translation().y() << ", "
             << selected_world_tcp.translation().z() << "] m\n";
      stream << "  final_tcp_yaw=" << std::fixed << std::setprecision(3)
             << selected_world_yaw << " rad\n";
      stream << "  final_joints=[";
      for (std::size_t index = 0; index < selected_joints.size(); ++index) {
        if (index > 0) {
          stream << ", ";
        }
        stream << std::fixed << std::setprecision(4) << selected_joints[index];
      }
      stream << "]\n";
      stream << "  cost=" << std::fixed << std::setprecision(6)
             << solution.cost();
      RCLCPP_INFO(node->get_logger(), "%s", stream.str().c_str());
    }

    if (!will_execute) {
      RCLCPP_INFO(node->get_logger(),
                  "PLAN ONLY: no trajectory was executed. Re-run with "
                  "execute:=true and allow_execution:=true (plus "
                  "allow_full_execution:=true for run_mode=full) after RViz "
                  "review");
    } else {
      geometry_msgs::msg::PoseStamped current_object_pose;
      geometry_msgs::msg::Vector3Stamped current_object_dimensions;
      std::optional<geometry_msgs::msg::PoseStamped> current_table_pose;
      std::optional<geometry_msgs::msg::Vector3Stamped>
          current_table_dimensions;
      std::optional<std::chrono::steady_clock::time_point>
          current_object_pose_received_at;
      std::optional<std::chrono::steady_clock::time_point>
          current_object_dimensions_received_at;
      std::optional<std::chrono::steady_clock::time_point>
          current_table_pose_received_at;
      std::optional<std::chrono::steady_clock::time_point>
          current_table_dimensions_received_at;
      const auto message_age = [&](const std_msgs::msg::Header& header,
                                   const std::optional<std::chrono::steady_clock::time_point>&
                                       received_at) {
        if (use_receipt_time_for_freshness) {
          if (!received_at.has_value()) {
            return std::numeric_limits<double>::infinity();
          }
          return std::chrono::duration<double>(
                     std::chrono::steady_clock::now() - *received_at)
              .count();
        }
        return (node->now() - rclcpp::Time(header.stamp)).seconds();
      };
      const auto age_is_fresh = [](const double age,
                                   const double maximum_age) {
        return std::isfinite(age) && age >= -0.5 && age <= maximum_age;
      };

      // Planning can temporarily saturate the CPU that also runs the RGB-D
      // detector.  Do not reject a safe full solution because the most recent
      // image happens to cross the age limit by a few milliseconds at the
      // exact instant planning returns.  Keep the strict age limit, but allow
      // perception a short bounded recovery window and require an actually
      // fresh synchronized object/table pair before taking the execution
      // snapshot.  A stopped camera or detector still times out and refuses
      // execution.
      {
        std::unique_lock<std::mutex> lock(pose_mutex);
        const auto fresh_visual_snapshot_ready = [&] {
          if (!detected_object_pose.has_value() ||
              !detected_object_dimensions.has_value() ||
              !detected_object_pose_received_at.has_value() ||
              !detected_object_dimensions_received_at.has_value()) {
            return false;
          }
          const double object_delta = std::abs(
              (rclcpp::Time(detected_object_pose->header.stamp) -
               rclcpp::Time(detected_object_dimensions->header.stamp))
                  .seconds());
          if (!std::isfinite(object_delta) ||
              object_delta > object_max_message_time_delta ||
              !age_is_fresh(message_age(detected_object_pose->header,
                                        detected_object_pose_received_at),
                            object_max_pose_age) ||
              !age_is_fresh(message_age(detected_object_dimensions->header,
                                        detected_object_dimensions_received_at),
                            object_max_pose_age)) {
            return false;
          }
          if (!table_recheck_before_execute) {
            return true;
          }
          if (!latest_table_pose.has_value() ||
              !latest_table_dimensions.has_value() ||
              !table_pose_received_at.has_value() ||
              !table_dimensions_received_at.has_value()) {
            return false;
          }
          const double table_delta = std::abs(
              (rclcpp::Time(latest_table_pose->header.stamp) -
               rclcpp::Time(latest_table_dimensions->header.stamp))
                  .seconds());
          return std::isfinite(table_delta) &&
                 table_delta <= table_max_message_time_delta &&
                 age_is_fresh(message_age(latest_table_pose->header,
                                          table_pose_received_at),
                              table_max_pose_age) &&
                 age_is_fresh(message_age(latest_table_dimensions->header,
                                          table_dimensions_received_at),
                              table_max_pose_age);
        };

        if (!fresh_visual_snapshot_ready()) {
          RCLCPP_WARN(
              node->get_logger(),
              "Execution recheck is waiting up to %.2f s for a fresh "
              "synchronized visual snapshot; safety age limits remain "
              "object=%.2f s table=%.2f s",
              visual_recheck_wait_timeout, object_max_pose_age,
              table_max_pose_age);
          const bool refreshed = pose_condition.wait_for(
              lock, std::chrono::duration<double>(visual_recheck_wait_timeout),
              fresh_visual_snapshot_ready);
          if (!refreshed) {
            throw std::runtime_error(
                "no fresh synchronized visual snapshot arrived within " +
                std::to_string(visual_recheck_wait_timeout) +
                " s after planning; camera/detector may be stalled, execution "
                "refused");
          }
          RCLCPP_INFO(node->get_logger(),
                      "Fresh synchronized visual snapshot recovered after "
                      "planning");
        }
        // Copy the exact pair that satisfied the predicate while the mutex is
        // still held. Pose and dimensions are separate ROS messages, so
        // releasing and reacquiring here could otherwise capture half of the
        // next pair and create another boundary race.
        current_object_pose = *detected_object_pose;
        current_object_pose_received_at = detected_object_pose_received_at;
        current_object_dimensions = *detected_object_dimensions;
        current_object_dimensions_received_at =
            detected_object_dimensions_received_at;
        current_table_pose = latest_table_pose;
        current_table_pose_received_at = table_pose_received_at;
        current_table_dimensions = latest_table_dimensions;
        current_table_dimensions_received_at = table_dimensions_received_at;
      }
      const auto require_fresh = [&](const std_msgs::msg::Header& header,
                                     const std::optional<
                                         std::chrono::steady_clock::time_point>&
                                         received_at,
                                     const double maximum_age,
                                     const std::string& label) {
        const double age = message_age(header, received_at);
        if (!age_is_fresh(age, maximum_age)) {
          throw std::runtime_error(
              label + " pose is stale at execution time (age=" +
              std::to_string(age) + " s)");
        }
      };
      require_fresh(current_object_pose.header, current_object_pose_received_at,
                    object_max_pose_age, object_id);
      require_fresh(current_object_dimensions.header,
                    current_object_dimensions_received_at,
                    object_max_pose_age, object_id + " dimensions");
      const double current_object_message_delta = std::abs(
          (rclcpp::Time(current_object_pose.header.stamp) -
           rclcpp::Time(current_object_dimensions.header.stamp))
              .seconds());
      if (!std::isfinite(current_object_message_delta) ||
          current_object_message_delta > object_max_message_time_delta) {
        throw std::runtime_error(
            "object pose/dimensions lost synchronization before execution");
      }
      const auto& planned = object_pose.pose;
      const auto& current = current_object_pose.pose;
      if (std::abs(current.position.x - planned.position.x) >
              object_max_motion_xyz[0] ||
          std::abs(current.position.y - planned.position.y) >
              object_max_motion_xyz[1] ||
          std::abs(current.position.z - planned.position.z) >
              object_max_motion_xyz[2] ||
          symmetric_angle_distance(yaw_from_pose(current),
                                   yaw_from_pose(planned),
                                   object_yaw_symmetry_order) >
              object_max_motion_yaw) {
        throw std::runtime_error(
            "object moved after planning; execution refused, acquire a new "
            "visual LOCK and replan");
      }
      const std::array<double, 3> current_size{
          current_object_dimensions.vector.x,
          current_object_dimensions.vector.y,
          current_object_dimensions.vector.z,
      };
      for (std::size_t index = 0; index < current_size.size(); ++index) {
        if (!std::isfinite(current_size[index]) || current_size[index] <= 0.0 ||
            std::abs(current_size[index] - object_size[index]) >
                object_max_size_change[index]) {
          throw std::runtime_error(
              "object dimensions changed after planning; execution refused, "
              "acquire a new visual LOCK and replan");
        }
      }
      if (table_recheck_before_execute) {
        if (!current_table_pose.has_value() ||
            !current_table_dimensions.has_value()) {
          throw std::runtime_error(
              "no synchronized live table geometry available for execution-time recheck");
        }
        require_fresh(current_table_pose->header,
                      current_table_pose_received_at, table_max_pose_age,
                      table_id);
        require_fresh(current_table_dimensions->header,
                      current_table_dimensions_received_at,
                      table_max_pose_age, table_id + " dimensions");
        const double table_message_delta = std::abs(
            (rclcpp::Time(current_table_pose->header.stamp) -
             rclcpp::Time(current_table_dimensions->header.stamp))
                .seconds());
        if (!std::isfinite(table_message_delta) ||
            table_message_delta > table_max_message_time_delta) {
          throw std::runtime_error(
              "box pose/dimensions lost synchronization before execution");
        }
        const std::array<double, 3> table_motion{
            std::abs(current_table_pose->pose.position.x -
                     table_collision_pose.position.x),
            std::abs(current_table_pose->pose.position.y -
                     table_collision_pose.position.y),
            std::abs(current_table_pose->pose.position.z -
                     (table_collision_pose.position.z -
                      0.5 * table_collision_padding_top))};
        const double table_yaw_motion = angle_distance(
            yaw_from_pose(current_table_pose->pose), table_yaw);
        const std::array<double, 3> current_table_size{
            current_table_dimensions->vector.x,
            current_table_dimensions->vector.y,
            current_table_dimensions->vector.z};
        std::array<double, 3> table_size_delta{};
        bool table_geometry_changed =
            table_yaw_motion > table_max_motion_yaw;
        for (std::size_t index = 0; index < 3; ++index) {
          const double planned_physical_size =
              table_dimensions[index] -
              (index < 2 ? 2.0 * table_collision_padding_xy
                         : table_collision_padding_top);
          table_size_delta[index] =
              std::abs(current_table_size[index] - planned_physical_size);
          if (!std::isfinite(current_table_size[index]) ||
              current_table_size[index] <= 0.0 ||
              table_motion[index] > table_max_motion_xyz[index] ||
              table_size_delta[index] > table_max_size_change[index]) {
            table_geometry_changed = true;
          }
        }
        if (table_geometry_changed) {
          RCLCPP_ERROR(
              node->get_logger(),
              "Execution-time box delta: position_abs=[%.4f, %.4f, %.4f] "
              "m (limits=[%.4f, %.4f, %.4f]), size_abs=[%.4f, %.4f, "
              "%.4f] m (limits=[%.4f, %.4f, %.4f]), yaw_abs=%.2f deg "
              "(limit=%.2f deg)",
              table_motion[0], table_motion[1], table_motion[2],
              table_max_motion_xyz[0], table_max_motion_xyz[1],
              table_max_motion_xyz[2], table_size_delta[0],
              table_size_delta[1], table_size_delta[2],
              table_max_size_change[0], table_max_size_change[1],
              table_max_size_change[2], table_yaw_motion * 180.0 / M_PI,
              table_max_motion_yaw * 180.0 / M_PI);
          throw std::runtime_error(
              "box/platform geometry changed after its PlanningScene snapshot; execution "
              "refused, rerun scene_manager and replan");
        }
      }
      RCLCPP_INFO(node->get_logger(),
                  "Execution-time visual recheck passed: object and platform "
                  "still match the planned scene");
      const auto execution_result =
          execute_sanitized_solution(task, solution, node->get_logger());
      if (execution_result.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS) {
        throw std::runtime_error(
            "run_mode=" + openarm_mtc::run_mode_name(run_mode) +
            " execution failed with MoveIt error " +
            std::to_string(execution_result.val));
      }
      RCLCPP_INFO(node->get_logger(), "run_mode=%s execution completed",
                  openarm_mtc::run_mode_name(run_mode).c_str());
      // § problem 8: optional controller tracking report. Reads the controller
      // state right after the trajectory finished; a nonzero residual here is
      // expected diagnostic data, not a vision offset to be compensated.
      if (controller_tracking) {
        print_controller_tracking(controller_state_topic, node->get_logger());
      }
      // §20 post-execution FK check. The planned target TCP is compared
      // against (a) the model TCP from the executed solution's final state
      // and (b) the TCP computed from the latest /joint_states. This is what
      // separates a model error (target/IK/setIKFrame) from a real-robot
      // error (joint tracking / gravity / URDF / physical TCP). A physical
      // ruler measurement of the real gripper is the final step of the chain.
      // A full run ends at the recorded home joints, so comparing that final
      // state to the earlier PreGrasp target would print a meaningless
      // 20-30 cm "error". This diagnostic is valid only when PreGrasp is the
      // terminal stage.
      if (pipeline_extent == openarm_mtc::PipelineExtent::PreGrasp) {
        Eigen::Isometry3d model_world_tcp = Eigen::Isometry3d::Identity();
        if (solution.end() && solution.end()->scene()) {
          const moveit::core::RobotState& model_final =
              solution.end()->scene()->getCurrentState();
          model_world_tcp =
              model_final.getGlobalLinkTransform(ee_frame) * ee_to_tcp;
        }
        moveit::core::RobotState actual(model);
        actual.setToDefaultValues();
        {
          std::lock_guard<std::mutex> lock(pose_mutex);
          if (latest_joint_state.has_value()) {
            actual.setVariablePositions(latest_joint_state->name,
                                        latest_joint_state->position);
          }
        }
        actual.update();
        const Eigen::Isometry3d actual_world_tcp =
            actual.getGlobalLinkTransform(ee_frame) * ee_to_tcp;
        const Eigen::Vector3d actual_dxyz(
            actual_world_tcp.translation() - model_world_tcp.translation());
        RCLCPP_INFO(node->get_logger(),
                    "PLANNED PREGRASP TCP FROM SELECTED SOLUTION: x=%.4f "
                    "y=%.4f z=%.4f",
                    model_world_tcp.translation().x(),
                    model_world_tcp.translation().y(),
                    model_world_tcp.translation().z());
        RCLCPP_INFO(node->get_logger(),
                    "FK TCP from /joint_states (actual): x=%.4f y=%.4f "
                    "z=%.4f; dxyz_vs_planned=[%+.4f, %+.4f, %+.4f] m",
                    actual_world_tcp.translation().x(),
                    actual_world_tcp.translation().y(),
                    actual_world_tcp.translation().z(), actual_dxyz.x(),
                    actual_dxyz.y(), actual_dxyz.z());
        RCLCPP_INFO(node->get_logger(),
                    "FK q_actual vs q_planned: compare the /joint_states "
                    "positions with the solution final state per joint "
                    "in RViz/joint_states to identify which joints carry "
                    "the residual error");
      }
    }
    // §14 terminal latch: only claim stage completion when the mode actually
    // ran its stages; plan_only is the only mode whose "completion" is a plan.
    if (production_demo && will_execute) {
      RCLCPP_INFO(node->get_logger(),
                  "OPENARM PICK DEMO COMPLETE - full visual pick/place/return "
                  "succeeded");
    } else if (will_execute || run_mode == openarm_mtc::RunMode::PlanOnly) {
      RCLCPP_INFO(node->get_logger(), "%s",
                  openarm_mtc::test_complete_message(run_mode).c_str());
    } else {
      RCLCPP_INFO(node->get_logger(),
                  "RUN_MODE %s PLANNED ONLY - no trajectory was executed; "
                  "set execute:=true with allow_execution:=true to run this "
                  "stage",
                  openarm_mtc::run_mode_name(run_mode).c_str());
    }
  } catch (const std::exception& error) {
    RCLCPP_ERROR(node->get_logger(), "Visual grasp task failed: %s", error.what());
    result = 1;
  }

  rclcpp::shutdown();
  spinner.join();
  return result;
}
