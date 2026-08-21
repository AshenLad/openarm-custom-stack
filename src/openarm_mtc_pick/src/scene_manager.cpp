#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.hpp>
#include <moveit_msgs/msg/collision_object.hpp>
#include <rclcpp/rclcpp.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>

namespace {

bool is_finite(const double value) { return std::isfinite(value); }

void require_three(const std::string& name, const std::vector<double>& values) {
  if (values.size() != 3) {
    throw std::invalid_argument(name + " must contain exactly 3 values");
  }
}

geometry_msgs::msg::Pose yaw_pose(const std::vector<double>& position,
                                  const double yaw) {
  require_three("table.position", position);
  geometry_msgs::msg::Pose pose;
  pose.position.x = position[0];
  pose.position.y = position[1];
  pose.position.z = position[2];
  pose.orientation.z = std::sin(yaw * 0.5);
  pose.orientation.w = std::cos(yaw * 0.5);
  return pose;
}

moveit_msgs::msg::CollisionObject make_box(
    const std::string& id, const std::string& frame,
    const std::vector<double>& dimensions, const geometry_msgs::msg::Pose& pose) {
  require_three(id + " dimensions", dimensions);
  for (const double dimension : dimensions) {
    if (!is_finite(dimension) || dimension <= 0.0) {
      throw std::invalid_argument(id + " dimensions must be finite and positive");
    }
  }

  moveit_msgs::msg::CollisionObject object;
  object.header.frame_id = frame;
  object.id = id;
  object.operation = moveit_msgs::msg::CollisionObject::ADD;

  shape_msgs::msg::SolidPrimitive primitive;
  primitive.type = shape_msgs::msg::SolidPrimitive::BOX;
  primitive.dimensions.assign(dimensions.begin(), dimensions.end());
  object.primitives.push_back(primitive);
  object.primitive_poses.push_back(pose);
  return object;
}

struct VisualBox {
  geometry_msgs::msg::Pose pose;
  std::vector<double> dimensions;
};

VisualBox wait_for_visual_box(
    const rclcpp::Node::SharedPtr& node, const std::string& planning_frame,
    const std::string& pose_topic, const std::string& dimensions_topic,
    const double timeout_seconds, const double max_message_delta_seconds) {
  if (!is_finite(timeout_seconds) || timeout_seconds < 0.0 ||
      !is_finite(max_message_delta_seconds) || max_message_delta_seconds < 0.0) {
    throw std::invalid_argument(
        "visual message timeout/delta is invalid; timeout 0 waits until "
        "synchronized visual data arrives");
  }
  geometry_msgs::msg::PoseStamped::SharedPtr latest_pose;
  geometry_msgs::msg::Vector3Stamped::SharedPtr latest_dimensions;
  const auto pose_subscription =
      node->create_subscription<geometry_msgs::msg::PoseStamped>(
          pose_topic, 10,
          [&latest_pose](geometry_msgs::msg::PoseStamped::SharedPtr message) {
            latest_pose = std::move(message);
          });
  const auto dimensions_subscription =
      node->create_subscription<geometry_msgs::msg::Vector3Stamped>(
          dimensions_topic, 10,
          [&latest_dimensions](
              geometry_msgs::msg::Vector3Stamped::SharedPtr message) {
            latest_dimensions = std::move(message);
          });
  (void)pose_subscription;
  (void)dimensions_subscription;

  if (timeout_seconds == 0.0) {
    RCLCPP_INFO(node->get_logger(),
                "Waiting without timeout for synchronized visual box on %s "
                "and %s",
                pose_topic.c_str(), dimensions_topic.c_str());
  } else {
    RCLCPP_INFO(node->get_logger(),
                "Waiting up to %.1f s for synchronized visual box on %s and %s",
                timeout_seconds, pose_topic.c_str(), dimensions_topic.c_str());
  }
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  const auto deadline = std::chrono::steady_clock::now() +
                        std::chrono::duration<double>(timeout_seconds);
  const auto maximum_delta = rclcpp::Duration::from_seconds(max_message_delta_seconds);
  bool synchronized_data_ready = false;
  while (rclcpp::ok() &&
         (timeout_seconds == 0.0 || std::chrono::steady_clock::now() < deadline)) {
    executor.spin_some();
    if (latest_pose && latest_dimensions) {
      const rclcpp::Time pose_stamp(latest_pose->header.stamp);
      const rclcpp::Time dimensions_stamp(latest_dimensions->header.stamp);
      if (std::abs((pose_stamp - dimensions_stamp).nanoseconds()) <=
          maximum_delta.nanoseconds()) {
        synchronized_data_ready = true;
        break;
      }
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }
  executor.remove_node(node);
  if (!synchronized_data_ready) {
    if (!rclcpp::ok()) {
      throw std::runtime_error(
          "interrupted while waiting for synchronized visual box data");
    }
    throw std::runtime_error(
        "timed out: stable visual box pose/dimensions were not received");
  }
  const rclcpp::Time pose_stamp(latest_pose->header.stamp);
  const rclcpp::Time dimensions_stamp(latest_dimensions->header.stamp);
  if (std::abs((pose_stamp - dimensions_stamp).seconds()) >
      max_message_delta_seconds) {
    throw std::runtime_error("visual box pose and dimensions timestamps do not match");
  }
  if (latest_pose->header.frame_id != planning_frame ||
      latest_dimensions->header.frame_id != planning_frame) {
    throw std::runtime_error(
        "visual box messages must both use planning frame '" + planning_frame + "'");
  }

  auto pose = latest_pose->pose;
  const auto& position = pose.position;
  const auto& orientation = pose.orientation;
  if (!is_finite(position.x) || !is_finite(position.y) ||
      !is_finite(position.z) || !is_finite(orientation.x) ||
      !is_finite(orientation.y) || !is_finite(orientation.z) ||
      !is_finite(orientation.w)) {
    throw std::runtime_error("visual box pose contains non-finite values");
  }
  const double quaternion_norm =
      std::sqrt(orientation.x * orientation.x + orientation.y * orientation.y +
                orientation.z * orientation.z + orientation.w * orientation.w);
  if (quaternion_norm < 0.99 || quaternion_norm > 1.01) {
    throw std::runtime_error("visual box orientation quaternion is not normalized");
  }
  pose.orientation.x /= quaternion_norm;
  pose.orientation.y /= quaternion_norm;
  pose.orientation.z /= quaternion_norm;
  pose.orientation.w /= quaternion_norm;
  if (std::abs(pose.orientation.x) > 0.05 ||
      std::abs(pose.orientation.y) > 0.05) {
    throw std::runtime_error("visual box orientation is not horizontal");
  }

  std::vector<double> dimensions{
      latest_dimensions->vector.x,
      latest_dimensions->vector.y,
      latest_dimensions->vector.z,
  };
  for (const double dimension : dimensions) {
    if (!is_finite(dimension) || dimension <= 0.0) {
      throw std::runtime_error("visual box dimensions must be finite and positive");
    }
  }
  return VisualBox{pose, dimensions};
}

}  // namespace

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("openarm_mtc_scene_manager");

  const bool configured = node->declare_parameter("scene.configured", false);
  const std::string planning_frame =
      node->declare_parameter("robot.planning_frame", "world");
  const std::string table_id = node->declare_parameter("table.id", "table");
  const std::string source = node->declare_parameter("table.source", "vision");
  if (!configured) {
    RCLCPP_ERROR(node->get_logger(),
                 "Scene write refused: scene.configured must be true");
    rclcpp::shutdown();
    return 2;
  }

  try {
    geometry_msgs::msg::Pose physical_pose;
    std::vector<double> physical_size;
    if (source == "vision") {
      const std::string pose_topic =
          node->declare_parameter("table.pose_topic", "/box/pose");
      const std::string dimensions_topic =
          node->declare_parameter("table.dimensions_topic", "/box/dimensions");
      const double wait_timeout =
          node->declare_parameter("table.pose_wait_timeout", 0.0);
      const double maximum_message_delta =
          node->declare_parameter("table.max_message_time_delta", 0.2);
      const auto visual = wait_for_visual_box(
          node, planning_frame, pose_topic, dimensions_topic, wait_timeout,
          maximum_message_delta);
      physical_pose = visual.pose;
      physical_size = visual.dimensions;
    } else if (source == "fixed_fake_only") {
      // This mode exists only for the isolated fake-hardware smoke test. The real
      // pick.yaml explicitly selects vision and has no fixed pose fallback.
      physical_size = node->declare_parameter<std::vector<double>>(
          "table.size", {0.0, 0.0, 0.0});
      const auto position = node->declare_parameter<std::vector<double>>(
          "table.position", {0.0, 0.0, 0.0});
      const double yaw = node->declare_parameter("table.yaw", 0.0);
      physical_pose = yaw_pose(position, yaw);
      RCLCPP_WARN(node->get_logger(),
                  "Using fixed_fake_only scene geometry; never use this config on hardware");
    } else {
      throw std::invalid_argument(
          "table.source must be 'vision' or 'fixed_fake_only'");
    }

    const double padding_xy = node->declare_parameter("table.padding_xy", 0.0);
    const double padding_top = node->declare_parameter("table.padding_top", 0.0);
    if (!is_finite(padding_xy) || !is_finite(padding_top) || padding_xy < 0.0 ||
        padding_top < 0.0) {
      throw std::invalid_argument("collision padding must be finite and non-negative");
    }
    const std::vector<double> padded_size{
        physical_size[0] + 2.0 * padding_xy,
        physical_size[1] + 2.0 * padding_xy,
        physical_size[2] + padding_top,
    };
    auto padded_pose = physical_pose;
    // Add Z padding only above the detected box; its physical bottom is preserved.
    padded_pose.position.z += 0.5 * padding_top;
    const auto table =
        make_box(table_id, planning_frame, padded_size, padded_pose);
    moveit::planning_interface::PlanningSceneInterface planning_scene;
    if (!planning_scene.applyCollisionObject(table)) {
      RCLCPP_ERROR(node->get_logger(), "MoveIt rejected the table collision object");
      rclcpp::shutdown();
      return 3;
    }

    const auto& q = physical_pose.orientation;
    const double yaw = std::atan2(2.0 * (q.w * q.z + q.x * q.y),
                                  1.0 - 2.0 * (q.y * q.y + q.z * q.z));
    RCLCPP_INFO(node->get_logger(),
                "Table physical center=[%.3f, %.3f, %.3f] m, "
                "size=[%.3f, %.3f, %.3f] m, yaw=%.1f deg, top_z=%.3f m",
                physical_pose.position.x, physical_pose.position.y,
                physical_pose.position.z, physical_size[0], physical_size[1],
                physical_size[2], yaw * 180.0 / std::acos(-1.0),
                physical_pose.position.z + 0.5 * physical_size[2]);
    RCLCPP_INFO(node->get_logger(),
                "Applied padded collision object '%s': center=[%.3f, %.3f, %.3f] "
                "m, size=[%.3f, %.3f, %.3f] m in frame '%s'",
                table_id.c_str(), padded_pose.position.x, padded_pose.position.y,
                padded_pose.position.z, padded_size[0], padded_size[1],
                padded_size[2], planning_frame.c_str());
  } catch (const std::exception& error) {
    RCLCPP_ERROR(node->get_logger(), "Scene update refused: %s", error.what());
    rclcpp::shutdown();
    return 4;
  }

  rclcpp::shutdown();
  return 0;
}
