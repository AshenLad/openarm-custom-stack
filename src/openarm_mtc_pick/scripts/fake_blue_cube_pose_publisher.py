#!/usr/bin/env python3

"""Publish an explicitly fake blue-cube pose for isolated MTC simulation."""

import math

from geometry_msgs.msg import PoseStamped, Vector3Stamped
import rclpy
from rclpy.node import Node


class FakeBlueCubePosePublisher(Node):
    def __init__(self) -> None:
        super().__init__("fake_blue_cube_pose_publisher")
        topic = str(self.declare_parameter("topic", "/blue_cube/pose").value)
        dimensions_topic = str(
            self.declare_parameter(
                "dimensions_topic", "/blue_cube/dimensions"
            ).value
        )
        self.frame_id = str(self.declare_parameter("frame_id", "world").value)
        self.position = [
            float(value)
            for value in self.declare_parameter(
                "position", [0.404, -0.001, 0.367]
            ).value
        ]
        self.yaw = float(self.declare_parameter("yaw", 0.0).value)
        self.size = [
            float(value)
            for value in self.declare_parameter("size", [0.04, 0.04, 0.04]).value
        ]
        rate_hz = float(self.declare_parameter("rate_hz", 10.0).value)
        if len(self.position) != 3 or not all(math.isfinite(v) for v in self.position):
            raise ValueError("position must contain three finite values")
        if not math.isfinite(self.yaw):
            raise ValueError("yaw must be finite")
        if len(self.size) != 3 or not all(
            math.isfinite(value) and value > 0.0 for value in self.size
        ):
            raise ValueError("size must contain three finite positive values")
        if not math.isfinite(rate_hz) or rate_hz <= 0.0:
            raise ValueError("rate_hz must be finite and positive")
        self.publisher = self.create_publisher(PoseStamped, topic, 10)
        self.dimensions_publisher = self.create_publisher(
            Vector3Stamped, dimensions_topic, 10
        )
        self.timer = self.create_timer(1.0 / rate_hz, self.publish_pose)
        self.get_logger().warning(
            f"Publishing FAKE object pose on {topic}, dimensions on "
            f"{dimensions_topic}, size={self.size}, yaw "
            f"{math.degrees(self.yaw):.1f} deg; "
            "never run this node with real hardware"
        )

    def publish_pose(self) -> None:
        message = PoseStamped()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = self.frame_id
        message.pose.position.x = self.position[0]
        message.pose.position.y = self.position[1]
        message.pose.position.z = self.position[2]
        message.pose.orientation.z = math.sin(0.5 * self.yaw)
        message.pose.orientation.w = math.cos(0.5 * self.yaw)
        self.publisher.publish(message)
        dimensions = Vector3Stamped()
        dimensions.header = message.header
        dimensions.vector.x = self.size[0]
        dimensions.vector.y = self.size[1]
        dimensions.vector.z = self.size[2]
        self.dimensions_publisher.publish(dimensions)


def main() -> None:
    rclpy.init()
    node = FakeBlueCubePosePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
