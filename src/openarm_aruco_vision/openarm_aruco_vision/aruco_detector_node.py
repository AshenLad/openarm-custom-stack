from __future__ import annotations

import math
from typing import Optional

import cv2
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseStamped, TransformStamped
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import TransformBroadcaster


def rotation_matrix_to_quaternion(matrix: np.ndarray) -> tuple[float, float, float, float]:
    """Convert a proper 3x3 rotation matrix to an xyzw quaternion."""
    m = np.asarray(matrix, dtype=np.float64)
    trace = float(np.trace(m))
    if trace > 0.0:
        scale = math.sqrt(trace + 1.0) * 2.0
        w = 0.25 * scale
        x = (m[2, 1] - m[1, 2]) / scale
        y = (m[0, 2] - m[2, 0]) / scale
        z = (m[1, 0] - m[0, 1]) / scale
    else:
        index = int(np.argmax(np.diag(m)))
        if index == 0:
            scale = math.sqrt(1.0 + m[0, 0] - m[1, 1] - m[2, 2]) * 2.0
            w = (m[2, 1] - m[1, 2]) / scale
            x = 0.25 * scale
            y = (m[0, 1] + m[1, 0]) / scale
            z = (m[0, 2] + m[2, 0]) / scale
        elif index == 1:
            scale = math.sqrt(1.0 + m[1, 1] - m[0, 0] - m[2, 2]) * 2.0
            w = (m[0, 2] - m[2, 0]) / scale
            x = (m[0, 1] + m[1, 0]) / scale
            y = 0.25 * scale
            z = (m[1, 2] + m[2, 1]) / scale
        else:
            scale = math.sqrt(1.0 + m[2, 2] - m[0, 0] - m[1, 1]) * 2.0
            w = (m[1, 0] - m[0, 1]) / scale
            x = (m[0, 2] + m[2, 0]) / scale
            y = (m[1, 2] + m[2, 1]) / scale
            z = 0.25 * scale
    quaternion = np.array([x, y, z, w], dtype=np.float64)
    quaternion /= np.linalg.norm(quaternion)
    return tuple(float(value) for value in quaternion)


class ArucoDetector(Node):
    def __init__(self) -> None:
        super().__init__("aruco_detector")
        defaults = (
            ("image_topic", "/camera/global_camera/color/image_raw"),
            ("camera_info_topic", "/camera/global_camera/color/camera_info"),
            ("dictionary", "DICT_6X6_50"),
            ("marker_id", 0),
            ("marker_size_m", 0.080),
            ("marker_frame", "aruco_marker_1"),
            ("camera_frame", ""),
            ("publish_debug_image", True),
            ("debug_image_topic", "/aruco_marker_1/debug_image"),
            ("pose_topic", "/aruco_marker_1/pose"),
        )
        for name, default in defaults:
            self.declare_parameter(name, default)

        dictionary_name = str(self.get_parameter("dictionary").value)
        if not hasattr(cv2.aruco, dictionary_name):
            raise ValueError(f"Unsupported ArUco dictionary: {dictionary_name}")
        self.dictionary_name = dictionary_name
        dictionary_id = getattr(cv2.aruco, dictionary_name)
        self.dictionary = cv2.aruco.getPredefinedDictionary(dictionary_id)
        # Ubuntu 24.04 on Jetson currently provides OpenCV 4.6.  Prefer its
        # mature procedural API: some ARM64 builds expose the newer wrapper
        # classes but crash while constructing them.
        if hasattr(cv2.aruco, "DetectorParameters_create"):
            self.detector_parameters = cv2.aruco.DetectorParameters_create()
        else:
            self.detector_parameters = cv2.aruco.DetectorParameters()
        self.detector_parameters.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX

        self.marker_id = int(self.get_parameter("marker_id").value)
        self.marker_size = float(self.get_parameter("marker_size_m").value)
        if self.marker_size <= 0.0:
            raise ValueError("marker_size_m must be positive")
        self.marker_frame = str(self.get_parameter("marker_frame").value)
        self.camera_frame_override = str(self.get_parameter("camera_frame").value)
        self.publish_debug = bool(self.get_parameter("publish_debug_image").value)
        self.bridge = CvBridge()
        self.camera_matrix: Optional[np.ndarray] = None
        self.distortion: Optional[np.ndarray] = None
        self.camera_frame = ""
        self.tf_broadcaster = TransformBroadcaster(self)
        self.pose_publisher = self.create_publisher(
            PoseStamped, str(self.get_parameter("pose_topic").value), 10
        )
        self.debug_publisher = self.create_publisher(
            Image,
            str(self.get_parameter("debug_image_topic").value),
            qos_profile_sensor_data,
        )

        self.create_subscription(
            CameraInfo,
            str(self.get_parameter("camera_info_topic").value),
            self.camera_info_callback,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Image,
            str(self.get_parameter("image_topic").value),
            self.image_callback,
            qos_profile_sensor_data,
        )
        self.reported_detection = False
        self.last_id_report_ns = 0
        self.get_logger().info(
            f"Waiting for marker ID {self.marker_id} ({dictionary_name}, "
            f"size={self.marker_size:.4f} m)"
        )

    def camera_info_callback(self, message: CameraInfo) -> None:
        matrix = np.asarray(message.k, dtype=np.float64).reshape(3, 3)
        if matrix[0, 0] <= 0.0 or matrix[1, 1] <= 0.0:
            return
        self.camera_matrix = matrix
        self.distortion = np.asarray(message.d, dtype=np.float64)
        self.camera_frame = self.camera_frame_override or message.header.frame_id

    def detect(self, gray: np.ndarray):
        return cv2.aruco.detectMarkers(
            gray, self.dictionary, parameters=self.detector_parameters
        )

    def image_callback(self, message: Image) -> None:
        if self.camera_matrix is None or self.distortion is None:
            return
        try:
            frame = self.bridge.imgmsg_to_cv2(message, desired_encoding="bgr8")
        except Exception as error:
            self.get_logger().error(f"cv_bridge conversion failed: {error}")
            return
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = self.detect(gray)
        debug_frame = frame.copy() if self.publish_debug else None
        if ids is not None and len(ids):
            flat_ids = ids.reshape(-1)
            now_ns = self.get_clock().now().nanoseconds
            if now_ns - self.last_id_report_ns >= 2_000_000_000:
                detected_ids = sorted({int(value) for value in flat_ids})
                self.get_logger().info(
                    f"Dictionary {self.dictionary_name} detected marker ID(s): "
                    f"{detected_ids}"
                )
                self.last_id_report_ns = now_ns
            if debug_frame is not None:
                cv2.aruco.drawDetectedMarkers(debug_frame, corners, ids)
            matches = np.flatnonzero(flat_ids == self.marker_id)
            if len(matches):
                half_size = self.marker_size * 0.5
                object_points = np.asarray(
                    [
                        [-half_size, half_size, 0.0],
                        [half_size, half_size, 0.0],
                        [half_size, -half_size, 0.0],
                        [-half_size, -half_size, 0.0],
                    ],
                    dtype=np.float32,
                )
                image_points = np.asarray(
                    corners[int(matches[0])], dtype=np.float32
                ).reshape(4, 2)
                solved, rvec, tvec = cv2.solvePnP(
                    object_points,
                    image_points,
                    self.camera_matrix,
                    self.distortion,
                    flags=cv2.SOLVEPNP_IPPE_SQUARE,
                )
                if not solved:
                    return
                rvec = np.asarray(rvec, dtype=np.float64).reshape(3)
                tvec = np.asarray(tvec, dtype=np.float64).reshape(3)
                rotation, _ = cv2.Rodrigues(rvec)
                quaternion = rotation_matrix_to_quaternion(rotation)
                parent_frame = self.camera_frame_override or self.camera_frame or message.header.frame_id
                if parent_frame:
                    self.publish_pose(message, parent_frame, tvec, quaternion)
                    if not self.reported_detection:
                        self.get_logger().info(
                            f"Detected marker {self.marker_id}; publishing "
                            f"{parent_frame} -> {self.marker_frame}"
                        )
                        self.reported_detection = True
                if debug_frame is not None:
                    cv2.drawFrameAxes(
                        debug_frame,
                        self.camera_matrix,
                        self.distortion,
                        rvec,
                        tvec,
                        self.marker_size * 0.5,
                    )
        if debug_frame is not None:
            debug_message = self.bridge.cv2_to_imgmsg(debug_frame, encoding="bgr8")
            debug_message.header = message.header
            self.debug_publisher.publish(debug_message)

    def publish_pose(
        self,
        image: Image,
        parent_frame: str,
        translation: np.ndarray,
        quaternion: tuple[float, float, float, float],
    ) -> None:
        pose = PoseStamped()
        pose.header.stamp = image.header.stamp
        pose.header.frame_id = parent_frame
        pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = (
            float(value) for value in translation
        )
        pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w = quaternion
        self.pose_publisher.publish(pose)

        transform = TransformStamped()
        transform.header = pose.header
        transform.child_frame_id = self.marker_frame
        transform.transform.translation.x = pose.pose.position.x
        transform.transform.translation.y = pose.pose.position.y
        transform.transform.translation.z = pose.pose.position.z
        transform.transform.rotation = pose.pose.orientation
        self.tf_broadcaster.sendTransform(transform)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ArucoDetector()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
