from __future__ import annotations

from collections import deque
from dataclasses import dataclass
import math
from typing import Optional

import cv2
from cv_bridge import CvBridge
from geometry_msgs.msg import (
    Point32,
    PolygonStamped,
    PoseStamped,
    TransformStamped,
    Vector3Stamped,
)
import numpy as np
import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Float64
from tf2_ros import Buffer, TransformBroadcaster, TransformException, TransformListener

from openarm_aruco_vision.blue_cube_detector_node import (
    deproject_pixels,
    quaternion_to_rotation_matrix,
)
from openarm_aruco_vision.obb import min_area_box_top


def normalize_axis_yaw(yaw: float) -> float:
    """Normalize a rectangle-axis angle to [-pi/2, pi/2)."""
    return (yaw + 0.5 * math.pi) % math.pi - 0.5 * math.pi


def axis_angle_error(angle: float, reference: float) -> float:
    """Smallest signed angle between two unoriented rectangle axes."""
    return 0.5 * math.atan2(
        math.sin(2.0 * (angle - reference)),
        math.cos(2.0 * (angle - reference)),
    )


def mean_axis_yaw(angles: np.ndarray) -> float:
    """Circular mean for an axis whose yaw is equivalent modulo pi."""
    return normalize_axis_yaw(
        0.5
        * math.atan2(
            float(np.mean(np.sin(2.0 * angles))),
            float(np.mean(np.cos(2.0 * angles))),
        )
    )


def measurement_jumped(
    sample: np.ndarray,
    previous: np.ndarray,
    position_threshold_m: float,
    yaw_threshold_rad: float,
) -> bool:
    """Return whether a new measurement invalidates the current LOCK window."""
    return bool(
        np.linalg.norm(sample[:3] - previous[:3]) > position_threshold_m
        or abs(axis_angle_error(sample[3], previous[3])) > yaw_threshold_rad
    )


def defer_locked_jump(
    was_locked: bool, consecutive_jumps: int, confirmation_frames: int
) -> bool:
    """Keep the last LOCK until a pose jump persists for several frames."""
    return bool(was_locked and consecutive_jumps < confirmation_frames)


def locked_box_measurement_changed(
    sample: np.ndarray,
    locked: np.ndarray,
    position_threshold_m: float,
    size_threshold_m: float,
    yaw_threshold_rad: float,
) -> bool:
    """Return whether a live box measurement invalidates a committed snapshot.

    A box sample is [center_x, center_y, top_z, yaw, size_x, size_y,
    support_z, height].  Compare every field consumed by PlanningScene, while
    keeping rectangle yaw modulo pi.  The caller confirms this result across
    several frames before dropping the transaction lock.
    """
    position_indices = (0, 1, 2, 6)
    size_indices = (4, 5, 7)
    return bool(
        any(
            abs(float(sample[index] - locked[index])) > position_threshold_m
            for index in position_indices
        )
        or any(
            abs(float(sample[index] - locked[index])) > size_threshold_m
            for index in size_indices
        )
        or abs(axis_angle_error(float(sample[3]), float(locked[3])))
        > yaw_threshold_rad
    )


def fit_horizontal_plane_ransac(
    points: np.ndarray,
    iterations: int,
    distance_threshold: float,
    max_tilt_rad: float,
    rng: np.random.Generator,
) -> Optional[np.ndarray]:
    """Return inlier mask for the largest approximately horizontal plane."""
    if points.ndim != 2 or points.shape[1] != 3 or points.shape[0] < 3:
        return None
    minimum_vertical_normal = math.cos(max_tilt_rad)
    best_mask: Optional[np.ndarray] = None
    best_count = 0

    for _ in range(iterations):
        indices = rng.choice(points.shape[0], size=3, replace=False)
        first, second, third = points[indices]
        normal = np.cross(second - first, third - first)
        norm = float(np.linalg.norm(normal))
        if norm < 1.0e-9:
            continue
        normal /= norm
        if abs(float(normal[2])) < minimum_vertical_normal:
            continue
        distances = np.abs((points - first) @ normal)
        mask = distances <= distance_threshold
        count = int(np.count_nonzero(mask))
        if count > best_count:
            best_count = count
            best_mask = mask

    if best_mask is None or best_count < 3:
        return None

    # Refine the model using all initial inliers, then classify once more.
    inlier_points = points[best_mask]
    centroid = np.mean(inlier_points, axis=0)
    _, _, vectors = np.linalg.svd(inlier_points - centroid, full_matrices=False)
    normal = vectors[-1]
    if abs(float(normal[2])) < minimum_vertical_normal:
        return None
    return np.abs((points - centroid) @ normal) <= distance_threshold


@dataclass
class SupportPlane:
    """Dominant horizontal support surface of the current frame.

    Stored as ``normal . x + d = 0`` with the unit normal pointing along
    world +Z, so ``height = normal . x + d`` is the signed elevation above
    the support surface.
    """

    normal: np.ndarray
    d: float
    inlier_count: int

    def z_at(self, center_xy: np.ndarray) -> float:
        """Support-plane Z evaluated at a world XY position."""
        return -(
            float(self.normal[0] * center_xy[0])
            + float(self.normal[1] * center_xy[1])
            + self.d
        ) / float(self.normal[2])


def estimate_support_plane_ransac(
    world_points: np.ndarray,
    iterations: int,
    distance_threshold: float,
    max_tilt_rad: float,
    minimum_points: int,
    maximum_points: int,
    rng: np.random.Generator,
) -> Optional[SupportPlane]:
    """Find the dominant horizontal support plane in a world point cloud.

    Runs once per frame over the whole valid-depth cloud (subsampled). The
    plane normal is enforced to be near world +Z so walls, box sides and the
    robot arm are never mistaken for the table.
    """
    if world_points.shape[0] < minimum_points:
        return None
    if world_points.shape[0] > maximum_points:
        selected = rng.choice(world_points.shape[0], maximum_points, replace=False)
        world_points = world_points[selected]
    mask = fit_horizontal_plane_ransac(
        world_points,
        iterations,
        distance_threshold,
        max_tilt_rad,
        rng,
    )
    if mask is None or np.count_nonzero(mask) < minimum_points:
        return None
    inliers = world_points[mask]
    centroid = np.mean(inliers, axis=0)
    _, _, vectors = np.linalg.svd(inliers - centroid, full_matrices=False)
    normal = vectors[-1]
    if abs(float(normal[2])) < math.cos(max_tilt_rad):
        return None
    if normal[2] < 0.0:
        normal = -normal
    d = -float(normal @ centroid)
    return SupportPlane(
        normal=normal,
        d=d,
        inlier_count=int(np.count_nonzero(mask)),
    )


def height_above_support(
    world_points: np.ndarray, normal: np.ndarray, d: float
) -> np.ndarray:
    """Signed elevation of world points above the support plane (positive up)."""
    return world_points @ normal + d


@dataclass
class ColoredSurface:
    mask: np.ndarray
    contour: np.ndarray
    pixel_area: int
    touches_border: bool


def build_white_mask(
    frame: np.ndarray,
    hsv_s_max: int,
    hsv_v_min: int,
    rgb_min: int,
    close_kernel_size: int,
    open_kernel_size: int,
) -> np.ndarray:
    """Return the uint8 white-surface mask of an RGB frame.

    Low saturation + high value plus an optional RGB floor, followed by the
    same close/open morphology previously applied before candidate
    extraction. No connected components are computed here; the mask is later
    intersected with the depth-derived elevation mask.
    """
    if frame.ndim != 3 or frame.shape[2] != 3:
        raise ValueError("color image is not BGR")
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(
        hsv,
        np.asarray([0, 0, hsv_v_min], dtype=np.uint8),
        np.asarray([179, hsv_s_max, 255], dtype=np.uint8),
    )
    if rgb_min > 0:
        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        mask &= cv2.inRange(
            rgb,
            (rgb_min, rgb_min, rgb_min),
            (255, 255, 255),
        )

    def odd_kernel(size: int) -> np.ndarray:
        size = max(1, int(size))
        if size % 2 == 0:
            size += 1
        return np.ones((size, size), dtype=np.uint8)

    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, odd_kernel(close_kernel_size))
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, odd_kernel(open_kernel_size))
    return mask


def extract_candidates(
    mask: np.ndarray,
    minimum_pixels: int,
    border_margin_px: int,
) -> list[ColoredSurface]:
    """Split a candidate mask into connected components.

    Each component keeps its filled external contour, so holes caused by the
    blue cube or specular tape are recovered (the footprint is restored, not
    required to be 100 % white). A component touching the image border is
    flagged for REJECT_BORDER instead of discarding the whole frame.
    """
    count, labels, stats, _ = cv2.connectedComponentsWithStats(mask, 8)
    if count <= 1:
        return []
    image_height, image_width = mask.shape
    margin = max(0, int(border_margin_px))
    candidates: list[ColoredSurface] = []
    for component_index in range(1, count):
        pixel_area = int(stats[component_index, cv2.CC_STAT_AREA])
        if pixel_area < minimum_pixels:
            continue
        component = np.where(labels == component_index, 255, 0).astype(np.uint8)
        contours, _ = cv2.findContours(
            component, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
        )
        if not contours:
            continue
        contour = max(contours, key=cv2.contourArea)
        x, y, width, height = cv2.boundingRect(contour)
        touches_border = (
            x <= margin
            or y <= margin
            or x + width >= image_width - margin
            or y + height >= image_height - margin
        )
        filled = np.zeros_like(mask)
        cv2.drawContours(filled, [contour], -1, 255, thickness=cv2.FILLED)
        candidates.append(
            ColoredSurface(
                mask=filled,
                contour=contour,
                pixel_area=pixel_area,
                touches_border=touches_border,
            )
        )
    candidates.sort(key=lambda surface: surface.pixel_area, reverse=True)
    return candidates


def choose_largest_valid_top(
    measurements: list[tuple[ColoredSurface, np.ndarray]],
) -> tuple[ColoredSurface, np.ndarray]:
    """Tie-break by the largest 3-D top footprint.

    The support surface and the thin white debris were already removed before
    this point, so every candidate here is a real elevated white top. Only the
    3-D footprint is compared, no combined scoring is used.
    """
    return max(measurements, key=lambda entry: entry[1][4] * entry[1][5])


def rectangle_corners(
    center_xy: np.ndarray, dimensions_xy: np.ndarray, yaw: float
) -> np.ndarray:
    half_length, half_width = 0.5 * dimensions_xy
    local = np.asarray(
        [
            [-half_length, -half_width],
            [half_length, -half_width],
            [half_length, half_width],
            [-half_length, half_width],
        ],
        dtype=np.float64,
    )
    cosine, sine = math.cos(yaw), math.sin(yaw)
    rotation = np.asarray([[cosine, -sine], [sine, cosine]])
    return local @ rotation.T + center_xy


class BoxDetector(Node):
    def __init__(self) -> None:
        super().__init__("box_detector")
        defaults = (
            ("color_image_topic", "/camera/global_camera/color/image_raw"),
            (
                "aligned_depth_topic",
                "/camera/global_camera/aligned_depth_to_color/image_raw",
            ),
            ("camera_info_topic", "/camera/global_camera/color/camera_info"),
            ("pose_topic", "/box/pose"),
            ("dimensions_topic", "/box/dimensions"),
            ("detected_dimensions_topic", "/box/detected_dimensions"),
            ("top_height_topic", "/box/top_height"),
            ("support_height_topic", "/box/support_height"),
            ("footprint_topic", "/box/footprint"),
            ("debug_image_topic", "/box/debug_image"),
            ("world_frame", "world"),
            ("box_frame", "detected_box"),
            ("surface_white_hsv_s_max", 90),
            ("surface_white_hsv_v_min", 170),
            ("surface_white_rgb_min", 150),
            ("surface_close_kernel", 11),
            ("surface_open_kernel", 3),
            ("surface_min_pixels", 2000),
            ("surface_border_margin_px", 8),
            # Minimum fraction of a candidate's depth points that must lie on
            # one horizontal plane before it is measured as a box top. This is
            # a quality gate, not a ranking score.
            ("surface_min_plane_inlier_ratio", 0.55),
            ("depth_scale", 0.001),
            ("min_depth_m", 0.10),
            ("max_depth_m", 2.0),
            ("point_stride", 4),
            ("max_cloud_points", 15000),
            ("processing_interval_sec", 0.20),
            ("max_rgb_depth_time_delta_sec", 0.20),
            ("ransac_iterations", 80),
            ("plane_distance_threshold_m", 0.008),
            ("max_plane_tilt_deg", 15.0),
            ("min_plane_points", 300),
            # Global per-frame support-plane search (one estimation per frame,
            # shared by every candidate; no per-candidate support search).
            ("support_min_plane_points", 120),
            ("support_max_cloud_points", 10000),
            ("support_ransac_iterations", 60),
            ("stable_samples", 5),
            ("max_position_std_m", 0.008),
            ("max_top_height_std_m", 0.006),
            ("max_height_std_m", 0.008),
            ("max_dimension_std_m", 0.010),
            ("max_yaw_std_deg", 2.0),
            ("sample_jump_reset_m", 0.04),
            ("sample_yaw_jump_reset_deg", 12.0),
            ("locked_max_position_change_m", 0.008),
            ("locked_max_size_change_m", 0.015),
            ("locked_max_yaw_change_deg", 5.0),
            ("lock_change_confirm_frames", 3),
            ("max_missed_frames", 3),
            ("debug_max_width", 640),
            ("debug_max_height", 480),
        )
        for name, default in defaults:
            self.declare_parameter(name, default)

        value = lambda name: self.get_parameter(name).value
        self.color_topic = str(value("color_image_topic"))
        self.depth_topic = str(value("aligned_depth_topic"))
        self.info_topic = str(value("camera_info_topic"))
        self.world_frame = str(value("world_frame"))
        self.box_frame = str(value("box_frame"))
        self.surface_white_hsv_s_max = int(value("surface_white_hsv_s_max"))
        self.surface_white_hsv_v_min = int(value("surface_white_hsv_v_min"))
        self.surface_white_rgb_min = int(value("surface_white_rgb_min"))
        self.surface_close_kernel = int(value("surface_close_kernel"))
        self.surface_open_kernel = int(value("surface_open_kernel"))
        self.surface_min_pixels = int(value("surface_min_pixels"))
        self.surface_border_margin_px = int(value("surface_border_margin_px"))
        self.surface_min_plane_inlier_ratio = float(
            value("surface_min_plane_inlier_ratio")
        )
        self.depth_scale = float(value("depth_scale"))
        self.min_depth = float(value("min_depth_m"))
        self.max_depth = float(value("max_depth_m"))
        self.point_stride = int(value("point_stride"))
        self.max_cloud_points = int(value("max_cloud_points"))
        self.processing_interval_ns = int(float(value("processing_interval_sec")) * 1e9)
        self.max_time_delta_ns = int(float(value("max_rgb_depth_time_delta_sec")) * 1e9)
        self.ransac_iterations = int(value("ransac_iterations"))
        self.plane_threshold = float(value("plane_distance_threshold_m"))
        self.max_plane_tilt_rad = math.radians(float(value("max_plane_tilt_deg")))
        self.min_plane_points = int(value("min_plane_points"))
        self.support_min_plane_points = int(value("support_min_plane_points"))
        self.support_max_cloud_points = int(value("support_max_cloud_points"))
        self.support_ransac_iterations = int(value("support_ransac_iterations"))
        self.stable_samples = int(value("stable_samples"))
        self.max_position_std = float(value("max_position_std_m"))
        self.max_top_height_std = float(value("max_top_height_std_m"))
        self.max_height_std = float(value("max_height_std_m"))
        self.max_dimension_std = float(value("max_dimension_std_m"))
        self.max_yaw_std = math.radians(float(value("max_yaw_std_deg")))
        self.sample_jump_reset = float(value("sample_jump_reset_m"))
        self.sample_yaw_jump_reset = math.radians(
            float(value("sample_yaw_jump_reset_deg"))
        )
        self.locked_max_position_change = float(
            value("locked_max_position_change_m")
        )
        self.locked_max_size_change = float(value("locked_max_size_change_m"))
        self.locked_max_yaw_change = math.radians(
            float(value("locked_max_yaw_change_deg"))
        )
        self.lock_change_confirm_frames = int(value("lock_change_confirm_frames"))
        self.max_missed_frames = int(value("max_missed_frames"))
        self.debug_max_width = int(value("debug_max_width"))
        self.debug_max_height = int(value("debug_max_height"))
        self._validate_parameters()

        self.bridge = CvBridge()
        self.camera_matrix: Optional[np.ndarray] = None
        self.camera_frame = ""
        self.depth_messages: deque[Image] = deque(maxlen=10)
        self.color_messages: deque[Image] = deque(maxlen=10)
        self.latest_rgbd_pair: Optional[tuple[Image, Image]] = None
        self.last_processed_color_stamp_ns = 0
        self.last_warning_ns: dict[str, int] = {}
        self.samples: deque[np.ndarray] = deque(maxlen=self.stable_samples)
        self.missed_frames = 0
        self.consecutive_jumps = 0
        self.was_locked = False
        self.locked_estimate: Optional[np.ndarray] = None
        self.rng = np.random.default_rng(20260817)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = TransformBroadcaster(self)
        sensor_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.input_callback_group = MutuallyExclusiveCallbackGroup()
        self.processing_callback_group = MutuallyExclusiveCallbackGroup()
        self.pose_publisher = self.create_publisher(PoseStamped, str(value("pose_topic")), 10)
        self.dimensions_publisher = self.create_publisher(
            Vector3Stamped, str(value("dimensions_topic")), 10
        )
        self.detected_dimensions_publisher = self.create_publisher(
            Vector3Stamped, str(value("detected_dimensions_topic")), 10
        )
        self.top_height_publisher = self.create_publisher(
            Float64, str(value("top_height_topic")), 10
        )
        self.support_height_publisher = self.create_publisher(
            Float64, str(value("support_height_topic")), 10
        )
        self.footprint_publisher = self.create_publisher(
            PolygonStamped, str(value("footprint_topic")), 10
        )
        self.debug_publisher = self.create_publisher(
            Image, str(value("debug_image_topic")), sensor_qos
        )
        self.create_subscription(
            CameraInfo,
            self.info_topic,
            self.camera_info_callback,
            sensor_qos,
            callback_group=self.input_callback_group,
        )
        self.create_subscription(
            Image,
            self.depth_topic,
            self.depth_callback,
            sensor_qos,
            callback_group=self.input_callback_group,
        )
        self.create_subscription(
            Image,
            self.color_topic,
            self.color_callback,
            sensor_qos,
            callback_group=self.input_callback_group,
        )
        self.create_timer(
            max(0.001, self.processing_interval_ns / 1.0e9),
            self.processing_callback,
            callback_group=self.processing_callback_group,
        )
        self.get_logger().info(
            "Depth-first box detector: one global support plane per frame, "
            "white tops are only accepted above the support surface"
        )

    def _validate_parameters(self) -> None:
        white_bounds = (
            self.surface_white_hsv_s_max,
            self.surface_white_hsv_v_min,
            self.surface_white_rgb_min,
        )
        if any(bound < 0 or bound > 255 for bound in white_bounds):
            raise ValueError("surface white HSV/RGB bounds must be within [0, 255]")
        if (
            self.surface_close_kernel < 1
            or self.surface_open_kernel < 1
            or self.surface_min_pixels < 1
            or self.surface_border_margin_px < 0
            or not 0.0 < self.surface_min_plane_inlier_ratio <= 1.0
        ):
            raise ValueError("white-surface segmentation parameters are invalid")
        if not 0.0 < self.min_depth < self.max_depth:
            raise ValueError("depth range is invalid")
        if self.depth_scale <= 0.0 or self.point_stride < 1 or self.max_cloud_points < 3:
            raise ValueError("depth sampling parameters are invalid")
        if self.processing_interval_ns < 0 or self.max_time_delta_ns <= 0:
            raise ValueError("processing/synchronization timing is invalid")
        if self.ransac_iterations < 1 or self.plane_threshold <= 0.0:
            raise ValueError("RANSAC parameters are invalid")
        if not 0.0 < self.max_plane_tilt_rad < 0.5 * math.pi:
            raise ValueError("max_plane_tilt_deg must be between 0 and 90")
        if self.min_plane_points < 3:
            raise ValueError("plane parameters are invalid")
        if (
            self.support_min_plane_points < 3
            or self.support_max_cloud_points < self.support_min_plane_points
            or self.support_ransac_iterations < 1
        ):
            raise ValueError("support-plane parameters are invalid")
        if self.stable_samples < 1:
            raise ValueError("stable_samples must be positive")
        if (
            self.locked_max_position_change <= 0.0
            or self.locked_max_size_change <= 0.0
            or self.locked_max_yaw_change <= 0.0
            or self.lock_change_confirm_frames < 1
        ):
            raise ValueError("box transaction-lock thresholds are invalid")
        if (
            self.max_position_std <= 0.0
            or self.max_top_height_std <= 0.0
            or self.max_height_std <= 0.0
            or self.max_dimension_std <= 0.0
            or self.max_yaw_std <= 0.0
            or self.sample_jump_reset <= 0.0
            or self.sample_yaw_jump_reset <= 0.0
            or self.max_missed_frames < 1
        ):
            raise ValueError("stability parameters are invalid")
        if self.debug_max_width < 1 or self.debug_max_height < 1:
            raise ValueError("debug image limits must be positive")

    @staticmethod
    def stamp_ns(message) -> int:
        return int(message.header.stamp.sec) * 1_000_000_000 + int(
            message.header.stamp.nanosec
        )

    def warn_throttled(self, key: str, message: str, interval_sec: float = 2.0) -> None:
        now = self.get_clock().now().nanoseconds
        if now - self.last_warning_ns.get(key, 0) >= int(interval_sec * 1e9):
            self.get_logger().warning(message)
            self.last_warning_ns[key] = now

    def camera_info_callback(self, message: CameraInfo) -> None:
        matrix = np.asarray(message.k, dtype=np.float64).reshape(3, 3)
        if matrix[0, 0] <= 0.0 or matrix[1, 1] <= 0.0:
            return
        self.camera_matrix = matrix
        self.camera_frame = message.header.frame_id

    def depth_callback(self, message: Image) -> None:
        """Buffer encoded depth briefly and form the newest synchronized pair."""
        self.depth_messages.append(message)
        self._match_rgbd_pair(message, self.color_messages, depth_is_new=True)
        if not self.camera_frame:
            self.camera_frame = message.header.frame_id

    def color_callback(self, message: Image) -> None:
        """Keep only the newest color frame; heavy work runs from a timer.

        This latest-frame pattern prevents a slow detector from accumulating
        stale color callbacks while the depth subscription advances, which
        otherwise creates artificial RGB/depth timestamp failures under CPU
        load.
        """
        self.color_messages.append(message)
        self._match_rgbd_pair(message, self.depth_messages, depth_is_new=False)

    def _match_rgbd_pair(
        self,
        message: Image,
        opposite_messages: deque[Image],
        depth_is_new: bool,
    ) -> None:
        """Update the latest approximate-time RGB-D pair from bounded queues."""
        if not opposite_messages:
            return
        stamp = self.stamp_ns(message)
        nearest = min(
            opposite_messages,
            key=lambda candidate: abs(self.stamp_ns(candidate) - stamp),
        )
        if abs(self.stamp_ns(nearest) - stamp) > self.max_time_delta_ns:
            return
        if depth_is_new:
            self.latest_rgbd_pair = (nearest, message)
        else:
            self.latest_rgbd_pair = (message, nearest)

    def processing_callback(self) -> None:
        pair = self.latest_rgbd_pair
        if pair is None:
            return
        message, depth_message = pair
        stamp_ns = self.stamp_ns(message)
        if stamp_ns == self.last_processed_color_stamp_ns:
            return
        self.last_processed_color_stamp_ns = stamp_ns
        # Snapshot both newest messages once so this detection uses one pair.
        camera_matrix = self.camera_matrix
        camera_frame = self.camera_frame or message.header.frame_id
        if camera_matrix is None:
            return
        depth_stamp_ns = self.stamp_ns(depth_message)
        if abs(stamp_ns - depth_stamp_ns) > self.max_time_delta_ns:
            self.warn_throttled("sync", "RGB and aligned-depth timestamps are too far apart")
            return
        try:
            frame = self.bridge.imgmsg_to_cv2(message, desired_encoding="bgr8")
        except Exception as error:
            self.warn_throttled("color_conversion", f"color conversion failed: {error}")
            return
        try:
            depth = np.asarray(
                self.bridge.imgmsg_to_cv2(
                    depth_message, desired_encoding="passthrough"
                )
            )
        except Exception as error:
            self.warn_throttled("depth_conversion", f"depth conversion failed: {error}")
            return
        if depth.ndim != 2:
            return
        if depth_message.encoding == "16UC1":
            depth = depth.astype(np.float32) * self.depth_scale
        else:
            depth = depth.astype(np.float32)
        debug = frame.copy()
        if frame.shape[:2] != depth.shape:
            self.warn_throttled(
                "shape",
                "aligned depth dimensions do not match RGB; enable align_depth",
            )
            self.publish_debug(message, debug, "depth/RGB size mismatch", False)
            return

        # Step 1: white surface mask (reused thresholding; no candidates yet).
        white_mask = build_white_mask(
            frame,
            self.surface_white_hsv_s_max,
            self.surface_white_hsv_v_min,
            self.surface_white_rgb_min,
            self.surface_close_kernel,
            self.surface_open_kernel,
        )
        if not np.any(white_mask):
            self.reject(message, debug, "no white surface")
            return

        try:
            transform = self.tf_buffer.lookup_transform(
                self.world_frame, camera_frame, Time()
            )
        except TransformException as error:
            self.warn_throttled(
                "tf", f"Waiting for TF {self.world_frame} <- {camera_frame}: {error}"
            )
            self.publish_debug(message, debug, "waiting for world<-camera TF", False)
            return
        rotation_message = transform.transform.rotation
        rotation = quaternion_to_rotation_matrix(
            rotation_message.x,
            rotation_message.y,
            rotation_message.z,
            rotation_message.w,
        )
        translation_message = transform.transform.translation
        translation = np.asarray(
            [translation_message.x, translation_message.y, translation_message.z],
            dtype=np.float64,
        )

        # Step 2: deproject the subsampled valid depth into world coordinates.
        rows_grid, columns_grid = np.mgrid[
            0 : depth.shape[0] : self.point_stride,
            0 : depth.shape[1] : self.point_stride,
        ]
        all_rows = rows_grid.ravel()
        all_columns = columns_grid.ravel()
        depths = depth[all_rows, all_columns].astype(np.float64)
        valid_depth = (
            np.isfinite(depths)
            & (depths >= self.min_depth)
            & (depths <= self.max_depth)
        )
        if np.count_nonzero(valid_depth) < self.min_plane_points:
            self.reject(message, debug, "not enough valid depth points")
            return
        camera_points = deproject_pixels(
            all_rows[valid_depth],
            all_columns[valid_depth],
            depths[valid_depth],
            camera_matrix,
        )
        all_world_points = np.full(
            (all_rows.size, 3), np.nan, dtype=np.float64
        )
        all_world_points[valid_depth] = camera_points @ rotation.T + translation

        # Step 3: one global support plane per frame, shared by all candidates.
        support_plane = estimate_support_plane_ransac(
            all_world_points[valid_depth],
            self.support_ransac_iterations,
            self.plane_threshold,
            self.max_plane_tilt_rad,
            self.support_min_plane_points,
            self.support_max_cloud_points,
            self.rng,
        )
        if support_plane is None:
            # Depth-first: without a support surface this frame is rejected.
            # There is deliberately no fallback to the old RGB-only detector.
            self.reject(message, debug, "no horizontal support plane in view")
            return

        # Step 4: keep only depth that is clearly above the support surface.
        min_elevation = max(3.0 * self.plane_threshold, 0.02)
        heights = height_above_support(
            all_world_points, support_plane.normal, support_plane.d
        )
        elevated_valid = np.isfinite(heights) & (heights >= min_elevation)
        grid_height, grid_width = rows_grid.shape
        elevated_small = np.zeros((grid_height, grid_width), dtype=np.uint8)
        elevated_small.ravel()[elevated_valid] = 255
        elevated_mask = cv2.resize(
            elevated_small,
            (depth.shape[1], depth.shape[0]),
            interpolation=cv2.INTER_NEAREST,
        )

        # Step 5: white AND elevated. Ground, tape and flat white debris are
        # removed by elevation before any candidate is proposed.
        candidate_mask = cv2.bitwise_and(white_mask, elevated_mask)
        if not np.any(candidate_mask):
            self.reject(message, debug, "no white surface above support")
            return
        candidates = extract_candidates(
            candidate_mask, self.surface_min_pixels, self.surface_border_margin_px
        )
        if not candidates:
            self.reject(message, debug, "white-elevated candidates too small")
            return

        # Debug overlay: support (drawn as its inlier points) and elevation.
        support_distances = np.abs(
            all_world_points @ support_plane.normal + support_plane.d
        )
        support_inliers = np.isfinite(support_distances) & (
            support_distances <= self.plane_threshold
        )
        self.draw_sample_pixels(
            debug,
            all_rows[support_inliers],
            all_columns[support_inliers],
            color=(255, 220, 0),
            alpha=0.22,
        )
        self.draw_surface_mask(debug, elevated_mask, color=(0, 255, 0), alpha=0.16)

        # Step 6: measure every candidate top and keep the valid ones.
        border_rejected = 0
        geometry_rejected = 0
        valid_measurements: list[tuple[ColoredSurface, np.ndarray]] = []
        for index, surface in enumerate(candidates):
            if surface.touches_border:
                border_rejected += 1
                self.get_logger().debug(
                    f"White candidate {index} rejected: touches image border"
                )
                self._annotate_candidate(
                    debug, surface, index, "REJECT_BORDER", (0, 0, 255)
                )
                continue
            sample, error = self._measure_top(
                surface, all_rows, all_columns, valid_depth, all_world_points,
                support_plane,
            )
            if sample is None:
                geometry_rejected += 1
                self.get_logger().debug(
                    f"White candidate {index} rejected: {error}"
                )
                self._annotate_candidate(
                    debug, surface, index, "REJECT_PLANE", (255, 0, 255)
                )
                continue
            self._annotate_candidate(
                debug, surface, index, "CANDIDATE VALID", (0, 255, 0)
            )
            valid_measurements.append((surface, sample))

        if not valid_measurements:
            self.reject(
                message,
                debug,
                f"No valid white top candidate "
                f"(total={len(candidates)}, border={border_rejected}, "
                f"plane={geometry_rejected})",
            )
            return

        # Step 7: simple selection - largest 3-D top footprint.
        surface, sample = choose_largest_valid_top(valid_measurements)
        self.draw_surface_mask(debug, surface.mask, color=(0, 0, 255), alpha=0.30)
        cv2.polylines(
            debug, [surface.contour], True, (0, 255, 255), 4, cv2.LINE_AA
        )
        x, y, _, _ = cv2.boundingRect(surface.contour)
        cv2.putText(
            debug,
            "SELECTED BOX TOP",
            (max(8, x), max(16, y - 10)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            (0, 255, 255),
            2,
            cv2.LINE_AA,
        )
        self.missed_frames = 0
        if self.was_locked and self.locked_estimate is not None:
            if locked_box_measurement_changed(
                sample,
                self.locked_estimate,
                self.locked_max_position_change,
                self.locked_max_size_change,
                self.locked_max_yaw_change,
            ):
                self.consecutive_jumps += 1
                if defer_locked_jump(
                    True,
                    self.consecutive_jumps,
                    self.lock_change_confirm_frames,
                ):
                    # Keep publishing the exact geometry already committed to
                    # PlanningScene. Do not let a sequence of individually
                    # small rolling-median changes walk the lock over time.
                    sample = self.locked_estimate.copy()
                else:
                    self.samples.clear()
                    self.was_locked = False
                    self.locked_estimate = None
                    self.consecutive_jumps = 0
            else:
                self.consecutive_jumps = 0
                # A transaction lock is immutable. Fresh camera timestamps
                # are published with this same geometry until a real movement
                # is confirmed across multiple frames.
                sample = self.locked_estimate.copy()
        elif self.samples:
            previous = self.stable_estimate(np.asarray(self.samples))
            if measurement_jumped(
                sample,
                previous,
                self.sample_jump_reset,
                self.sample_yaw_jump_reset,
            ):
                self.consecutive_jumps += 1
                if defer_locked_jump(
                    self.was_locked,
                    self.consecutive_jumps,
                    self.lock_change_confirm_frames,
                ):
                    # Isolated partial-footprint measurements are ignored; the
                    # last stable window remains publishable. A real move is
                    # accepted after the configured number of confirmations.
                    sample = previous
                else:
                    self.samples.clear()
                    self.was_locked = False
                    self.locked_estimate = None
                    self.consecutive_jumps = 0
            else:
                self.consecutive_jumps = 0
        self.samples.append(sample)
        sample_array = np.asarray(self.samples)
        estimate = self.stable_estimate(sample_array)
        yaw_errors = np.asarray(
            [axis_angle_error(value, estimate[3]) for value in sample_array[:, 3]]
        )
        stable = (
            len(self.samples) == self.samples.maxlen
            and np.all(np.std(sample_array[:, :2], axis=0) <= self.max_position_std)
            and float(np.std(sample_array[:, 2])) <= self.max_top_height_std
            and float(np.std(yaw_errors)) <= self.max_yaw_std
            and np.all(np.std(sample_array[:, 4:6], axis=0) <= self.max_dimension_std)
            and float(np.std(sample_array[:, 7])) <= self.max_height_std
        )
        if stable:
            first_lock = not self.was_locked or self.locked_estimate is None
            if first_lock:
                self.locked_estimate = estimate.copy()
            self.publish_detection(message, self.locked_estimate)
            if first_lock:
                display_dimensions = self.locked_estimate[4:6]
                self.get_logger().info(
                    f"Box LOCK: center xyz=[{self.locked_estimate[0]:.3f}, "
                    f"{self.locked_estimate[1]:.3f}, "
                    f"{0.5 * (self.locked_estimate[2] + self.locked_estimate[6]):.3f}] m, "
                    f"physical size xyz=[{display_dimensions[0]:.3f}, "
                    f"{display_dimensions[1]:.3f}, "
                    f"{self.locked_estimate[7]:.3f}] m, "
                    f"yaw={math.degrees(self.locked_estimate[3]):.1f} deg, "
                    f"top_z={self.locked_estimate[2]:.3f} m, "
                    f"support_z={self.locked_estimate[6]:.3f} m, "
                    "source=depth_first_white_top"
                )
            self.was_locked = True
        elif self.was_locked and self.locked_estimate is not None:
            # The physical box did not move merely because one rolling window
            # became noisy. Keep the exact geometry committed to the current
            # PlanningScene until a persistent jump is confirmed.
            self.publish_detection(message, self.locked_estimate)
        else:
            self.was_locked = False
        display_estimate = (
            self.locked_estimate
            if self.was_locked and self.locked_estimate is not None
            else estimate
        )
        display_dimensions = display_estimate[4:6]
        display_corners = rectangle_corners(
            display_estimate[:2], display_dimensions, display_estimate[3]
        )
        self.draw_world_rectangle(
            debug, display_corners, display_estimate[2], rotation, translation
        )
        status = (
            f"SUPPORT OK | "
            f"center [{display_estimate[0]:.3f}, {display_estimate[1]:.3f}] "
            f"yaw {math.degrees(display_estimate[3]):.1f}deg "
            f"top {display_estimate[2]:.3f} support {display_estimate[6]:.3f} "
            f"height {display_estimate[7]:.3f} "
            f"size {display_estimate[4]:.3f}x{display_estimate[5]:.3f} | "
            f"{'LOCK' if self.was_locked else f'STAB {len(self.samples)}/{self.samples.maxlen}'}"
        )
        self.publish_debug(message, debug, status, self.was_locked)

    def _measure_top(
        self,
        surface: ColoredSurface,
        all_rows: np.ndarray,
        all_columns: np.ndarray,
        valid_depth: np.ndarray,
        all_world_points: np.ndarray,
        support_plane: SupportPlane,
    ) -> tuple[Optional[np.ndarray], str]:
        """Measure one elevated white top in RGB-D world coordinates.

        The support surface is already known (one per frame); support_z is
        evaluated from that global plane at the fitted top center. Returns
        ``(sample, "")`` on success and ``(None, reason)`` on failure.
        """
        mask_surface = surface.mask[all_rows, all_columns] != 0
        candidate_valid = valid_depth & mask_surface
        rows = all_rows[candidate_valid]
        columns = all_columns[candidate_valid]
        world_points = all_world_points[candidate_valid]
        if world_points.shape[0] > self.max_cloud_points:
            selected = self.rng.choice(
                world_points.shape[0], self.max_cloud_points, replace=False
            )
            rows, columns, world_points = (
                rows[selected],
                columns[selected],
                world_points[selected],
            )
        if world_points.shape[0] < self.min_plane_points:
            return None, "not enough white depth points"
        plane_mask = fit_horizontal_plane_ransac(
            world_points,
            self.ransac_iterations,
            self.plane_threshold,
            self.max_plane_tilt_rad,
            self.rng,
        )
        if plane_mask is None or np.count_nonzero(plane_mask) < self.min_plane_points:
            return None, "white top has no horizontal plane"
        plane_inlier_ratio = float(np.count_nonzero(plane_mask)) / float(
            world_points.shape[0]
        )
        if plane_inlier_ratio < self.surface_min_plane_inlier_ratio:
            return None, f"white plane coverage too low ({plane_inlier_ratio:.2f})"
        plane_points = world_points[plane_mask]
        top_z = float(np.median(plane_points[:, 2]))
        fit = min_area_box_top(plane_points)
        if fit is None:
            return None, "white top has no usable footprint"
        support_z = support_plane.z_at(fit.center_xy)
        measured_height = top_z - support_z
        if not math.isfinite(measured_height) or measured_height <= 0.0:
            return None, "measured box height is not positive"
        sample = np.asarray(
            [
                fit.center_xy[0],
                fit.center_xy[1],
                top_z,
                fit.yaw,
                fit.dimensions_xy[0],
                fit.dimensions_xy[1],
                support_z,
                measured_height,
            ],
            dtype=np.float64,
        )
        return sample, ""

    @staticmethod
    def _annotate_candidate(
        frame: np.ndarray,
        surface: ColoredSurface,
        candidate_index: int,
        tag: str,
        color: tuple[int, int, int],
    ) -> None:
        """Draw a candidate contour and its selection state on the debug image."""
        x, y, _, _ = cv2.boundingRect(surface.contour)
        cv2.polylines(frame, [surface.contour], True, color, 2, cv2.LINE_AA)
        cv2.putText(
            frame,
            f"cand{int(candidate_index)} {tag}",
            (max(8, x), max(16, y - 6)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.45,
            color,
            1,
            cv2.LINE_AA,
        )

    @staticmethod
    def stable_estimate(samples: np.ndarray) -> np.ndarray:
        estimate = np.median(samples, axis=0)
        estimate[3] = mean_axis_yaw(samples[:, 3])
        return estimate

    def reject(self, source: Image, debug: np.ndarray, reason: str) -> None:
        reason_key = reason.split("(", maxsplit=1)[0].strip()
        self.warn_throttled(
            f"reject:{reason_key}", f"Box candidate rejected: {reason}", 2.0
        )
        self.missed_frames += 1
        if self.missed_frames >= self.max_missed_frames:
            self.samples.clear()
            self.was_locked = False
            self.locked_estimate = None
        self.publish_debug(source, debug, reason, False)

    @staticmethod
    def draw_surface_mask(
        frame: np.ndarray,
        mask: np.ndarray,
        color: tuple[int, int, int] = (0, 0, 255),
        alpha: float = 0.24,
    ) -> None:
        overlay = frame.copy()
        overlay[mask != 0] = color
        cv2.addWeighted(overlay, alpha, frame, 1.0 - alpha, 0.0, dst=frame)

    @staticmethod
    def draw_sample_pixels(
        frame: np.ndarray,
        rows: np.ndarray,
        columns: np.ndarray,
        color: tuple[int, int, int],
        alpha: float,
    ) -> None:
        mask = np.zeros(frame.shape[:2], dtype=np.uint8)
        mask[rows, columns] = 255
        mask = cv2.dilate(mask, np.ones((3, 3), dtype=np.uint8))
        overlay = frame.copy()
        overlay[mask != 0] = color
        cv2.addWeighted(overlay, alpha, frame, 1.0 - alpha, 0.0, dst=frame)

    def draw_world_rectangle(
        self,
        frame: np.ndarray,
        corners_xy: np.ndarray,
        top_z: float,
        world_from_camera_rotation: np.ndarray,
        world_from_camera_translation: np.ndarray,
    ) -> None:
        world_corners = np.column_stack(
            (corners_xy, np.full(corners_xy.shape[0], top_z))
        )
        camera_corners = (world_corners - world_from_camera_translation) @ (
            world_from_camera_rotation
        )
        if np.any(camera_corners[:, 2] <= 0.01):
            return
        camera_matrix = self.camera_matrix
        if camera_matrix is None:
            return
        fx, fy = camera_matrix[0, 0], camera_matrix[1, 1]
        cx, cy = camera_matrix[0, 2], camera_matrix[1, 2]
        pixels = np.column_stack(
            (
                fx * camera_corners[:, 0] / camera_corners[:, 2] + cx,
                fy * camera_corners[:, 1] / camera_corners[:, 2] + cy,
            )
        )
        pixels = np.rint(pixels).astype(np.int32)
        cv2.polylines(frame, [pixels], True, (0, 255, 0), 3, cv2.LINE_AA)
        center = np.rint(np.mean(pixels, axis=0)).astype(int)
        cv2.drawMarker(
            frame, tuple(center), (255, 0, 255), cv2.MARKER_CROSS, 18, 2
        )

    def publish_detection(self, source: Image, estimate: np.ndarray) -> None:
        top_z = float(estimate[2])
        support_z = float(estimate[6])
        yaw = float(estimate[3])
        footprint_dimensions = estimate[4:6]
        # All three dimensions are measured. No fixed-size fallback is allowed.
        dimensions = np.asarray(
            [footprint_dimensions[0], footprint_dimensions[1], estimate[7]]
        )
        center_z = 0.5 * (top_z + support_z)
        pose = PoseStamped()
        pose.header.stamp = source.header.stamp
        pose.header.frame_id = self.world_frame
        pose.pose.position.x = float(estimate[0])
        pose.pose.position.y = float(estimate[1])
        pose.pose.position.z = center_z
        pose.pose.orientation.z = math.sin(0.5 * yaw)
        pose.pose.orientation.w = math.cos(0.5 * yaw)
        self.pose_publisher.publish(pose)

        dimensions_message = Vector3Stamped()
        dimensions_message.header = pose.header
        dimensions_message.vector.x = float(dimensions[0])
        dimensions_message.vector.y = float(dimensions[1])
        dimensions_message.vector.z = float(dimensions[2])
        self.dimensions_publisher.publish(dimensions_message)

        measured_message = Vector3Stamped()
        measured_message.header = pose.header
        measured_message.vector.x = float(dimensions[0])
        measured_message.vector.y = float(dimensions[1])
        measured_message.vector.z = float(dimensions[2])
        self.detected_dimensions_publisher.publish(measured_message)
        self.top_height_publisher.publish(Float64(data=top_z))
        self.support_height_publisher.publish(Float64(data=support_z))

        corners = rectangle_corners(estimate[:2], dimensions[:2], yaw)
        footprint = PolygonStamped()
        footprint.header = pose.header
        footprint.polygon.points = [
            Point32(x=float(point[0]), y=float(point[1]), z=top_z)
            for point in corners
        ]
        self.footprint_publisher.publish(footprint)

        transform = TransformStamped()
        transform.header = pose.header
        transform.child_frame_id = self.box_frame
        transform.transform.translation.x = pose.pose.position.x
        transform.transform.translation.y = pose.pose.position.y
        transform.transform.translation.z = pose.pose.position.z
        transform.transform.rotation = pose.pose.orientation
        self.tf_broadcaster.sendTransform(transform)

    def publish_debug(
        self, source: Image, frame: np.ndarray, status: str, locked: bool
    ) -> None:
        color = (0, 255, 0) if locked else (0, 128, 255)
        lines = [part.strip() for part in status.split("|", maxsplit=1)]
        bar_height = 14 + 34 * len(lines)
        cv2.rectangle(
            frame, (8, 8), (frame.shape[1] - 8, bar_height), (0, 0, 0), -1
        )
        for index, line in enumerate(lines):
            cv2.putText(
                frame,
                line,
                (18, 36 + 32 * index),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.62,
                color,
                2,
                cv2.LINE_AA,
            )
        height, width = frame.shape[:2]
        scale = min(
            1.0,
            self.debug_max_width / float(width),
            self.debug_max_height / float(height),
        )
        if scale < 1.0:
            frame = cv2.resize(
                frame,
                (max(1, int(width * scale)), max(1, int(height * scale))),
                interpolation=cv2.INTER_AREA,
            )
        message = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
        message.header = source.header
        self.debug_publisher.publish(message)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = BoxDetector()
    executor = MultiThreadedExecutor(num_threads=3)
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        executor.remove_node(node)
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
