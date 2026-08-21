"""Generic RGB-D object-on-box detector.

The executable and ``/blue_cube/*`` topics retain their legacy names for MTC
compatibility. Detection itself is color independent: points are selected by
the live box footprint and height above its live top plane, filtered with a
radius-neighbour test, split with the classic PCL Euclidean-cluster pipeline,
and measured with a minimum-area XY OBB.
"""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass
import math
from typing import Optional

import cv2
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseStamped, TransformStamped, Vector3Stamped
import numpy as np
import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from scipy.spatial import cKDTree
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformBroadcaster, TransformException, TransformListener

from openarm_aruco_vision.obb import min_area_box_top


def quaternion_to_rotation_matrix(
    x: float, y: float, z: float, w: float
) -> np.ndarray:
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if norm == 0.0:
        raise ValueError("zero-length quaternion")
    x, y, z, w = x / norm, y / norm, z / norm, w / norm
    return np.asarray(
        [
            [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w), 2.0 * (x * z + y * w)],
            [2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - x * w)],
            [2.0 * (x * z - y * w), 2.0 * (y * z + x * w), 1.0 - 2.0 * (x * x + y * y)],
        ],
        dtype=np.float64,
    )


def deproject_pixels(
    rows: np.ndarray,
    columns: np.ndarray,
    depths: np.ndarray,
    camera_matrix: np.ndarray,
) -> np.ndarray:
    """Deproject aligned color pixels using measured depth in metres."""
    fx, fy = camera_matrix[0, 0], camera_matrix[1, 1]
    cx, cy = camera_matrix[0, 2], camera_matrix[1, 2]
    return np.column_stack(
        (
            (columns.astype(np.float64) - cx) * depths / fx,
            (rows.astype(np.float64) - cy) * depths / fy,
            depths,
        )
    )


def normalize_axis_yaw(yaw: float) -> float:
    return (yaw + 0.5 * math.pi) % math.pi - 0.5 * math.pi


def axis_yaw_error(angle: float, reference: float) -> float:
    return 0.5 * math.atan2(
        math.sin(2.0 * (angle - reference)),
        math.cos(2.0 * (angle - reference)),
    )


def axis_yaw_mean(angles: np.ndarray) -> float:
    return normalize_axis_yaw(
        0.5
        * math.atan2(
            float(np.mean(np.sin(2.0 * angles))),
            float(np.mean(np.cos(2.0 * angles))),
        )
    )


def symmetric_yaw_mean(angles: np.ndarray, symmetry_order: int) -> float:
    """Circular yaw mean for rectangle (2) or square (4) symmetry."""
    return float(
        (1.0 / symmetry_order)
        * math.atan2(
            float(np.mean(np.sin(symmetry_order * angles))),
            float(np.mean(np.cos(symmetry_order * angles))),
        )
    )


def symmetric_yaw_errors(
    angles: np.ndarray, reference: float, symmetry_order: int
) -> np.ndarray:
    return (1.0 / symmetry_order) * np.arctan2(
        np.sin(symmetry_order * (angles - reference)),
        np.cos(symmetry_order * (angles - reference)),
    )


def object_symmetry_order(dimensions: np.ndarray) -> int:
    """Return the detector/MTC shared planar symmetry contract."""
    if dimensions.shape[0] < 2 or dimensions[0] <= 0.0 or dimensions[1] <= 0.0:
        return 2
    longer = float(max(dimensions[0], dimensions[1]))
    shorter = float(min(dimensions[0], dimensions[1]))
    return 4 if longer / shorter <= 1.30 else 2


def robust_object_estimate(samples: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """Median geometry plus symmetry-aware yaw and yaw residuals."""
    if samples.ndim != 2 or samples.shape[1] != 7 or samples.shape[0] == 0:
        raise ValueError("object samples must have shape N x 7")
    estimate = np.median(samples, axis=0)
    symmetry_order = object_symmetry_order(estimate[3:5])
    estimate[6] = symmetric_yaw_mean(samples[:, 6], symmetry_order)
    yaw_errors = symmetric_yaw_errors(
        samples[:, 6], float(estimate[6]), symmetry_order
    )
    return estimate, yaw_errors


def object_measurement_jumped(
    sample: np.ndarray,
    reference: np.ndarray,
    max_position_change_m: float,
    max_size_change_m: float,
    max_yaw_change_rad: float,
) -> bool:
    """Detect a material scene change without treating square yaw flips as motion."""
    if sample.shape != (7,) or reference.shape != (7,):
        return True
    symmetry_order = object_symmetry_order(reference[3:5])
    yaw_error = abs(
        float(
            symmetric_yaw_errors(
                np.asarray([sample[6]]), float(reference[6]), symmetry_order
            )[0]
        )
    )
    return bool(
        np.linalg.norm(sample[:3] - reference[:3]) > max_position_change_m
        or np.max(np.abs(sample[3:6] - reference[3:6])) > max_size_change_m
        or yaw_error > max_yaw_change_rad
    )


@dataclass(frozen=True)
class BoxGeometry:
    center_xy: np.ndarray
    dimensions_xy: np.ndarray
    top_z: float
    yaw: float


@dataclass(frozen=True)
class ObjectFit:
    center: np.ndarray
    dimensions: np.ndarray
    yaw: float


@dataclass(frozen=True)
class HorizontalPlane:
    normal: np.ndarray
    d: float

    def heights(self, points: np.ndarray) -> np.ndarray:
        return points @ self.normal + self.d

    def z_at(self, xy: np.ndarray) -> float:
        return -(
            float(self.normal[0] * xy[0])
            + float(self.normal[1] * xy[1])
            + self.d
        ) / float(self.normal[2])


def points_in_oriented_footprint(
    world_points: np.ndarray, box: BoxGeometry, inset_m: float = 0.0
) -> np.ndarray:
    """Mask points inside the live oriented box footprint."""
    delta = world_points[:, :2] - box.center_xy
    cosine, sine = math.cos(box.yaw), math.sin(box.yaw)
    rotation = np.asarray([[cosine, -sine], [sine, cosine]])
    local = delta @ rotation
    half_extents = np.maximum(0.0, 0.5 * box.dimensions_xy - inset_m)
    return np.all(np.abs(local) <= half_extents, axis=1)


def radius_outlier_filter(
    points: np.ndarray, radius_m: float, minimum_neighbors: int
) -> np.ndarray:
    """PCL/Open3D-style radius outlier removal using a mature KD-tree."""
    if points.shape[0] < minimum_neighbors:
        return np.empty((0, 3), dtype=np.float64)
    tree = cKDTree(points)
    keep = np.fromiter(
        (
            len(tree.query_ball_point(point, radius_m)) >= minimum_neighbors
            for point in points
        ),
        dtype=bool,
        count=points.shape[0],
    )
    return points[keep]


def voxel_down_sample(points: np.ndarray, voxel_size_m: float) -> np.ndarray:
    """PCL VoxelGrid equivalent: replace occupied voxels with centroids."""
    if points.shape[0] == 0:
        return points.copy()
    voxel_indices = np.floor(points / voxel_size_m).astype(np.int64)
    _, inverse = np.unique(voxel_indices, axis=0, return_inverse=True)
    counts = np.bincount(inverse)
    return np.column_stack(
        [
            np.bincount(inverse, weights=points[:, axis]) / counts
            for axis in range(3)
        ]
    )


def estimate_box_top_plane(
    world_points: np.ndarray,
    box: BoxGeometry,
    distance_threshold_m: float = 0.006,
    max_tilt_rad: float = math.radians(15.0),
    iterations: int = 60,
    rng: Optional[np.random.Generator] = None,
) -> Optional[HorizontalPlane]:
    """RANSAC the dominant near-horizontal plane in the live footprint."""
    if rng is None:
        rng = np.random.default_rng(20260820)
    finite = np.all(np.isfinite(world_points), axis=1)
    inside = points_in_oriented_footprint(world_points, box)
    points = world_points[finite & inside]
    # The published top height seeds a geometric search band wide enough for
    # the allowed plane tilt across this dynamically measured footprint.
    half_diagonal = 0.5 * float(np.linalg.norm(box.dimensions_xy))
    band = half_diagonal * math.tan(max_tilt_rad) + 3.0 * distance_threshold_m
    points = points[np.abs(points[:, 2] - box.top_z) <= band]
    if points.shape[0] < 30:
        return None
    if points.shape[0] > 6000:
        selected = rng.choice(points.shape[0], 6000, replace=False)
        points = points[selected]
    minimum_vertical = math.cos(max_tilt_rad)
    best_mask: Optional[np.ndarray] = None
    best_count = 0
    for _ in range(iterations):
        indices = rng.choice(points.shape[0], 3, replace=False)
        first, second, third = points[indices]
        normal = np.cross(second - first, third - first)
        norm = float(np.linalg.norm(normal))
        if norm <= 1.0e-9:
            continue
        normal /= norm
        if abs(float(normal[2])) < minimum_vertical:
            continue
        distances = np.abs((points - first) @ normal)
        mask = distances <= distance_threshold_m
        count = int(np.count_nonzero(mask))
        if count > best_count:
            best_count, best_mask = count, mask
    if best_mask is None or best_count < 30:
        return None
    inliers = points[best_mask]
    centroid = np.mean(inliers, axis=0)
    _, _, vectors = np.linalg.svd(inliers - centroid, full_matrices=False)
    normal = vectors[-1]
    if abs(float(normal[2])) < minimum_vertical:
        return None
    if normal[2] < 0.0:
        normal = -normal
    return HorizontalPlane(normal=normal, d=-float(normal @ centroid))


def euclidean_clusters(
    points: np.ndarray, tolerance_m: float, minimum_points: int
) -> list[np.ndarray]:
    """Classic PCL EuclideanClusterExtraction over a KD-tree."""
    if points.shape[0] < minimum_points:
        return []
    tree = cKDTree(points)
    visited = np.zeros(points.shape[0], dtype=bool)
    clusters: list[np.ndarray] = []
    for seed in range(points.shape[0]):
        if visited[seed]:
            continue
        visited[seed] = True
        members = [seed]
        frontier = [seed]
        while frontier:
            current = frontier.pop()
            for neighbor in tree.query_ball_point(points[current], tolerance_m):
                if not visited[neighbor]:
                    visited[neighbor] = True
                    members.append(neighbor)
                    frontier.append(neighbor)
        if len(members) >= minimum_points:
            clusters.append(points[np.asarray(members, dtype=np.int64)])
    clusters.sort(key=lambda cluster: cluster.shape[0], reverse=True)
    return clusters


def densest_top_component(
    points: np.ndarray,
    neighbor_count: int = 6,
    mad_scale: float = 5.0,
    connectivity_scale: float = 3.0,
    minimum_points: int = 6,
) -> np.ndarray:
    """Keep the densest connected support on an object's horizontal top.

    D435 depth discontinuities commonly create one-dimensional, same-height
    trails beside a real object.  They can survive the earlier 3-D radius
    filter and join the object cluster, but their k-nearest-neighbour distance
    is much larger than that of the physical surface.  Apply a robust
    statistical outlier test in XY, then retain the largest connected
    component.  Both the rejection threshold and connectivity distance are
    derived from the observed sampling density; no object size or shape is
    assumed.
    """
    if (
        not isinstance(points, np.ndarray)
        or points.ndim != 2
        or points.shape[1] != 3
        or points.shape[0] < minimum_points
        or neighbor_count < 2
        or mad_scale <= 0.0
        or connectivity_scale <= 1.0
        or minimum_points < 3
    ):
        return np.empty((0, 3), dtype=np.float64)
    finite_points = points[np.all(np.isfinite(points), axis=1)]
    if finite_points.shape[0] < minimum_points:
        return np.empty((0, 3), dtype=np.float64)

    # StatisticalOutlierRemoval-style score: mean distance to the k nearest
    # XY neighbours.  Median/MAD makes the threshold independent of metres,
    # camera range, voxel size, and whether the object is square/rectangular.
    xy = finite_points[:, :2]
    query_count = min(neighbor_count + 1, xy.shape[0])
    distances, _ = cKDTree(xy).query(xy, k=query_count)
    if distances.ndim == 1:
        return finite_points.copy()
    scores = np.mean(distances[:, 1:], axis=1)
    score_median = float(np.median(scores))
    score_mad = float(np.median(np.abs(scores - score_median)))
    robust_scale = max(score_mad, 0.05 * score_median, 1.0e-9)
    supported = finite_points[scores <= score_median + mad_scale * robust_scale]
    if supported.shape[0] < minimum_points:
        return np.empty((0, 3), dtype=np.float64)

    # Connectivity is intentionally evaluated in XY.  The input has already
    # been restricted to a narrow upper-height band, while the artifact to be
    # removed is primarily a lateral depth-edge trail.
    xy_tree = cKDTree(supported[:, :2])
    nearest_distances, _ = xy_tree.query(supported[:, :2], k=2)
    sampling_spacing = float(np.median(nearest_distances[:, 1]))
    if not math.isfinite(sampling_spacing) or sampling_spacing <= 0.0:
        return np.empty((0, 3), dtype=np.float64)
    connectivity_m = connectivity_scale * sampling_spacing
    visited = np.zeros(supported.shape[0], dtype=bool)
    components: list[np.ndarray] = []
    for seed in range(supported.shape[0]):
        if visited[seed]:
            continue
        visited[seed] = True
        members = [seed]
        frontier = [seed]
        while frontier:
            current = frontier.pop()
            for neighbor in xy_tree.query_ball_point(
                supported[current, :2], connectivity_m
            ):
                if not visited[neighbor]:
                    visited[neighbor] = True
                    members.append(neighbor)
                    frontier.append(neighbor)
        if len(members) >= minimum_points:
            components.append(np.asarray(members, dtype=np.int64))
    if not components:
        return np.empty((0, 3), dtype=np.float64)
    largest = max(components, key=lambda indices: indices.size)
    return supported[largest]


def extract_object_clusters(
    world_points: np.ndarray,
    box: BoxGeometry,
    top_clearance_m: float,
    outlier_radius_m: float,
    outlier_minimum_neighbors: int,
    cluster_tolerance_m: float,
    cluster_minimum_points: int,
    top_plane: Optional[HorizontalPlane] = None,
    voxel_size_m: float = 0.004,
    footprint_inset_m: float = 0.0,
) -> list[np.ndarray]:
    """Dynamic footprint -> remove top plane -> outliers -> 3-D clusters."""
    if world_points.ndim != 2 or world_points.shape[1] != 3:
        return []
    finite = np.all(np.isfinite(world_points), axis=1)
    inside = points_in_oriented_footprint(
        world_points, box, footprint_inset_m
    )
    if top_plane is None:
        top_plane = HorizontalPlane(
            normal=np.asarray([0.0, 0.0, 1.0]), d=-box.top_z
        )
    above_top = top_plane.heights(world_points) >= top_clearance_m
    object_points = world_points[finite & inside & above_top]
    object_points = voxel_down_sample(object_points, voxel_size_m)
    object_points = radius_outlier_filter(
        object_points, outlier_radius_m, outlier_minimum_neighbors
    )
    return euclidean_clusters(
        object_points, cluster_tolerance_m, cluster_minimum_points
    )


def fit_object_obb(
    cluster: np.ndarray,
    box_top: float | HorizontalPlane,
    top_band_m: float = 0.002,
    top_density_neighbor_count: int = 6,
    top_density_mad_scale: float = 5.0,
    top_component_connectivity_scale: float = 3.0,
) -> Optional[ObjectFit]:
    """Measure object center, L/W/H and horizontal long-axis yaw."""
    plane = (
        box_top
        if isinstance(box_top, HorizontalPlane)
        else HorizontalPlane(np.asarray([0.0, 0.0, 1.0]), -float(box_top))
    )
    heights = plane.heights(cluster)
    finite = np.isfinite(heights) & np.all(np.isfinite(cluster), axis=1)
    heights = heights[finite]
    points = cluster[finite]
    if heights.size < 6:
        return None

    # Recover the upper horizontal surface rather than taking every point in a
    # broad upper slab.  A single-view RGB-D cloud contains vertical side-wall
    # samples and flying edge pixels; feeding those directly to a convex hull
    # is what produced false 70-100 mm "long" sides for a 40 mm block.
    if not math.isfinite(top_band_m) or top_band_m <= 0.0:
        return None
    top_seed = float(np.percentile(heights, 80.0))
    top_mask = np.abs(heights - top_seed) <= top_band_m
    if int(np.count_nonzero(top_mask)) < 6:
        top_seed = float(np.percentile(heights, 90.0))
        top_mask = np.abs(heights - top_seed) <= 2.0 * top_band_m
    top_points = points[top_mask]
    if top_points.shape[0] < 6:
        return None

    top_points = densest_top_component(
        top_points,
        neighbor_count=top_density_neighbor_count,
        mad_scale=top_density_mad_scale,
        connectivity_scale=top_component_connectivity_scale,
        minimum_points=6,
    )
    if top_points.shape[0] < 6:
        return None

    # Use the dense top band for height as well.  The previous 99th percentile
    # allowed the same edge outliers to raise the object center and alter the
    # MTC fingertip-clearance offset.
    height = float(np.median(plane.heights(top_points)))
    if not math.isfinite(height) or height <= 0.0:
        return None
    fit = min_area_box_top(
        top_points, low_percentile=2.0, high_percentile=98.0, min_points=6
    )
    if fit is None:
        return None
    base_z = plane.z_at(fit.center_xy)
    return ObjectFit(
        center=np.asarray(
            [fit.center_xy[0], fit.center_xy[1], base_z + 0.5 * height]
        ),
        dimensions=np.asarray(
            [fit.dimensions_xy[0], fit.dimensions_xy[1], height]
        ),
        yaw=fit.yaw,
    )


class BlueCubeDetector(Node):
    """Generic object detector retaining a legacy class/executable name."""

    def __init__(self) -> None:
        super().__init__("blue_cube_detector")
        defaults = (
            ("color_image_topic", "/camera/global_camera/color/image_raw"),
            ("aligned_depth_topic", "/camera/global_camera/aligned_depth_to_color/image_raw"),
            ("camera_info_topic", "/camera/global_camera/color/camera_info"),
            # Legacy topic/frame names retained for downstream compatibility.
            ("world_pose_topic", "/blue_cube/pose"),
            ("dimensions_topic", "/blue_cube/dimensions"),
            ("camera_pose_topic", "/blue_cube/camera_pose"),
            ("debug_image_topic", "/blue_cube/debug_image"),
            ("box_pose_topic", "/box/pose"),
            ("box_dimensions_topic", "/box/dimensions"),
            ("world_frame", "world"),
            ("cube_frame", "blue_cube"),
            ("depth_scale", 0.001),
            ("min_depth_m", 0.10),
            ("max_depth_m", 2.0),
            ("point_stride", 3),
            ("max_rgb_depth_time_delta_sec", 0.50),
            ("box_top_clearance_m", 0.024),
            ("voxel_size_m", 0.004),
            ("footprint_inset_m", 0.012),
            ("outlier_radius_m", 0.018),
            ("outlier_min_neighbors", 4),
            ("cluster_tolerance_m", 0.018),
            ("cluster_min_points", 20),
            ("object_top_band_m", 0.002),
            ("top_density_neighbor_count", 6),
            ("top_density_mad_scale", 5.0),
            ("top_component_connectivity_scale", 3.0),
            ("stable_samples", 5),
            ("max_position_std_m", 0.008),
            ("max_size_std_m", 0.008),
            ("max_yaw_std_rad", 0.12),
            ("sample_jump_reset_m", 0.03),
            # Once LOCKed, isolated RGB-D edge artifacts must not make a
            # stationary scene disappear or silently replace the planning
            # snapshot. A real change must persist for several frames.
            ("locked_max_position_change_m", 0.012),
            ("locked_max_size_change_m", 0.012),
            ("locked_max_yaw_change_rad", 0.20),
            ("lock_change_confirm_frames", 3),
            ("max_missed_frames", 3),
            ("processing_interval_sec", 0.10),
        )
        for name, default in defaults:
            self.declare_parameter(name, default)
        value = lambda name: self.get_parameter(name).value
        self.color_topic = str(value("color_image_topic"))
        self.depth_topic = str(value("aligned_depth_topic"))
        self.info_topic = str(value("camera_info_topic"))
        self.world_frame = str(value("world_frame"))
        self.cube_frame = str(value("cube_frame"))
        self.depth_scale = float(value("depth_scale"))
        self.min_depth = float(value("min_depth_m"))
        self.max_depth = float(value("max_depth_m"))
        self.point_stride = int(value("point_stride"))
        self.max_time_delta_ns = int(
            float(value("max_rgb_depth_time_delta_sec")) * 1.0e9
        )
        self.top_clearance = float(value("box_top_clearance_m"))
        self.voxel_size = float(value("voxel_size_m"))
        self.footprint_inset = float(value("footprint_inset_m"))
        self.outlier_radius = float(value("outlier_radius_m"))
        self.outlier_min_neighbors = int(value("outlier_min_neighbors"))
        self.cluster_tolerance = float(value("cluster_tolerance_m"))
        self.cluster_min_points = int(value("cluster_min_points"))
        self.object_top_band = float(value("object_top_band_m"))
        self.top_density_neighbor_count = int(
            value("top_density_neighbor_count")
        )
        self.top_density_mad_scale = float(value("top_density_mad_scale"))
        self.top_component_connectivity_scale = float(
            value("top_component_connectivity_scale")
        )
        stable_samples = int(value("stable_samples"))
        self.max_position_std = float(value("max_position_std_m"))
        self.max_size_std = float(value("max_size_std_m"))
        self.max_yaw_std = float(value("max_yaw_std_rad"))
        self.sample_jump_reset = float(value("sample_jump_reset_m"))
        self.locked_max_position_change = float(
            value("locked_max_position_change_m")
        )
        self.locked_max_size_change = float(value("locked_max_size_change_m"))
        self.locked_max_yaw_change = float(value("locked_max_yaw_change_rad"))
        self.lock_change_confirm_frames = int(value("lock_change_confirm_frames"))
        self.max_missed_frames = int(value("max_missed_frames"))
        self.processing_interval = float(value("processing_interval_sec"))
        if (
            self.depth_scale <= 0.0
            or not 0.0 < self.min_depth < self.max_depth
            or self.point_stride < 1
            or self.max_time_delta_ns <= 0
            or self.top_clearance <= 0.0
            or self.voxel_size <= 0.0
            or self.footprint_inset < 0.0
            or self.outlier_radius <= 0.0
            or self.outlier_min_neighbors < 1
            or self.cluster_tolerance <= 0.0
            or self.cluster_min_points < 3
            or self.object_top_band <= 0.0
            or self.top_density_neighbor_count < 2
            or self.top_density_mad_scale <= 0.0
            or self.top_component_connectivity_scale <= 1.0
            or stable_samples < 1
            or self.locked_max_position_change <= 0.0
            or self.locked_max_size_change <= 0.0
            or self.locked_max_yaw_change <= 0.0
            or self.lock_change_confirm_frames < 1
            or self.max_missed_frames < 1
            or self.processing_interval <= 0.0
        ):
            raise ValueError("generic object geometry parameters are invalid")

        self.bridge = CvBridge()
        self.camera_matrix: Optional[np.ndarray] = None
        self.camera_frame = ""
        self.box_pose: Optional[PoseStamped] = None
        self.box_dimensions: Optional[Vector3Stamped] = None
        self.box_geometry: Optional[BoxGeometry] = None
        self.color_messages: deque[Image] = deque(maxlen=10)
        self.depth_messages: deque[Image] = deque(maxlen=10)
        self.latest_rgbd_pair: Optional[tuple[Image, Image]] = None
        self.last_processed_stamp_ns = 0
        self.samples: deque[np.ndarray] = deque(maxlen=stable_samples)
        self.missed_frames = 0
        self.locked_estimate: Optional[np.ndarray] = None
        self.consecutive_scene_changes = 0
        self.reported_detection = False
        self.last_warning_ns: dict[str, int] = {}
        self.rng = np.random.default_rng(20260820)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.world_pose_topic = str(value("world_pose_topic"))
        self.dimensions_topic = str(value("dimensions_topic"))
        self.world_pose_publisher = self.create_publisher(
            PoseStamped, self.world_pose_topic, 10
        )
        self.dimensions_publisher = self.create_publisher(
            Vector3Stamped, self.dimensions_topic, 10
        )
        self.camera_pose_publisher = self.create_publisher(
            PoseStamped, str(value("camera_pose_topic")), 10
        )
        self.debug_publisher = self.create_publisher(
            Image, str(value("debug_image_topic")), qos_profile_sensor_data
        )
        self.input_group = MutuallyExclusiveCallbackGroup()
        self.processing_group = MutuallyExclusiveCallbackGroup()
        self.create_subscription(
            CameraInfo, self.info_topic, self.camera_info_callback,
            qos_profile_sensor_data, callback_group=self.input_group,
        )
        self.create_subscription(
            Image, self.depth_topic, self.depth_callback,
            qos_profile_sensor_data, callback_group=self.input_group,
        )
        self.create_subscription(
            Image, self.color_topic, self.color_callback,
            qos_profile_sensor_data, callback_group=self.input_group,
        )
        self.create_subscription(
            PoseStamped, str(value("box_pose_topic")), self.box_pose_callback, 10,
            callback_group=self.input_group,
        )
        self.create_subscription(
            Vector3Stamped, str(value("box_dimensions_topic")),
            self.box_dimensions_callback, 10, callback_group=self.input_group,
        )
        self.create_timer(
            self.processing_interval,
            self.processing_callback,
            callback_group=self.processing_group,
        )
        self.get_logger().info(
            "Waiting for color-independent 3-D object above the live box top; "
            "legacy /blue_cube/* interfaces are retained"
        )

    @staticmethod
    def stamp_ns(message) -> int:
        return int(message.header.stamp.sec) * 1_000_000_000 + int(
            message.header.stamp.nanosec
        )

    def warn_throttled(self, key: str, message: str) -> None:
        now = self.get_clock().now().nanoseconds
        if now - self.last_warning_ns.get(key, 0) >= 2_000_000_000:
            self.get_logger().warning(message)
            self.last_warning_ns[key] = now

    def camera_info_callback(self, message: CameraInfo) -> None:
        matrix = np.asarray(message.k, dtype=np.float64).reshape(3, 3)
        if matrix[0, 0] > 0.0 and matrix[1, 1] > 0.0:
            self.camera_matrix = matrix
            self.camera_frame = message.header.frame_id

    def depth_callback(self, message: Image) -> None:
        self.depth_messages.append(message)
        self._match_rgbd(message, self.color_messages, depth_is_new=True)
        if not self.camera_frame:
            self.camera_frame = message.header.frame_id

    def color_callback(self, message: Image) -> None:
        self.color_messages.append(message)
        self._match_rgbd(message, self.depth_messages, depth_is_new=False)

    def _match_rgbd(
        self, message: Image, opposite: deque[Image], depth_is_new: bool
    ) -> None:
        if not opposite:
            return
        stamp = self.stamp_ns(message)
        nearest = min(opposite, key=lambda item: abs(self.stamp_ns(item) - stamp))
        if abs(self.stamp_ns(nearest) - stamp) > self.max_time_delta_ns:
            return
        self.latest_rgbd_pair = (
            (nearest, message) if depth_is_new else (message, nearest)
        )

    def box_pose_callback(self, message: PoseStamped) -> None:
        if message.header.frame_id == self.world_frame:
            self.box_pose = message
            self._update_box_geometry()

    def box_dimensions_callback(self, message: Vector3Stamped) -> None:
        if message.header.frame_id == self.world_frame:
            self.box_dimensions = message
            self._update_box_geometry()

    def _update_box_geometry(self) -> None:
        if self.box_pose is None or self.box_dimensions is None:
            return
        if self.stamp_ns(self.box_pose) != self.stamp_ns(self.box_dimensions):
            return
        pose = self.box_pose.pose
        dimensions = self.box_dimensions.vector
        values = np.asarray(
            [pose.position.x, pose.position.y, pose.position.z,
             dimensions.x, dimensions.y, dimensions.z], dtype=np.float64
        )
        if not np.all(np.isfinite(values)) or np.any(values[3:] <= 0.0):
            return
        yaw = normalize_axis_yaw(
            2.0 * math.atan2(float(pose.orientation.z), float(pose.orientation.w))
        )
        self.box_geometry = BoxGeometry(
            center_xy=values[:2],
            dimensions_xy=values[3:5],
            top_z=float(values[2] + 0.5 * values[5]),
            yaw=yaw,
        )

    def processing_callback(self) -> None:
        pair = self.latest_rgbd_pair
        box = self.box_geometry
        camera_matrix = self.camera_matrix
        if pair is None or box is None or camera_matrix is None:
            return
        color_message, depth_message = pair
        stamp = self.stamp_ns(depth_message)
        if stamp == self.last_processed_stamp_ns:
            return
        self.last_processed_stamp_ns = stamp
        try:
            frame = self.bridge.imgmsg_to_cv2(color_message, desired_encoding="bgr8")
            raw_depth = np.asarray(
                self.bridge.imgmsg_to_cv2(depth_message, desired_encoding="passthrough")
            )
        except Exception as error:
            self.warn_throttled("conversion", f"RGB-D conversion failed: {error}")
            return
        depth = (
            raw_depth.astype(np.float32) * self.depth_scale
            if depth_message.encoding == "16UC1"
            else raw_depth.astype(np.float32)
        )
        if frame.shape[:2] != depth.shape:
            self.warn_throttled("shape", "aligned depth and color sizes differ")
            return
        camera_frame = self.camera_frame or depth_message.header.frame_id
        try:
            transform = self.tf_buffer.lookup_transform(
                self.world_frame, camera_frame, Time()
            )
        except TransformException as error:
            self.warn_throttled(
                "tf", f"Waiting for TF {self.world_frame} <- {camera_frame}: {error}"
            )
            return
        q = transform.transform.rotation
        rotation = quaternion_to_rotation_matrix(q.x, q.y, q.z, q.w)
        t = transform.transform.translation
        translation = np.asarray([t.x, t.y, t.z], dtype=np.float64)

        rows_grid, columns_grid = np.mgrid[
            0:depth.shape[0]:self.point_stride,
            0:depth.shape[1]:self.point_stride,
        ]
        rows, columns = rows_grid.ravel(), columns_grid.ravel()
        depths = depth[rows, columns].astype(np.float64)
        valid = (
            np.isfinite(depths)
            & (depths >= self.min_depth)
            & (depths <= self.max_depth)
        )
        camera_points = deproject_pixels(
            rows[valid], columns[valid], depths[valid], camera_matrix
        )
        world_points = camera_points @ rotation.T + translation
        top_plane = estimate_box_top_plane(world_points, box, rng=self.rng)
        if top_plane is None:
            held = self._record_miss(
                color_message, camera_frame, rotation, translation
            )
            self.warn_throttled("top_plane", "No box-top plane inside live footprint")
            self.publish_debug(
                color_message,
                frame.copy(),
                "OBJECT LOCK HOLD" if held else "NO BOX-TOP PLANE",
            )
            return
        clusters = extract_object_clusters(
            world_points,
            box,
            self.top_clearance,
            self.outlier_radius,
            self.outlier_min_neighbors,
            self.cluster_tolerance,
            self.cluster_min_points,
            top_plane,
            self.voxel_size,
            self.footprint_inset,
        )
        fit = (
            fit_object_obb(
                clusters[0],
                top_plane,
                self.object_top_band,
                self.top_density_neighbor_count,
                self.top_density_mad_scale,
                self.top_component_connectivity_scale,
            )
            if clusters
            else None
        )
        debug = frame.copy()
        if fit is None:
            held = self._record_miss(
                color_message, camera_frame, rotation, translation
            )
            self.warn_throttled("no_object", "No valid 3-D object cluster above box top")
            self.publish_debug(
                color_message,
                debug,
                "OBJECT LOCK HOLD" if held else "NO 3-D OBJECT",
            )
            return

        self.missed_frames = 0
        sample = np.concatenate((fit.center, fit.dimensions, [fit.yaw]))
        if self.locked_estimate is not None and object_measurement_jumped(
            sample,
            self.locked_estimate,
            self.locked_max_position_change,
            self.locked_max_size_change,
            self.locked_max_yaw_change,
        ):
            self.consecutive_scene_changes += 1
            if self.consecutive_scene_changes < self.lock_change_confirm_frames:
                estimate = self.locked_estimate
                self.publish_poses(
                    color_message,
                    camera_frame,
                    estimate[:3],
                    estimate[3:6],
                    float(estimate[6]),
                    rotation,
                    translation,
                )
                self._draw_cluster(
                    debug,
                    rows[valid],
                    columns[valid],
                    world_points,
                    fit,
                    top_plane,
                )
                self.publish_debug(
                    color_message,
                    debug,
                    f"OBJECT LOCK HOLD CHANGE "
                    f"{self.consecutive_scene_changes}/"
                    f"{self.lock_change_confirm_frames}",
                )
                return
            # A persistent change is real. Drop the old transaction and build
            # a fresh stable window around the new physical scene.
            self.samples.clear()
            self.locked_estimate = None
            self.consecutive_scene_changes = 0
        else:
            self.consecutive_scene_changes = 0
        if self.samples:
            previous, _ = robust_object_estimate(np.asarray(self.samples))
            if np.linalg.norm(sample[:3] - previous[:3]) > self.sample_jump_reset:
                self.samples.clear()
        self.samples.append(sample)
        values = np.asarray(self.samples)
        estimate, yaw_errors = robust_object_estimate(values)
        stable = (
            len(self.samples) == self.samples.maxlen
            and np.all(np.std(values[:, :3], axis=0) <= self.max_position_std)
            and np.all(np.std(values[:, 3:6], axis=0) <= self.max_size_std)
            and float(np.std(yaw_errors)) <= self.max_yaw_std
        )
        if stable:
            first_lock = self.locked_estimate is None
            if first_lock:
                self.locked_estimate = estimate.copy()
                self.get_logger().info(
                    "Object LOCK acquired; production geometry will be held "
                    "through isolated RGB-D outliers"
                )
        published_estimate = self.locked_estimate
        if published_estimate is not None:
            self.publish_poses(
                color_message,
                camera_frame,
                published_estimate[:3],
                published_estimate[3:6],
                float(published_estimate[6]),
                rotation,
                translation,
            )
        self._draw_cluster(
            debug, rows[valid], columns[valid], world_points, fit, top_plane,
        )
        status = (
            f"3-D OBJECT {estimate[3]:.3f}x{estimate[4]:.3f}x{estimate[5]:.3f} "
            f"{'LOCK' if self.locked_estimate is not None else f'STAB {len(self.samples)}/{self.samples.maxlen}'}"
        )
        self.publish_debug(color_message, debug, status)

    def _record_miss(
        self,
        image: Image,
        camera_frame: str,
        rotation: np.ndarray,
        translation: np.ndarray,
    ) -> bool:
        self.missed_frames += 1
        if (
            self.locked_estimate is not None
            and self.missed_frames < self.max_missed_frames
        ):
            estimate = self.locked_estimate
            self.publish_poses(
                image,
                camera_frame,
                estimate[:3],
                estimate[3:6],
                float(estimate[6]),
                rotation,
                translation,
            )
            return True
        if self.missed_frames >= self.max_missed_frames:
            self.samples.clear()
            self.locked_estimate = None
            self.consecutive_scene_changes = 0
        return False

    @staticmethod
    def _draw_cluster(
        frame: np.ndarray,
        rows: np.ndarray,
        columns: np.ndarray,
        world_points: np.ndarray,
        fit: ObjectFit,
        top_plane: HorizontalPlane,
    ) -> None:
        delta = world_points[:, :2] - fit.center[:2]
        cosine, sine = math.cos(fit.yaw), math.sin(fit.yaw)
        local = delta @ np.asarray([[cosine, -sine], [sine, cosine]])
        inside_fit = np.all(
            np.abs(local) <= 0.5 * fit.dimensions[:2] + 0.003, axis=1
        )
        on_fitted_top = np.abs(
            top_plane.heights(world_points) - fit.dimensions[2]
        ) <= 0.007
        selected = (
            np.all(np.isfinite(world_points), axis=1)
            & inside_fit
            & on_fitted_top
        )
        frame[rows[selected], columns[selected]] = (0, 255, 0)

    def publish_poses(
        self,
        image: Image,
        camera_frame: str,
        world_center: np.ndarray,
        world_dimensions: np.ndarray,
        world_yaw: float,
        world_from_camera_rotation: np.ndarray,
        world_from_camera_translation: np.ndarray,
    ) -> None:
        world_pose = PoseStamped()
        world_pose.header.stamp = image.header.stamp
        world_pose.header.frame_id = self.world_frame
        world_pose.pose.position.x = float(world_center[0])
        world_pose.pose.position.y = float(world_center[1])
        world_pose.pose.position.z = float(world_center[2])
        world_pose.pose.orientation.z = math.sin(0.5 * world_yaw)
        world_pose.pose.orientation.w = math.cos(0.5 * world_yaw)
        self.world_pose_publisher.publish(world_pose)
        dimensions = Vector3Stamped()
        dimensions.header = world_pose.header
        dimensions.vector.x = float(world_dimensions[0])
        dimensions.vector.y = float(world_dimensions[1])
        dimensions.vector.z = float(world_dimensions[2])
        self.dimensions_publisher.publish(dimensions)

        camera_center = world_from_camera_rotation.T @ (
            world_center - world_from_camera_translation
        )
        camera_pose = PoseStamped()
        camera_pose.header = world_pose.header
        camera_pose.header.frame_id = camera_frame
        camera_pose.pose.position.x = float(camera_center[0])
        camera_pose.pose.position.y = float(camera_center[1])
        camera_pose.pose.position.z = float(camera_center[2])
        camera_pose.pose.orientation.w = 1.0
        self.camera_pose_publisher.publish(camera_pose)
        transform = TransformStamped()
        transform.header = world_pose.header
        transform.child_frame_id = self.cube_frame
        transform.transform.translation.x = world_pose.pose.position.x
        transform.transform.translation.y = world_pose.pose.position.y
        transform.transform.translation.z = world_pose.pose.position.z
        transform.transform.rotation = world_pose.pose.orientation
        self.tf_broadcaster.sendTransform(transform)
        if not self.reported_detection:
            self.get_logger().info(
                "Publishing generic object geometry on legacy "
                f"{self.world_pose_topic} and {self.dimensions_topic}"
            )
            self.reported_detection = True

    def publish_debug(self, source: Image, frame: np.ndarray, status: str) -> None:
        cv2.putText(
            frame, status, (20, 35), cv2.FONT_HERSHEY_SIMPLEX, 0.7,
            (0, 255, 0), 2, cv2.LINE_AA,
        )
        message = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
        message.header = source.header
        self.debug_publisher.publish(message)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = BlueCubeDetector()
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
