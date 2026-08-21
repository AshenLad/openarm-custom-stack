"""Minimum-area oriented rectangle for horizontal planar point clouds.

The box top is observed by one RGB-D camera and is commonly incomplete: an
object hides part of the surface and grazing angles make the point density
uneven. PCA follows the distribution of the visible points, so its first
eigenvector is not necessarily parallel to a physical box edge. This module
instead uses OpenCV's mature rotating-calipers ``minAreaRect`` primitive on
the 2-D convex hull.

The returned yaw describes an unoriented long axis in [-pi/2, pi/2), and the
dimensions always follow L >= W. No temporal state or known box dimension is
used.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Optional

import cv2
import numpy as np


@dataclass
class BoxTopFit:
    """Minimum-area box-top measurement in world coordinates."""

    center_xy: np.ndarray
    dimensions_xy: np.ndarray
    yaw: float


def normalize_axis_yaw(yaw: float) -> float:
    """Normalize an unoriented axis angle to [-pi/2, pi/2)."""
    return (yaw + 0.5 * math.pi) % math.pi - 0.5 * math.pi


def min_area_box_top(
    points_3d: np.ndarray,
    low_percentile: float = 0.0,
    high_percentile: float = 100.0,
    min_points: int = 3,
) -> Optional[BoxTopFit]:
    """Fit a minimum-area rectangle to a horizontal top-plane cloud.

    ``minAreaRect`` supplies the orientation of the convex hull. Bounds are
    measured from projection percentiles: the default 0/100 preserves the
    complete sparse box boundary, while object callers can request robust
    trimming of unsupported RGB-D fringe points. Partial measurements are
    handled by the detector's temporal LOCK gate rather than a known size.
    """
    if (
        not isinstance(points_3d, np.ndarray)
        or points_3d.ndim != 2
        or points_3d.shape[1] != 3
        or points_3d.shape[0] < min_points
        or not 0.0 <= low_percentile < high_percentile <= 100.0
    ):
        return None

    xy = points_3d[:, :2].astype(np.float64, copy=False)
    xy = xy[np.all(np.isfinite(xy), axis=1)]
    if xy.shape[0] < min_points:
        return None

    # Fit around a local origin to preserve float32 precision at large world
    # coordinates, then translate the robust center back to world XY.
    origin = np.median(xy, axis=0)
    local_xy = xy - origin
    hull = cv2.convexHull(local_xy.astype(np.float32).reshape(-1, 1, 2))
    if hull.shape[0] < 3 or abs(float(cv2.contourArea(hull))) <= 1.0e-10:
        return None
    rectangle = cv2.minAreaRect(hull)
    corners = cv2.boxPoints(rectangle).astype(np.float64)
    edges = np.roll(corners, -1, axis=0) - corners
    edge_lengths = np.linalg.norm(edges, axis=1)
    long_edge = edges[int(np.argmax(edge_lengths))]
    long_norm = float(np.linalg.norm(long_edge))
    if long_norm <= 1.0e-12:
        return None
    long_axis = long_edge / long_norm
    short_axis = np.asarray([-long_axis[1], long_axis[0]], dtype=np.float64)

    # With robust bounds the hull orientation itself must also be robust. A
    # D435 edge trail can rotate minAreaRect even when its final 2/98 extent is
    # trimmed. Search the unoriented 90-degree interval for the minimum
    # quantile-bounded rectangle, then refine locally. Box callers use 0/100
    # and retain OpenCV's exact rotating-calipers result.
    if low_percentile > 0.0 or high_percentile < 100.0:
        def robust_area(angle: float) -> tuple[float, np.ndarray, np.ndarray]:
            axis = np.asarray([math.cos(angle), math.sin(angle)])
            perpendicular = np.asarray([-axis[1], axis[0]])
            first = local_xy @ axis
            second = local_xy @ perpendicular
            first_low, first_high = np.percentile(
                first, [low_percentile, high_percentile]
            )
            second_low, second_high = np.percentile(
                second, [low_percentile, high_percentile]
            )
            spans = np.asarray(
                [first_high - first_low, second_high - second_low],
                dtype=np.float64,
            )
            return float(spans[0] * spans[1]), axis, perpendicular

        coarse_angles = np.linspace(0.0, 0.5 * math.pi, 361, endpoint=False)
        coarse = [robust_area(float(angle)) for angle in coarse_angles]
        best_index = int(np.argmin([candidate[0] for candidate in coarse]))
        best_angle = float(coarse_angles[best_index])
        coarse_step = 0.5 * math.pi / coarse_angles.size
        fine_angles = np.linspace(
            best_angle - coarse_step,
            best_angle + coarse_step,
            41,
        )
        fine = [robust_area(float(angle)) for angle in fine_angles]
        best_fine = int(np.argmin([candidate[0] for candidate in fine]))
        _, long_axis, short_axis = fine[best_fine]

    # A convex hull is sensitive to valid-but-unsupported depth-edge pixels.
    # Measure the final bounds from the requested robust projection quantiles.
    # The orientation still comes from rotating calipers; only unsupported
    # fringe points are prevented from defining the physical L/W and center.
    long_projection = local_xy @ long_axis
    short_projection = local_xy @ short_axis
    long_low, long_high = np.percentile(
        long_projection, [low_percentile, high_percentile]
    )
    short_low, short_high = np.percentile(
        short_projection, [low_percentile, high_percentile]
    )
    dimensions = np.asarray(
        [float(long_high - long_low), float(short_high - short_low)],
        dtype=np.float64,
    )
    if not np.all(np.isfinite(dimensions)) or np.any(dimensions <= 0.0):
        return None

    center_local = (
        0.5 * float(long_low + long_high) * long_axis
        + 0.5 * float(short_low + short_high) * short_axis
    )
    center_xy = center_local + origin
    yaw = normalize_axis_yaw(
        math.atan2(float(long_axis[1]), float(long_axis[0]))
    )

    # Nearly square clouds can make either rectangle edge the nominal long
    # edge. Preserve the public L >= W convention together with its yaw.
    if dimensions[1] > dimensions[0]:
        dimensions = dimensions[::-1].copy()
        yaw = normalize_axis_yaw(yaw + 0.5 * math.pi)
    return BoxTopFit(
        center_xy=center_xy,
        dimensions_xy=dimensions,
        yaw=yaw,
    )


# Compatibility for out-of-tree code importing the temporary refactor name.
pca_box_top = min_area_box_top
