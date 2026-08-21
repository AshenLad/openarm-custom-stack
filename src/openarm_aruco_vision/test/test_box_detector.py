import math

import cv2
import numpy as np

from openarm_aruco_vision.box_detector_node import (
    BoxDetector,
    axis_angle_error,
    build_white_mask,
    choose_largest_valid_top,
    defer_locked_jump,
    estimate_support_plane_ransac,
    extract_candidates,
    fit_horizontal_plane_ransac,
    height_above_support,
    measurement_jumped,
    locked_box_measurement_changed,
    mean_axis_yaw,
    normalize_axis_yaw,
)
from openarm_aruco_vision.blue_cube_detector_node import deproject_pixels
from openarm_aruco_vision.obb import min_area_box_top


# ---------------------------------------------------------------------------
# Synthetic scene helpers for the depth-first pipeline.
#
# Camera looks straight down at the table: camera +Z points at world -Z, so a
# desk at camera depth 0.50 m becomes world z = -0.50 and a box top at camera
# depth 0.25 m becomes world z = -0.25 (0.25 m above the desk). The camera
# frame equals the world frame except for this fixed rotation, so the scene
# tests the exact pipeline used by color_callback: white mask -> support
# plane -> elevation mask -> white & elevated -> candidates.
# ---------------------------------------------------------------------------

CAMERA_MATRIX = np.asarray(
    [[300.0, 0.0, 160.0], [0.0, 300.0, 120.0], [0.0, 0.0, 1.0]], dtype=np.float64
)
CAMERA_TO_WORLD = np.asarray(
    [[1.0, 0.0, 0.0], [0.0, -1.0, 0.0], [0.0, 0.0, -1.0]], dtype=np.float64
)
DESK_DEPTH_M = 0.50
BOX_DEPTH_M = 0.25
TAPE_DEPTH_M = 0.48  # 0.02 m above the desk, below the 0.024 m elevation gate
MIN_ELEVATION_M = max(3.0 * 0.008, 0.02)


def make_scene(
    box_rect=None,
    tape_rects=(),
    box_depth=BOX_DEPTH_M,
    tape_depth=TAPE_DEPTH_M,
    desk_depth=DESK_DEPTH_M,
    shape=(240, 320),
):
    """Build a synthetic RGB frame and aligned depth image.

    Rectangles are ``(x0, y0, x1, y1)`` inclusive in image coordinates.
    """
    height, width = shape
    frame = np.full((height, width, 3), (30, 30, 30), dtype=np.uint8)
    depth = np.full((height, width), desk_depth, dtype=np.float32)
    cv2.rectangle(frame, (0, 0), (width - 1, height - 1), (235, 235, 235), cv2.FILLED)
    if box_rect is not None:
        x0, y0, x1, y1 = box_rect
        cv2.rectangle(frame, (x0, y0), (x1, y1), (245, 245, 245), cv2.FILLED)
        depth[y0 : y1 + 1, x0 : x1 + 1] = box_depth
    for x0, y0, x1, y1 in tape_rects:
        cv2.rectangle(frame, (x0, y0), (x1, y1), (240, 240, 240), cv2.FILLED)
        depth[y0 : y1 + 1, x0 : x1 + 1] = tape_depth
    return frame, depth


def run_scene(frame, depth, stride=2, rng=None):
    """Run the depth-first white-top pipeline on a synthetic scene.

    Returns ``(candidate_mask, plane, surfaces)``. Mirrors color_callback:
    white mask, one global support plane, elevation mask, white & elevated,
    connected-component candidates.
    """
    if rng is None:
        rng = np.random.default_rng(0)
    white_mask = build_white_mask(frame, 90, 170, 150, 11, 3)
    rows_grid, columns_grid = np.mgrid[
        0 : depth.shape[0] : stride, 0 : depth.shape[1] : stride
    ]
    all_rows = rows_grid.ravel()
    all_columns = columns_grid.ravel()
    depths = depth[all_rows, all_columns].astype(np.float64)
    valid_depth = np.isfinite(depths) & (depths >= 0.05) & (depths <= 2.0)
    if np.count_nonzero(valid_depth) < 60:
        return white_mask, None, []
    camera_points = deproject_pixels(
        all_rows[valid_depth], all_columns[valid_depth], depths[valid_depth],
        CAMERA_MATRIX,
    )
    all_world_points = np.full((all_rows.size, 3), np.nan, dtype=np.float64)
    all_world_points[valid_depth] = camera_points @ CAMERA_TO_WORLD.T
    plane = estimate_support_plane_ransac(
        all_world_points[valid_depth], 80, 0.008, math.radians(15), 60, 10000, rng
    )
    if plane is None:
        return white_mask, None, []
    heights = height_above_support(all_world_points, plane.normal, plane.d)
    elevated_valid = np.isfinite(heights) & (heights >= MIN_ELEVATION_M)
    grid_height, grid_width = rows_grid.shape
    elevated_small = np.zeros((grid_height, grid_width), dtype=np.uint8)
    elevated_small.ravel()[elevated_valid] = 255
    elevated_mask = cv2.resize(
        elevated_small,
        (depth.shape[1], depth.shape[0]),
        interpolation=cv2.INTER_NEAREST,
    )
    candidate_mask = cv2.bitwise_and(white_mask, elevated_mask)
    surfaces = extract_candidates(candidate_mask, 200, 8)
    return candidate_mask, plane, surfaces


# ---------------------------------------------------------------------------
# Kept: RANSAC / rectangle / angle statistics (still validate real behaviour).
# ---------------------------------------------------------------------------


def test_horizontal_ransac_rejects_vertical_outliers():
    rng = np.random.default_rng(7)
    xy = rng.uniform([-0.12, -0.08], [0.12, 0.08], size=(1500, 2))
    plane = np.column_stack((xy, rng.normal(0.34, 0.001, size=xy.shape[0])))
    vertical = np.column_stack(
        (
            rng.normal(0.2, 0.001, size=400),
            rng.uniform(-0.2, 0.2, size=400),
            rng.uniform(0.2, 0.45, size=400),
        )
    )
    points = np.vstack((plane, vertical))

    mask = fit_horizontal_plane_ransac(
        points,
        iterations=120,
        distance_threshold=0.005,
        max_tilt_rad=math.radians(10.0),
        rng=np.random.default_rng(8),
    )

    assert mask is not None
    assert np.count_nonzero(mask[:1500]) > 1450
    assert np.count_nonzero(mask[1500:]) < 30


def test_oriented_rectangle_recovers_long_axis_and_dimensions():
    yaw = math.radians(31.0)
    center = np.asarray([0.42, -0.03])
    local_x, local_y = np.meshgrid(
        np.linspace(-0.125, 0.125, 80),
        np.linspace(-0.085, 0.085, 55),
    )
    local = np.column_stack((local_x.ravel(), local_y.ravel()))
    rotation = np.asarray(
        [[math.cos(yaw), -math.sin(yaw)], [math.sin(yaw), math.cos(yaw)]]
    )
    points = local @ rotation.T + center

    points_3d = np.column_stack((points, np.full(points.shape[0], 0.25)))
    fit = min_area_box_top(points_3d)

    assert fit is not None
    assert np.allclose(fit.center_xy, center, atol=0.006)
    assert np.allclose(fit.dimensions_xy, [0.25, 0.17], atol=0.012)
    assert abs(axis_angle_error(fit.yaw, yaw)) < math.radians(2.0)


def test_tape_ring_with_center_strip_still_fits_outer_box():
    center = np.asarray([0.42, 0.01])
    yaw = math.radians(-24.0)
    x = np.linspace(-0.125, 0.125, 100)
    y = np.linspace(-0.085, 0.085, 70)
    thickness = 0.018
    local_x, local_y = np.meshgrid(x, y)
    tape = (
        (np.abs(local_x) >= 0.125 - thickness)
        | (np.abs(local_y) >= 0.085 - thickness)
        | (np.abs(local_x) <= 0.5 * thickness)
    )
    local = np.column_stack((local_x[tape], local_y[tape]))
    rotation = np.asarray(
        [[math.cos(yaw), -math.sin(yaw)], [math.sin(yaw), math.cos(yaw)]]
    )
    points = local @ rotation.T + center

    points_3d = np.column_stack((points, np.full(points.shape[0], 0.25)))
    fit = min_area_box_top(points_3d)

    assert fit is not None
    assert np.allclose(fit.center_xy, center, atol=0.008)
    assert np.allclose(fit.dimensions_xy, [0.25, 0.17], atol=0.015)
    assert abs(axis_angle_error(fit.yaw, yaw)) < math.radians(3.0)


def test_min_area_rectangle_keeps_box_axis_with_asymmetric_occlusion():
    """A missing top patch must not rotate yaw toward the visible centroid."""
    center = np.asarray([0.42, -0.02])
    yaw = math.radians(28.0)
    local_x, local_y = np.meshgrid(
        np.linspace(-0.125, 0.125, 100),
        np.linspace(-0.085, 0.085, 70),
    )
    visible = ~(
        (local_x > -0.08)
        & (local_x < 0.10)
        & (local_y > -0.055)
        & (local_y < 0.075)
    )
    local = np.column_stack((local_x[visible], local_y[visible]))
    rotation = np.asarray(
        [[math.cos(yaw), -math.sin(yaw)], [math.sin(yaw), math.cos(yaw)]]
    )
    points = local @ rotation.T + center
    points_3d = np.column_stack((points, np.full(points.shape[0], 0.25)))

    fit = min_area_box_top(points_3d)

    assert fit is not None
    assert np.allclose(fit.center_xy, center, atol=0.006)
    assert np.allclose(fit.dimensions_xy, [0.25, 0.17], atol=0.012)
    assert abs(axis_angle_error(fit.yaw, yaw)) < math.radians(2.0)


def test_min_area_rectangle_rejects_invalid_clouds():
    assert min_area_box_top(np.empty((0, 3))) is None
    assert min_area_box_top(np.zeros((10, 2))) is None
    assert min_area_box_top(np.zeros((10, 3))) is None


def test_axis_angle_statistics_cross_ninety_degree_boundary():
    angles = np.radians([89.0, -89.0, 88.0, -88.0])
    average = mean_axis_yaw(angles)

    assert abs(abs(math.degrees(average)) - 90.0) < 1.0e-6
    assert abs(axis_angle_error(math.radians(-89.0), math.radians(89.0))) < math.radians(3)
    assert -0.5 * math.pi <= normalize_axis_yaw(average) < 0.5 * math.pi


# ---------------------------------------------------------------------------
# New: global support plane / height-above-support (support height + Test H).
# ---------------------------------------------------------------------------


def test_support_plane_estimates_desk_height_and_box_elevation():
    rng = np.random.default_rng(11)
    xy = rng.uniform([-0.3, -0.3], [0.3, 0.3], size=(4000, 2))
    support = np.column_stack((xy, rng.normal(-0.5, 0.0015, size=xy.shape[0])))
    box_xy = rng.uniform([-0.12, -0.085], [0.12, 0.085], size=(900, 2))
    top = np.column_stack((box_xy, rng.normal(-0.25, 0.0015, size=box_xy.shape[0])))

    plane = estimate_support_plane_ransac(
        np.vstack((support, top)),
        80,
        0.008,
        math.radians(15),
        120,
        10000,
        np.random.default_rng(12),
    )

    assert plane is not None
    assert abs(float(plane.normal[2])) > 0.99
    assert np.isclose(plane.z_at(np.zeros(2)), -0.5, atol=0.003)
    heights = height_above_support(top, plane.normal, plane.d)
    assert np.isclose(float(np.median(heights)), 0.25, atol=0.005)


def test_support_plane_handles_tilted_surface():
    # Test H: a slightly tilted support surface must still produce a correct
    # height-above-support for an object standing on it.
    rng = np.random.default_rng(21)
    xy = rng.uniform([-0.3, -0.3], [0.3, 0.3], size=(4000, 2))
    tilt = 0.02
    support_z = 0.50 + tilt * xy[:, 0]
    support = np.column_stack(
        (xy[:, 0], xy[:, 1], rng.normal(support_z, 0.0015, size=xy.shape[0]))
    )
    box_xy = rng.uniform([-0.1, -0.08], [0.1, 0.08], size=(800, 2))
    top_z = 0.50 + tilt * box_xy[:, 0] + 0.25
    top = np.column_stack(
        (box_xy[:, 0], box_xy[:, 1], rng.normal(top_z, 0.0015, size=box_xy.shape[0]))
    )

    plane = estimate_support_plane_ransac(
        np.vstack((support, top)),
        100,
        0.008,
        math.radians(15),
        120,
        10000,
        np.random.default_rng(22),
    )

    assert plane is not None
    assert abs(float(plane.normal[2])) > 0.98
    heights = height_above_support(top, plane.normal, plane.d)
    assert np.isclose(float(np.median(heights)), 0.25, atol=0.008)


def test_support_plane_rejects_cloud_with_too_few_points():
    # Invalid/sparse depth must reject the frame instead of guessing a plane.
    points = np.zeros((5, 3))
    plane = estimate_support_plane_ransac(
        points, 60, 0.008, math.radians(15), 60, 10000, np.random.default_rng(3)
    )
    assert plane is None


# ---------------------------------------------------------------------------
# New: depth-first white-top pipeline scenarios (Tests A-G).
# ---------------------------------------------------------------------------


def test_white_ground_is_not_a_candidate():
    # Test A: a white support surface at height ~0 must never be a candidate.
    frame, depth = make_scene(box_rect=None)
    candidate_mask, plane, surfaces = run_scene(frame, depth)

    assert plane is not None
    assert np.count_nonzero(candidate_mask) == 0
    assert surfaces == []


def test_thin_white_debris_is_not_a_candidate():
    # Test B: a thin white object only slightly above the support must not
    # become a candidate even though it is white.
    frame, depth = make_scene(box_rect=None, tape_rects=[(10, 200, 90, 210)])
    candidate_mask, plane, surfaces = run_scene(frame, depth)

    assert plane is not None
    assert candidate_mask[205, 50] == 0  # tape location is removed
    assert surfaces == []


def test_real_white_box_is_a_candidate():
    # Test C: a real white box clearly above the support becomes a candidate.
    box_rect = (100, 60, 220, 180)
    frame, depth = make_scene(box_rect=box_rect)
    candidate_mask, plane, surfaces = run_scene(frame, depth)

    assert plane is not None
    assert len(surfaces) == 1
    assert not surfaces[0].touches_border
    center_row = (box_rect[1] + box_rect[3]) // 2
    center_column = (box_rect[0] + box_rect[2]) // 2
    assert candidate_mask[center_row, center_column] != 0
    assert candidate_mask[10, 10] == 0  # white desk outside the box is removed


def test_white_ground_with_box_selects_box():
    # Test D: white ground + center box -> ground removed, box selected.
    box_rect = (100, 60, 220, 180)
    frame, depth = make_scene(box_rect=box_rect)
    candidate_mask, plane, surfaces = run_scene(frame, depth)

    assert plane is not None
    assert len(surfaces) == 1
    center_row = (box_rect[1] + box_rect[3]) // 2
    center_column = (box_rect[0] + box_rect[2]) // 2
    assert surfaces[0].mask[center_row, center_column] == 255
    # White desk is deleted by the elevation step.
    assert np.count_nonzero(candidate_mask) < np.count_nonzero(
        build_white_mask(frame, 90, 170, 150, 11, 3)
    )


def test_white_ground_tape_and_box_selects_box():
    # Test E (most important regression): white ground + tape + box. Ground
    # and tape must not enter the candidates; only the box top remains.
    box_rect = (100, 60, 220, 180)
    frame, depth = make_scene(
        box_rect=box_rect, tape_rects=[(10, 200, 90, 210)]
    )
    candidate_mask, plane, surfaces = run_scene(frame, depth)

    assert plane is not None
    assert candidate_mask[10, 10] == 0  # white ground: no candidate
    assert candidate_mask[205, 50] == 0  # tape: no candidate
    assert len(surfaces) == 1  # only the box top remains
    assert not surfaces[0].touches_border
    center_row = (box_rect[1] + box_rect[3]) // 2
    center_column = (box_rect[0] + box_rect[2]) // 2
    assert surfaces[0].mask[center_row, center_column] == 255


def test_box_top_touching_image_border_is_rejected():
    # Test F: a real box top cropped by the image border must be flagged for
    # REJECT_BORDER; the full footprint cannot be recovered.
    frame, depth = make_scene(box_rect=(0, 60, 220, 140))
    candidate_mask, plane, surfaces = run_scene(frame, depth)

    assert plane is not None
    assert len(surfaces) >= 1
    assert all(surface.touches_border for surface in surfaces)


def test_blue_cube_hole_footprint_is_recovered():
    # Test G: the blue cube on the box top creates a hole in the white mask;
    # the filled external contour must restore the full top footprint.
    box_rect = (100, 60, 220, 180)
    frame, depth = make_scene(box_rect=box_rect)
    frame[100:141, 140:181] = (240, 100, 20)  # blue cube hole
    candidate_mask, plane, surfaces = run_scene(frame, depth)

    assert plane is not None
    assert len(surfaces) == 1
    assert surfaces[0].mask[120, 160] == 255  # hole filled by external contour
    assert surfaces[0].pixel_area > 0


# ---------------------------------------------------------------------------
# New: simple selection and white-mask gates.
# ---------------------------------------------------------------------------


def test_largest_3d_footprint_wins():
    small = np.asarray([0.0, 0.0, 0.30, 0.0, 0.10, 0.08, 0.02, 0.28])
    large = np.asarray([0.0, 0.0, 0.30, 0.0, 0.25, 0.17, 0.02, 0.28])
    selected_surface, selected_sample = choose_largest_valid_top(
        [("small", small), ("large", large)]
    )
    assert selected_surface == "large"
    assert np.allclose(selected_sample, large)


def test_white_mask_rejects_strongly_colored_large_region():
    # A saturated blue region must never survive the white mask, even when it
    # is the largest blob in the image.
    frame = np.zeros((240, 320, 3), dtype=np.uint8)
    cv2.rectangle(frame, (100, 80), (220, 160), (255, 0, 0), cv2.FILLED)
    mask = build_white_mask(frame, 90, 170, 150, 11, 3)
    assert np.count_nonzero(mask) == 0
    assert extract_candidates(mask, 200, 8) == []


def test_white_mask_filters_small_blob_by_minimum_pixels():
    # A small white blob (arm / fixture) is dropped by the minimum pixel area.
    frame = np.full((240, 320, 3), (30, 30, 30), dtype=np.uint8)
    cv2.rectangle(frame, (150, 110), (180, 130), (240, 240, 240), cv2.FILLED)
    mask = build_white_mask(frame, 90, 170, 150, 11, 3)
    assert extract_candidates(mask, 2000, 8) == []


def test_stable_estimate_uses_median_and_circular_yaw():
    # LOCK-related: the stable estimate is the per-field median with a
    # circular mean for the rectangle axis yaw.
    samples = np.asarray(
        [
            [0.300, 0.010, 0.250, math.radians(89.0), 0.250, 0.170, 0.020, 0.230],
            [0.300, 0.010, 0.250, math.radians(-89.0), 0.250, 0.170, 0.020, 0.230],
            [0.300, 0.010, 0.250, math.radians(88.0), 0.250, 0.170, 0.020, 0.230],
        ]
    )
    estimate = BoxDetector.stable_estimate(samples)

    assert np.allclose(estimate[:2], [0.300, 0.010])
    assert np.isclose(estimate[2], 0.250)
    assert np.allclose(estimate[4:6], [0.250, 0.170])
    # [89, -89, 88] degrees have a true circular axis mean of ~89.3 degrees.
    assert abs(abs(math.degrees(estimate[3])) - 90.0) < 2.0


def test_large_yaw_or_position_jump_requires_relock():
    previous = np.asarray(
        [0.30, 0.01, 0.25, math.radians(-7.0), 0.24, 0.17, 0.01, 0.24]
    )
    stable = previous.copy()
    stable[0] += 0.002
    yaw_outlier = previous.copy()
    yaw_outlier[3] = math.radians(84.0)
    moved = previous.copy()
    moved[1] += 0.06

    assert not measurement_jumped(
        stable, previous, 0.04, math.radians(12.0)
    )
    assert measurement_jumped(
        yaw_outlier, previous, 0.04, math.radians(12.0)
    )
    assert measurement_jumped(
        moved, previous, 0.04, math.radians(12.0)
    )


def test_locked_jump_requires_three_consecutive_confirmations():
    assert defer_locked_jump(True, 1, 3)
    assert defer_locked_jump(True, 2, 3)
    assert not defer_locked_jump(True, 3, 3)
    assert not defer_locked_jump(False, 1, 3)


def test_committed_box_snapshot_checks_every_planning_scene_field():
    locked = np.asarray(
        [0.30, 0.01, 0.25, math.radians(8.0), 0.25, 0.17, 0.01, 0.24]
    )
    noise = locked.copy()
    noise[0] += 0.004
    noise[4] += 0.009
    noise[3] += math.radians(2.0)
    assert not locked_box_measurement_changed(
        noise, locked, 0.008, 0.015, math.radians(5.0)
    )

    for index, amount in ((0, 0.009), (2, 0.009), (6, 0.009),
                          (4, 0.016), (5, 0.016), (7, 0.016)):
        changed = locked.copy()
        changed[index] += amount
        assert locked_box_measurement_changed(
            changed, locked, 0.008, 0.015, math.radians(5.0)
        )

    rotated = locked.copy()
    rotated[3] += math.radians(6.0)
    assert locked_box_measurement_changed(
        rotated, locked, 0.008, 0.015, math.radians(5.0)
    )
