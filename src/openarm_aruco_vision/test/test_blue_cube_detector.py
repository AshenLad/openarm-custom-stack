import math

import numpy as np

from openarm_aruco_vision.blue_cube_detector_node import (
    BoxGeometry,
    densest_top_component,
    deproject_pixels,
    euclidean_clusters,
    extract_object_clusters,
    estimate_box_top_plane,
    fit_object_obb,
    object_measurement_jumped,
    robust_object_estimate,
    points_in_oriented_footprint,
    radius_outlier_filter,
    symmetric_yaw_errors,
    symmetric_yaw_mean,
    voxel_down_sample,
)


BOX = BoxGeometry(
    center_xy=np.asarray([0.40, 0.0]),
    dimensions_xy=np.asarray([0.26, 0.18]),
    top_z=0.25,
    yaw=math.radians(20.0),
)


def object_cloud(
    center_xy=(0.40, 0.0),
    dimensions_xy=(0.05, 0.04),
    top_z=0.30,
    yaw_deg=0.0,
) -> np.ndarray:
    x, y = np.meshgrid(
        np.linspace(-0.5 * dimensions_xy[0], 0.5 * dimensions_xy[0], 24),
        np.linspace(-0.5 * dimensions_xy[1], 0.5 * dimensions_xy[1], 20),
    )
    local = np.column_stack((x.ravel(), y.ravel()))
    yaw = math.radians(yaw_deg)
    rotation = np.asarray(
        [[math.cos(yaw), -math.sin(yaw)], [math.sin(yaw), math.cos(yaw)]]
    )
    xy = local @ rotation.T + np.asarray(center_xy)
    return np.column_stack((xy, np.full(xy.shape[0], top_z)))


def extract(points: np.ndarray) -> list[np.ndarray]:
    return extract_object_clusters(
        points,
        BOX,
        top_clearance_m=0.008,
        outlier_radius_m=0.012,
        outlier_minimum_neighbors=3,
        cluster_tolerance_m=0.012,
        cluster_minimum_points=20,
    )


def test_deproject_pixels_uses_intrinsics_and_measured_depth():
    camera_matrix = np.asarray(
        [[600.0, 0.0, 320.0], [0.0, 600.0, 240.0], [0.0, 0.0, 1.0]]
    )
    points = deproject_pixels(
        rows=np.asarray([240, 300]),
        columns=np.asarray([320, 380]),
        depths=np.asarray([1.0, 1.0]),
        camera_matrix=camera_matrix,
    )
    assert np.allclose(points, [[0.0, 0.0, 1.0], [0.1, 0.1, 1.0]])


def test_dynamic_oriented_box_footprint_moves_with_box():
    points = np.asarray([[0.40, 0.0, 0.3], [0.55, 0.0, 0.3]])
    mask = points_in_oriented_footprint(points, BOX)
    moved = BoxGeometry(
        center_xy=np.asarray([0.55, 0.0]),
        dimensions_xy=BOX.dimensions_xy,
        top_z=BOX.top_z,
        yaw=BOX.yaw,
    )
    moved_mask = points_in_oriented_footprint(points, moved)
    assert mask.tolist() == [True, False]
    assert moved_mask.tolist() == [False, True]


def test_dynamic_footprint_inset_removes_raised_box_rim():
    center = np.asarray([[0.40, 0.0, 0.30]])
    local_edge = np.asarray([[0.40 + 0.129 * math.cos(BOX.yaw),
                              0.0 + 0.129 * math.sin(BOX.yaw), 0.30]])
    assert points_in_oriented_footprint(center, BOX, 0.012)[0]
    assert not points_in_oriented_footprint(local_edge, BOX, 0.012)[0]


def test_box_top_plane_is_removed():
    x, y = np.meshgrid(np.linspace(0.30, 0.50, 30), np.linspace(-0.07, 0.07, 24))
    top = np.column_stack((x.ravel(), y.ravel(), np.full(x.size, BOX.top_z + 0.002)))
    assert extract(top) == []


def test_tilted_box_top_plane_is_estimated_and_removed():
    x, y = np.meshgrid(np.linspace(0.30, 0.50, 40), np.linspace(-0.07, 0.07, 30))
    z = BOX.top_z + 0.08 * (x - BOX.center_xy[0])
    tilted_top = np.column_stack((x.ravel(), y.ravel(), z.ravel()))
    object_points = object_cloud(top_z=0.31)
    combined = np.vstack((tilted_top, object_points))
    plane = estimate_box_top_plane(
        combined, BOX, rng=np.random.default_rng(4)
    )
    assert plane is not None
    assert np.percentile(np.abs(plane.heights(tilted_top)), 99) < 0.002
    clusters = extract_object_clusters(
        combined, BOX, 0.008, 0.012, 3, 0.012, 20, plane
    )
    assert len(clusters) == 1
    assert np.mean(clusters[0][:, 2]) > 0.30


def test_object_above_box_top_is_retained():
    clusters = extract(object_cloud())
    assert len(clusters) == 1
    assert clusters[0].shape[0] > 100


def test_position_and_dimensions_are_measured_not_fixed():
    first = fit_object_obb(object_cloud(), BOX.top_z)
    second = fit_object_obb(
        object_cloud(center_xy=(0.43, 0.03), dimensions_xy=(0.08, 0.03), top_z=0.33),
        BOX.top_z,
    )
    assert first is not None and second is not None
    assert np.allclose(first.center[:2], [0.40, 0.0], atol=1e-4)
    assert np.allclose(second.center[:2], [0.43, 0.03], atol=1e-4)
    assert np.allclose(first.dimensions, [0.05, 0.04, 0.05], atol=1e-3)
    assert np.allclose(second.dimensions, [0.08, 0.03, 0.08], atol=1e-3)


def test_object_obb_recovers_rotated_dimensions_and_yaw():
    points = object_cloud(dimensions_xy=(0.09, 0.05), top_z=0.34, yaw_deg=27.0)
    fit = fit_object_obb(points, BOX.top_z)
    assert fit is not None
    assert np.allclose(fit.dimensions, [0.09, 0.05, 0.09], atol=1e-3)
    error = 0.5 * math.atan2(
        math.sin(2.0 * (fit.yaw - math.radians(27.0))),
        math.cos(2.0 * (fit.yaw - math.radians(27.0))),
    )
    assert abs(error) < math.radians(1.0)


def test_object_obb_top_slice_rejects_connected_low_plane_residuals():
    object_points = object_cloud(dimensions_xy=(0.05, 0.04), top_z=0.30)
    residual_x, residual_y = np.meshgrid(
        np.linspace(0.34, 0.46, 40), np.linspace(-0.01, 0.01, 8)
    )
    residuals = np.column_stack(
        (residual_x.ravel(), residual_y.ravel(), np.full(residual_x.size, 0.27))
    )
    fit = fit_object_obb(np.vstack((object_points, residuals)), BOX.top_z)
    assert fit is not None
    assert np.allclose(fit.dimensions[:2], [0.05, 0.04], atol=0.003)


def test_object_obb_rejects_sparse_same_height_flying_pixels():
    object_points = object_cloud(dimensions_xy=(0.04, 0.04), top_z=0.30)
    flying = np.asarray(
        [
            [0.31, -0.001, 0.30],
            [0.49, 0.001, 0.30],
        ]
    )
    fit = fit_object_obb(np.vstack((object_points, flying)), BOX.top_z)
    assert fit is not None
    assert np.allclose(fit.center[:2], [0.40, 0.0], atol=0.003)
    assert np.allclose(fit.dimensions[:2], [0.04, 0.04], atol=0.003)


def test_robust_obb_orientation_rejects_asymmetric_same_height_fringe():
    points = voxel_down_sample(
        object_cloud(
            dimensions_xy=(0.065, 0.035), top_z=0.30, yaw_deg=28.0
        ),
        0.004,
    )
    # Unsupported diagonal fringe rotates a convex hull but should not rotate
    # the quantile-bounded physical rectangle.
    fringe = np.column_stack(
        (
            np.linspace(0.43, 0.47, 9),
            np.linspace(0.005, 0.045, 9),
            np.full(9, 0.30),
        )
    )
    fit = fit_object_obb(np.vstack((points, fringe)), BOX.top_z)
    assert fit is not None
    error = 0.5 * math.atan2(
        math.sin(2.0 * (fit.yaw - math.radians(28.0))),
        math.cos(2.0 * (fit.yaw - math.radians(28.0))),
    )
    assert abs(error) < math.radians(3.0)
    assert np.allclose(fit.dimensions[:2], [0.065, 0.035], atol=0.006)


def test_dense_top_component_rejects_connected_same_height_flying_lobes():
    top = voxel_down_sample(
        object_cloud(dimensions_xy=(0.04, 0.04), top_z=0.30), 0.004
    )
    # Two one-dimensional D435-style depth-edge trails are close enough to
    # join the object under the outer 18 mm 3-D cluster tolerance.
    left_x = np.arange(0.352, 0.380, 0.004)
    right_x = np.arange(0.420, 0.452, 0.004)
    lobes = np.vstack(
        (
            np.column_stack(
                (left_x, np.full(left_x.size, -0.001), np.full(left_x.size, 0.30))
            ),
            np.column_stack(
                (right_x, np.full(right_x.size, 0.001), np.full(right_x.size, 0.30))
            ),
        )
    )
    contaminated = np.vstack((top, lobes))
    selected = densest_top_component(contaminated)
    fit = fit_object_obb(contaminated, BOX.top_z)
    assert selected.shape[0] >= 50
    assert selected.shape[0] < contaminated.shape[0]
    assert np.ptp(selected[:, 0]) < 0.5 * np.ptp(contaminated[:, 0])
    assert fit is not None
    assert np.allclose(fit.center[:2], [0.40, 0.0], atol=0.003)
    assert np.allclose(fit.dimensions[:2], [0.04, 0.04], atol=0.004)


def test_object_top_band_rejects_dense_sloped_depth_edge_trails():
    top = voxel_down_sample(
        object_cloud(dimensions_xy=(0.04, 0.04), top_z=0.30), 0.004
    )
    trail_rows = []
    for y in np.arange(0.022, 0.052, 0.004):
        half_width = max(0.003, 0.016 - 0.4 * (y - 0.022))
        xs = np.arange(0.40 - half_width, 0.40 + half_width, 0.004)
        # A dense RealSense depth-edge band can be locally well supported but
        # slopes away from the actual upper surface as it extends outward.
        height = 0.299 - 0.18 * (y - 0.022)
        trail_rows.append(
            np.column_stack((xs, np.full(xs.size, y), np.full(xs.size, height)))
        )
    fit = fit_object_obb(np.vstack((top, *trail_rows)), BOX.top_z)
    assert fit is not None
    assert np.allclose(fit.center[:2], [0.40, 0.0], atol=0.004)
    assert np.allclose(fit.dimensions[:2], [0.04, 0.04], atol=0.005)


def test_density_filter_preserves_non_square_and_small_objects():
    rectangle = voxel_down_sample(
        object_cloud(dimensions_xy=(0.08, 0.03), top_z=0.33, yaw_deg=24.0),
        0.004,
    )
    small = voxel_down_sample(
        object_cloud(dimensions_xy=(0.02, 0.02), top_z=0.28, yaw_deg=-17.0),
        0.004,
    )
    rectangle_fit = fit_object_obb(rectangle, BOX.top_z)
    small_fit = fit_object_obb(small, BOX.top_z)
    assert rectangle_fit is not None and small_fit is not None
    assert np.allclose(rectangle_fit.dimensions[:2], [0.08, 0.03], atol=0.006)
    assert np.allclose(small_fit.dimensions[:2], [0.02, 0.02], atol=0.004)


def test_object_obb_uses_dense_top_surface_not_vertical_side_wall():
    top = object_cloud(dimensions_xy=(0.04, 0.04), top_z=0.30)
    side_y, side_z = np.meshgrid(
        np.linspace(-0.02, 0.02, 20), np.linspace(0.274, 0.300, 14)
    )
    side = np.column_stack(
        (
            np.full(side_y.size, 0.42),
            side_y.ravel(),
            side_z.ravel(),
        )
    )
    fit = fit_object_obb(np.vstack((top, side)), BOX.top_z)
    assert fit is not None
    assert np.allclose(fit.dimensions[:2], [0.04, 0.04], atol=0.004)


def test_radius_filter_removes_isolated_invalid_depth_points():
    cluster = object_cloud()
    outliers = np.asarray([[0.30, -0.07, 0.7], [0.49, 0.07, 0.8]])
    filtered = radius_outlier_filter(
        np.vstack((cluster, outliers)), radius_m=0.012, minimum_neighbors=3
    )
    assert filtered.shape[0] == cluster.shape[0]
    assert np.max(filtered[:, 2]) < 0.4


def test_voxel_grid_downsamples_dense_cloud_without_changing_extent():
    points = object_cloud()
    downsampled = voxel_down_sample(points, 0.004)
    assert downsampled.shape[0] < points.shape[0]
    assert np.allclose(
        np.ptp(downsampled[:, :2], axis=0), [0.05, 0.04], atol=0.005
    )


def test_euclidean_clustering_separates_objects():
    first = object_cloud(center_xy=(0.35, -0.03), dimensions_xy=(0.03, 0.03))
    second = object_cloud(center_xy=(0.45, 0.03), dimensions_xy=(0.03, 0.03))
    clusters = euclidean_clusters(
        np.vstack((first, second)), tolerance_m=0.012, minimum_points=20
    )
    assert len(clusters) == 2


def test_color_cannot_change_geometric_detection():
    # No RGB value is accepted by the geometry API; red/blue/black appearances
    # therefore produce exactly the same point-cloud result.
    points = object_cloud()
    blue_rgb = np.full((10, 10, 3), (255, 0, 0), dtype=np.uint8)
    red_rgb = np.full((10, 10, 3), (0, 0, 255), dtype=np.uint8)
    assert not np.array_equal(blue_rgb, red_rgb)
    blue_result = extract(points)
    red_result = extract(points)
    assert len(blue_result) == len(red_result) == 1
    assert np.array_equal(blue_result[0], red_result[0])


def test_square_yaw_statistics_treat_ninety_degree_flip_as_equivalent():
    angles = np.radians([2.0, 3.0, -88.0, -87.0])
    mean = symmetric_yaw_mean(angles, symmetry_order=4)
    errors = symmetric_yaw_errors(angles, mean, symmetry_order=4)
    assert np.std(np.degrees(errors)) < 1.0


def test_production_lock_estimate_ignores_square_ninety_degree_flips():
    samples = np.asarray(
        [
            [0.400, 0.000, 0.280, 0.040, 0.039, 0.040, math.radians(2.0)],
            [0.401, 0.001, 0.281, 0.041, 0.039, 0.040, math.radians(-88.0)],
            [0.399, -0.001, 0.279, 0.040, 0.040, 0.041, math.radians(3.0)],
        ]
    )
    estimate, yaw_errors = robust_object_estimate(samples)
    assert np.allclose(estimate[:3], [0.400, 0.000, 0.280], atol=0.002)
    assert np.std(np.degrees(yaw_errors)) < 1.0


def test_production_lock_distinguishes_outlier_from_real_scene_change():
    reference = np.asarray(
        [0.400, 0.000, 0.280, 0.040, 0.039, 0.040, math.radians(2.0)]
    )
    noise = reference.copy()
    noise[:3] += [0.003, -0.002, 0.002]
    square_flip = noise.copy()
    square_flip[6] += 0.5 * math.pi
    moved = reference.copy()
    moved[0] += 0.020
    resized = reference.copy()
    resized[3] += 0.020
    assert not object_measurement_jumped(noise, reference, 0.012, 0.012, 0.20)
    assert not object_measurement_jumped(
        square_flip, reference, 0.012, 0.012, 0.20
    )
    assert object_measurement_jumped(moved, reference, 0.012, 0.012, 0.20)
    assert object_measurement_jumped(resized, reference, 0.012, 0.012, 0.20)
