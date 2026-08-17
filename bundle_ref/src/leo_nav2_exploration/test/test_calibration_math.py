import math

import numpy as np
import pytest

from leo_nav2_exploration.calibration_math import (
    camera_level_from_floor_normal,
    fit_plane_ransac,
    fit_plane_svd,
    linear_odometry_scale,
    normalize_angle,
    rotational_odometry_scale,
    sector_statistics,
    unwrap_angle_sequence,
)


def test_sector_statistics_filters_invalid_ranges_and_uses_median():
    ranges = [float("nan"), 0.1, 1.0, 1.1, 8.0, float("inf")]
    stats = sector_statistics(
        ranges=ranges,
        angle_min=-0.5,
        angle_increment=0.2,
        center_angle=0.0,
        half_width=0.5,
        minimum_range=0.2,
        maximum_range=5.0,
    )
    assert stats.count == 2
    assert stats.median == pytest.approx(1.05)
    assert stats.minimum == pytest.approx(1.0)


def test_fit_plane_svd_recovers_horizontal_plane_with_noise():
    rng = np.random.default_rng(4)
    x = rng.uniform(-1.0, 1.0, 500)
    z = rng.uniform(0.5, 3.0, 500)
    y = np.full_like(x, 0.72) + rng.normal(0.0, 0.002, x.size)
    points = np.column_stack([x, y, z])
    plane = fit_plane_svd(points)
    assert abs(abs(plane.normal[1]) - 1.0) < 0.01
    assert plane.rms_error < 0.005
    assert abs(plane.distance) == pytest.approx(0.72, abs=0.01)


def test_ransac_plane_rejects_outliers():
    rng = np.random.default_rng(12)
    x = rng.uniform(-1.0, 1.0, 500)
    z = rng.uniform(0.5, 3.0, 500)
    y = 0.65 + 0.05 * x + rng.normal(0.0, 0.003, x.size)
    inliers = np.column_stack([x, y, z])
    outliers = rng.uniform(-2.0, 2.0, (80, 3))
    plane = fit_plane_ransac(
        np.vstack([inliers, outliers]),
        distance_threshold=0.015,
        iterations=250,
        seed=5,
    )
    assert plane.inlier_count >= 470
    assert plane.rms_error < 0.01



def test_ransac_expected_normal_selects_floor_instead_of_larger_wall():
    rng = np.random.default_rng(23)
    # A large vertical wall would win unconstrained RANSAC.
    wall_y = rng.uniform(-0.8, 0.8, 900)
    wall_z = rng.uniform(0.2, 2.5, 900)
    wall_x = np.full_like(wall_y, 1.2) + rng.normal(0.0, 0.002, wall_y.size)
    wall = np.column_stack([wall_x, wall_y, wall_z])

    floor_x = rng.uniform(-1.0, 1.0, 550)
    floor_z = rng.uniform(0.3, 2.5, 550)
    floor_y = np.full_like(floor_x, 0.70) + rng.normal(0.0, 0.002, floor_x.size)
    floor = np.column_stack([floor_x, floor_y, floor_z])

    plane = fit_plane_ransac(
        np.vstack([wall, floor]),
        distance_threshold=0.012,
        iterations=400,
        seed=11,
        expected_normal=np.array([0.0, -1.0, 0.0]),
        maximum_normal_angle=math.radians(50.0),
    )
    assert plane.inlier_count >= 500
    assert plane.normal[1] < -0.98
    assert plane.distance == pytest.approx(0.70, abs=0.015)


def test_camera_level_angles_for_level_and_downward_pitched_camera():
    level = camera_level_from_floor_normal(np.array([0.0, -1.0, 0.0]))
    assert level.roll_rad == pytest.approx(0.0)
    assert level.pitch_down_rad == pytest.approx(0.0)

    alpha = math.radians(18.0)
    pitched = camera_level_from_floor_normal(
        np.array([0.0, -math.cos(alpha), math.sin(alpha)])
    )
    assert pitched.roll_rad == pytest.approx(0.0, abs=1e-8)
    assert pitched.pitch_down_rad == pytest.approx(alpha, abs=1e-8)


def test_angle_unwrap_crosses_pi_without_jump():
    wrapped = [math.radians(v) for v in (170, 179, -178, -170)]
    unwrapped = unwrap_angle_sequence(wrapped)
    degrees = [math.degrees(v) for v in unwrapped]
    assert degrees == pytest.approx([170, 179, 182, 190])
    assert normalize_angle(math.radians(190)) == pytest.approx(math.radians(-170))


def test_odometry_scale_factors_are_actual_over_reported():
    assert linear_odometry_scale(actual_distance=2.0, reported_distance=1.92) == pytest.approx(2.0 / 1.92)
    assert rotational_odometry_scale(
        actual_angle=2 * math.pi,
        reported_angle=math.radians(350),
    ) == pytest.approx(360 / 350)


@pytest.mark.parametrize("reported", [0.0, 1e-12])
def test_odometry_scale_rejects_zero_reported_motion(reported):
    with pytest.raises(ValueError):
        linear_odometry_scale(1.0, reported)


def test_fit_board_from_scan_recovers_front_board_distance_and_yaw():
    from leo_nav2_exploration.calibration_math import fit_board_from_scan

    angle_min = math.radians(-30.0)
    angle_increment = math.radians(1.0)
    angles = [angle_min + i * angle_increment for i in range(61)]
    ranges = []
    for angle in angles:
        if abs(angle) <= math.radians(20.0):
            ranges.append(1.0 / math.cos(angle))
        else:
            ranges.append(float("inf"))
    fit = fit_board_from_scan(
        ranges=ranges,
        angle_min=angle_min,
        angle_increment=angle_increment,
        expected_angle=0.0,
        half_width=math.radians(25.0),
        minimum_range=0.2,
        maximum_range=3.0,
        range_tolerance=0.25,
    )
    assert fit.distance == pytest.approx(1.0, abs=0.005)
    assert fit.normal_angle == pytest.approx(0.0, abs=0.005)
    assert fit.rms_error < 0.005


def test_quaternion_to_euler_recovers_yaw():
    from leo_nav2_exploration.calibration_math import quaternion_to_euler

    yaw = math.radians(37.0)
    roll, pitch, result_yaw = quaternion_to_euler(0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))
    assert roll == pytest.approx(0.0)
    assert pitch == pytest.approx(0.0)
    assert result_yaw == pytest.approx(yaw)
