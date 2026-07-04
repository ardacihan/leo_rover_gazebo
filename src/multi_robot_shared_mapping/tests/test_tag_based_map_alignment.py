"""Unit tests for tag-based 2D map alignment."""

import math

import pytest

from multi_robot_shared_mapping.tag_map_alignment import (
    apply_2d_transform,
    estimate_2d_transform,
    estimate_transform_from_single_tag,
    transform_points,
)


def _rotate_points(points, dx, dy, yaw):
    return transform_points(points, dx, dy, yaw)


def test_perfect_transform():
    source = [(0.0, 0.0), (1.0, 0.0), (0.0, 1.0)]
    dx, dy, yaw = 2.36, -11.27, 0.1
    target = _rotate_points(source, dx, dy, yaw)

    estimate = estimate_2d_transform(
        source,
        target,
        ground_truth=(dx, dy, yaw),
    )

    assert estimate.success
    assert estimate.num_tags == 3
    assert estimate.translation_error is not None
    assert estimate.yaw_error is not None
    assert estimate.translation_error < 1e-6
    assert estimate.yaw_error < 1e-6
    assert estimate.mean_reprojection_error < 1e-6


def test_noisy_detections():
    source = [(0.0, 0.0), (2.0, 0.0), (0.0, 2.0), (2.0, 2.0)]
    dx, dy, yaw = 2.36, -11.27, 0.05
    target = _rotate_points(source, dx, dy, yaw)

    noisy_target = [
        (x + 0.05, y - 0.04) for x, y in target
    ]

    estimate = estimate_2d_transform(
        source,
        noisy_target,
        ground_truth=(dx, dy, yaw),
    )

    assert estimate.success
    assert estimate.translation_error < 0.25
    assert estimate.yaw_error < math.radians(5.0)


def test_missing_tag_still_estimates_with_lower_confidence():
    source = [(0.0, 0.0), (1.0, 0.0), (0.0, 1.0)]
    dx, dy, yaw = 1.0, -2.0, 0.0
    target_full = _rotate_points(source, dx, dy, yaw)

    estimate = estimate_2d_transform(
        source[:2],
        target_full[:2],
        ground_truth=(dx, dy, yaw),
    )

    assert estimate.success
    assert estimate.num_tags == 2
    assert estimate.confidence <= 0.6


def test_single_common_tag_weak_hint():
    """Single tag yields a transform hint but estimate_2d_transform still rejects."""
    dx, dy, yaw = 2.0, -1.0, 0.1
    source = (1.0, 2.0, 0.0)
    tx, ty = apply_2d_transform(source[0], source[1], dx, dy, yaw)
    hint_dx, hint_dy, hint_yaw = estimate_transform_from_single_tag(
        tx, ty, yaw, source[0], source[1], source[2]
    )
    assert abs(hint_dx - dx) < 1e-5
    assert abs(hint_dy - dy) < 1e-5
    assert abs(hint_yaw - yaw) < 1e-5


def test_single_common_tag_fails():
    source = [(1.0, 2.0)]
    target = [(3.0, -9.0)]

    estimate = estimate_2d_transform(source, target)

    assert not estimate.success
    assert "at least 2" in estimate.message.lower()


def test_outlier_is_rejected():
    source = [(0.0, 0.0), (2.0, 0.0), (0.0, 2.0), (2.0, 2.0)]
    dx, dy, yaw = 2.36, -11.27, 0.0
    target = _rotate_points(source, dx, dy, yaw)
    bad_target = list(target)
    bad_target[2] = (bad_target[2][0] + 2.0, bad_target[2][1] + 2.0)

    estimate = estimate_2d_transform(source, bad_target)

    assert not estimate.success
    assert estimate.mean_reprojection_error > 0.35
    assert "rejected" in estimate.message.lower()


def test_apply_2d_transform_matches_estimate():
    source = [(1.5, -0.5), (0.0, 2.0)]
    dx, dy, yaw = 2.36, -11.27, 0.2
    target = transform_points(source, dx, dy, yaw)

    estimate = estimate_2d_transform(source, target)

    assert estimate.success
    assert abs(estimate.dx - dx) < 1e-5
    assert abs(estimate.dy - dy) < 1e-5
    assert abs(estimate.yaw - yaw) < 1e-5
