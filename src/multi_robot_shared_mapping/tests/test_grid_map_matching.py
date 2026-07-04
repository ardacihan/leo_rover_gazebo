#!/usr/bin/env python3
"""Unit tests for pure occupancy-grid matching (no ROS required)."""

import math
import os
import sys

import numpy as np
import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from multi_robot_shared_mapping.grid_map_matching import (  # noqa: E402
    build_lookup_grid,
    downsample_points,
    match_maps,
    occupancy_grid_to_points,
)


def make_room_points():
    """L-shaped wall outline, asymmetric so yaw is observable.  # shape: (N, 2)"""
    xs = np.arange(0.0, 5.0, 0.1)
    ys = np.arange(0.0, 3.0, 0.1)
    wall_bottom = np.column_stack([xs, np.zeros_like(xs)])
    wall_left = np.column_stack([np.zeros_like(ys), ys])
    wall_stub = np.column_stack([np.full(10, 2.5), np.arange(0.0, 1.0, 0.1)])
    return np.vstack([wall_bottom, wall_left, wall_stub])


def apply_transform(points, dx, dy, yaw):
    c, s = math.cos(yaw), math.sin(yaw)
    rot = np.array([[c, -s], [s, c]])
    return points @ rot.T + np.array([dx, dy])


def test_occupancy_grid_to_points_extracts_occupied_cells():
    # 2x3 grid: only one occupied cell at (ix=2, iy=1).
    data = [0, -1, 0, 0, 0, 100]
    points = occupancy_grid_to_points(
        data, width=3, height=2, resolution=1.0, origin_x=10.0, origin_y=20.0
    )
    assert points.shape == (1, 2)
    assert points[0] == pytest.approx([12.5, 21.5])


def test_downsample_respects_max_points():
    points = np.random.default_rng(0).uniform(0, 10, size=(2000, 2))
    down = downsample_points(points, voxel_size=0.1, max_points=100)
    assert len(down) <= 100


def test_match_identity():
    points = make_room_points()
    result = match_maps(
        points, points,
        center=(0.0, 0.0, 0.0), xy_range=1.0, yaw_range=0.3,
        coarse_xy_step=0.25, coarse_yaw_step=math.radians(5.0),
    )
    assert result.success
    assert result.dx == pytest.approx(0.0, abs=0.15)
    assert result.dy == pytest.approx(0.0, abs=0.15)
    assert result.yaw == pytest.approx(0.0, abs=math.radians(3.0))
    assert result.normalized_overlap_score > 0.9


def test_match_recovers_known_offset():
    source = make_room_points()
    true_dx, true_dy, true_yaw = 1.2, -0.8, 0.25
    target = apply_transform(source, true_dx, true_dy, true_yaw)

    result = match_maps(
        source, target,
        center=(0.0, 0.0, 0.0), xy_range=3.0, yaw_range=math.radians(45.0),
        coarse_xy_step=0.5, coarse_yaw_step=math.radians(10.0),
    )
    assert result.success
    assert result.dx == pytest.approx(true_dx, abs=0.2)
    assert result.dy == pytest.approx(true_dy, abs=0.2)
    assert result.yaw == pytest.approx(true_yaw, abs=math.radians(5.0))


def test_match_empty_source_fails():
    result = match_maps(np.zeros((0, 2)), make_room_points())
    assert not result.success
    assert result.overlap_score == 0


def test_free_space_extraction():
    # free (0), occupied (100), unknown (-1)
    data = [0, 100, -1]
    free = occupancy_grid_to_points(
        data, width=3, height=1, resolution=1.0,
        origin_x=0.0, origin_y=0.0, select="free",
    )
    assert free.shape == (1, 2)
    assert free[0] == pytest.approx([0.5, 0.5])


def test_lookup_grid_free_space_penalty():
    # Two walls with free space between them (all within the grid extent;
    # free cells outside the occupied bounding box are ignored by design).
    occupied = np.array([[5.0, 5.0], [9.0, 5.0]])
    free = np.array([[7.0, 5.0]])
    grid, min_x, min_y = build_lookup_grid(
        occupied, resolution=1.0, free_points=free
    )
    occ_ix = int((5.0 - min_x) / 1.0)
    occ_iy = int((5.0 - min_y) / 1.0)
    free_ix = int((7.0 - min_x) / 1.0)
    assert grid[occ_iy, occ_ix] == 2          # exact occupied
    assert grid[occ_iy, occ_ix + 1] == 1      # dilated ring
    assert grid[occ_iy, free_ix] == -1        # free space penalized


def test_match_with_free_space_still_recovers_offset():
    source = make_room_points()
    target = apply_transform(source, 1.0, -0.5, 0.1)
    # Free space in the middle of the target room must not break matching.
    free_x, free_y = np.meshgrid(np.arange(1.0, 2.0, 0.2), np.arange(1.0, 2.0, 0.2))
    free = apply_transform(
        np.column_stack([free_x.ravel(), free_y.ravel()]), 1.0, -0.5, 0.1
    )

    result = match_maps(
        source, target,
        center=(0.0, 0.0, 0.0), xy_range=2.0, yaw_range=math.radians(30.0),
        coarse_xy_step=0.5, coarse_yaw_step=math.radians(10.0),
        target_free_points=free,
    )
    assert result.success
    assert result.dx == pytest.approx(1.0, abs=0.2)
    assert result.dy == pytest.approx(-0.5, abs=0.2)
    assert result.yaw == pytest.approx(0.1, abs=math.radians(5.0))
