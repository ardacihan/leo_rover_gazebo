"""Regression tests for rotated, real OccupancyGrid fusion geometry."""

import math
import os
import sys

import numpy as np
import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from multi_robot_shared_mapping.occupancy_fusion import (  # noqa: E402
    GridSpec,
    fuse_grids,
    grid_bounds,
)


def test_grid_bounds_honor_origin_orientation():
    spec = GridSpec(np.zeros((2, 4), np.int16), 1.0, 10.0, 20.0, math.pi / 2)
    assert grid_bounds(spec) == pytest.approx((8.0, 20.0, 10.0, 24.0))


def test_inverse_resampling_has_no_diagonal_holes_in_known_patch():
    source = GridSpec(np.zeros((8, 8), np.int16), 0.25, 0.0, 0.0)
    fused, resolution, _, _ = fuse_grids(
        [(source, (1.0, -0.5, math.radians(31.0)))])
    assert resolution == pytest.approx(0.25)
    known = fused >= 0
    # Every scanline through a rotated solid patch must be one contiguous run;
    # forward splatting used to leave unknown pinholes between mapped cells.
    for row in known:
        indices = np.flatnonzero(row)
        if indices.size:
            assert row[indices[0]:indices[-1] + 1].all()


def test_occupied_evidence_dominates_free_conflict():
    free = GridSpec(np.zeros((2, 2), np.int16), 1.0, 0.0, 0.0)
    wall_data = np.zeros((2, 2), np.int16)
    wall_data[0, 0] = 100
    wall = GridSpec(wall_data, 1.0, 0.0, 0.0)
    fused, _, _, _ = fuse_grids([(free, None), (wall, None)])
    assert fused[0, 0] == 100
    assert fused[1, 1] == 0


def test_origin_pose_and_inter_map_transform_cancel_without_doubling_map():
    data = np.zeros((5, 7), np.int16)
    data[0, :] = 100
    data[:, 0] = 100
    first = GridSpec(data, 0.2, 0.0, 0.0)
    yaw = math.radians(27.0)
    ox, oy = 4.0, -3.0
    second = GridSpec(data, 0.2, ox, oy, yaw)
    # T maps the second map coordinates back to the first map coordinates.
    c, s = math.cos(yaw), math.sin(yaw)
    inverse_origin = (-c * ox - s * oy, s * ox - c * oy, -yaw)
    fused, _, _, _ = fuse_grids([
        (first, None), (second, inverse_origin)])
    assert int((fused >= 0).sum()) == data.size
    assert int((fused >= 50).sum()) == int((data >= 50).sum())
