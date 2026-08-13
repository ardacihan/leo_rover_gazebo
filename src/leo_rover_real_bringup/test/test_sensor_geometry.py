#!/usr/bin/env python3

import math
import pathlib
import sys
import unittest

import numpy as np


SCRIPTS = pathlib.Path(__file__).parents[1] / "scripts"
sys.path.insert(0, str(SCRIPTS))

from sensor_geometry import (  # noqa: E402
    merge_planar_points,
    points_to_scan_ranges,
    project_depth_to_base,
    quaternion_rotation_matrix,
    scan_to_base_points,
)


class SensorGeometryTests(unittest.TestCase):
    def test_height_filter_removes_floor_and_keeps_obstacle(self):
        points = np.asarray([
            [0.50, 0.00, 0.01],
            [0.80, 0.00, 0.10],
            [1.20, 0.00, 0.60],
        ])
        ranges = points_to_scan_ranges(
            points,
            min_height=0.04,
            max_height=0.45,
            angle_min=-0.5,
            angle_max=0.5,
            angle_increment=0.1,
            range_min=0.2,
            range_max=3.0,
        )
        self.assertAlmostEqual(float(ranges[5]), 0.8, places=5)

    def test_projection_uses_optical_axis_then_base_transform(self):
        depth = np.asarray([[2.0]], dtype=np.float32)
        # Optical Z-forward becomes base X-forward.
        rotation = np.asarray([[0.0, 0.0, 1.0], [-1.0, 0.0, 0.0], [0.0, -1.0, 0.0]])
        points, fraction = project_depth_to_base(
            depth, 1.0, 1.0, 0.0, 0.0, rotation, [0.1, 0.0, 0.3], pixel_stride=1
        )
        self.assertEqual(fraction, 1.0)
        np.testing.assert_allclose(points[0], [2.1, 0.0, 0.3])

    def test_lidar_pi_yaw_maps_raw_rear_to_base_front(self):
        rotation = quaternion_rotation_matrix((0.0, 0.0, 1.0, 0.0))
        points = scan_to_base_points(
            [1.0], math.pi, 1.0, 0.1, 5.0, rotation, [0.0, 0.0, 0.2]
        )
        np.testing.assert_allclose(points[0, :2], [1.0, 0.0], atol=1.0e-8)

    def test_bounded_self_mask_keeps_same_bearing_far_obstacle(self):
        points = scan_to_base_points(
            [0.15, 1.0],
            math.radians(50.0),
            math.radians(1.0),
            0.02,
            5.0,
            np.eye(3),
            [0.0, 0.0, 0.0],
            self_mask_angle_min=math.radians(45.0),
            self_mask_angle_max=math.radians(82.0),
            self_mask_max_range=0.22,
        )
        self.assertEqual(len(points), 1)
        self.assertAlmostEqual(math.hypot(points[0, 0], points[0, 1]), 1.0)

    def test_footprint_backstop_drops_bracket_outside_mast_window(self):
        """A real Rover 4 bracket return sits outside the mast angle window.

        Raw-laser -141 deg at 0.032 m, with the lidar at (0.0775, 0.04) yawed
        by pi, lands at base (0.102, 0.060): inside the footprint, so it must
        not survive as an obstacle.
        """
        angle = math.radians(-141.0)
        rotation = [
            [math.cos(math.pi), -math.sin(math.pi), 0.0],
            [math.sin(math.pi), math.cos(math.pi), 0.0],
            [0.0, 0.0, 1.0],
        ]
        kwargs = dict(
            ranges=[0.032],
            angle_min=angle,
            angle_increment=math.radians(1.0),
            range_min=0.02,
            range_max=12.0,
            rotation=rotation,
            translation=[0.0775, 0.04, 0.2458],
            self_mask_angle_min=math.radians(12.0),
            self_mask_angle_max=math.radians(83.0),
            self_mask_max_range=0.22,
        )
        survives = scan_to_base_points(**kwargs)
        self.assertEqual(len(survives), 1, "mast window alone should miss it")
        self.assertLess(
            math.hypot(survives[0, 0], survives[0, 1]), 0.22,
            "and it lands inside the footprint",
        )
        dropped = scan_to_base_points(self_mask_footprint_radius=0.22, **kwargs)
        self.assertEqual(len(dropped), 0)

    def test_footprint_backstop_keeps_external_returns(self):
        rotation = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]
        points = scan_to_base_points(
            ranges=[1.5],
            angle_min=0.0,
            angle_increment=math.radians(1.0),
            range_min=0.02,
            range_max=12.0,
            rotation=rotation,
            translation=[0.0, 0.0, 0.0],
            self_mask_footprint_radius=0.22,
        )
        self.assertEqual(len(points), 1)

    def test_fusion_uses_nearest_return(self):
        ranges = merge_planar_points(
            [np.asarray([[2.0, 0.0, 0.2]]), np.asarray([[0.7, 0.0, 0.1]])],
            -0.5,
            0.5,
            0.1,
            0.2,
            3.0,
        )
        self.assertAlmostEqual(float(ranges[5]), 0.7, places=5)


if __name__ == "__main__":
    unittest.main()
