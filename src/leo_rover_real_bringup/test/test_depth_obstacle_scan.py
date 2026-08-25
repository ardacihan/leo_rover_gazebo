"""Floor rejection is the entire reason this node exists.

Measured on Rover 4: a row-based depth-to-scan conversion reported the floor as
an obstacle at a fixed 1.44 m while the LIDAR saw 3.5-4.8 m of open space.
depthimage_to_laserscan cannot fix that, because it collapses image rows
without knowing where they land in the world.

These tests reproduce that exact geometry -- camera 0.25 m up, level, looking
at a flat floor -- and assert the height filter removes it while keeping a real
obstacle standing on that same floor.
"""

import math
import unittest

import numpy


# Rover 4's measured depth intrinsics and mount.
FX = FY = 214.6
CX, CY = 212.0, 121.2
WIDTH, HEIGHT = 424, 240
CAMERA_HEIGHT = 0.25

MIN_HEIGHT = 0.06
MAX_HEIGHT = 0.60
RANGE_MIN, RANGE_MAX = 0.45, 6.0


def project_to_base(depth, step=1):
    """Mirror of the node: pixels -> optical 3D -> base frame (level camera).

    A level, forward-facing camera maps optical (x right, y down, z forward)
    to base (x forward, y left, z up) as: X=z, Y=-x, Z=CAMERA_HEIGHT-y.
    """
    d = depth[::step, ::step]
    rows = numpy.arange(0, HEIGHT, step, dtype=float)
    cols = numpy.arange(0, WIDTH, step, dtype=float)
    x = (cols[None, :] - CX) / FX * d
    y = (rows[:, None] - CY) / FY * d
    z = d

    valid = numpy.isfinite(d) & (d > 0.0) & (d <= RANGE_MAX)
    X = z[valid]
    Y = -x[valid]
    Z = CAMERA_HEIGHT - y[valid]
    return X, Y, Z


def nearest_forward_range(depth, use_height_filter):
    X, Y, Z = project_to_base(depth)
    if use_height_filter:
        keep = (Z >= MIN_HEIGHT) & (Z <= MAX_HEIGHT)
        X, Y = X[keep], Y[keep]
    if X.size == 0:
        return None
    planar = numpy.hypot(X, Y)
    bearing = numpy.degrees(numpy.arctan2(Y, X))
    front = (numpy.abs(bearing) <= 20.0) & (planar >= RANGE_MIN) & (planar <= RANGE_MAX)
    if not front.any():
        return None
    return float(planar[front].min())


def flat_floor_depth():
    """Depth image of nothing but a flat floor, camera level at 0.25 m."""
    depth = numpy.full((HEIGHT, WIDTH), numpy.nan)
    for row in range(HEIGHT):
        angle = math.atan2(row - CY, FY)   # positive = below the optical axis
        if angle <= 1e-4:
            continue                       # at or above horizontal: sees nothing
        ray = CAMERA_HEIGHT / math.sin(angle)
        if ray > RANGE_MAX:
            continue
        depth[row, :] = ray
    return depth


def floor_with_box(distance, box_height):
    """A box of the given height standing on the floor at the given distance."""
    depth = flat_floor_depth()
    for row in range(HEIGHT):
        angle = math.atan2(row - CY, FY)
        height_at_box = CAMERA_HEIGHT - distance * math.tan(angle)
        if 0.0 <= height_at_box <= box_height:
            depth[row, WIDTH // 2 - 40:WIDTH // 2 + 40] = distance / math.cos(angle)
    return depth


class FloorRejectionTests(unittest.TestCase):
    def test_unfiltered_conversion_reports_the_floor_as_an_obstacle(self):
        """This is the bug being fixed -- it must actually reproduce."""
        floor_only = nearest_forward_range(flat_floor_depth(), use_height_filter=False)
        self.assertIsNotNone(floor_only)
        self.assertLess(
            floor_only, 3.0,
            "expected the unfiltered conversion to see the floor up close",
        )

    def test_height_filter_removes_a_bare_floor_entirely(self):
        """With only floor in view, the scan must report nothing at all."""
        self.assertIsNone(
            nearest_forward_range(flat_floor_depth(), use_height_filter=True)
        )

    def test_a_box_on_the_floor_survives_the_filter(self):
        """The point of the camera: a 0.30 m box the LIDAR plane would clip."""
        depth = floor_with_box(distance=1.50, box_height=0.30)
        found = nearest_forward_range(depth, use_height_filter=True)
        self.assertIsNotNone(found, "box was filtered away with the floor")
        self.assertAlmostEqual(found, 1.50, delta=0.15)

    def test_a_low_obstacle_below_the_band_is_ignored(self):
        """A 2 cm cable is not an obstacle; it must not stop the robot."""
        depth = floor_with_box(distance=1.50, box_height=0.02)
        self.assertIsNone(nearest_forward_range(depth, use_height_filter=True))

    def test_overhead_geometry_above_the_band_is_ignored(self):
        """Something at 1.2 m clears the rover and must not register."""
        depth = numpy.full((HEIGHT, WIDTH), numpy.nan)
        for row in range(HEIGHT):
            angle = math.atan2(row - CY, FY)
            height = CAMERA_HEIGHT - 2.0 * math.tan(angle)
            if height >= 1.0:
                depth[row, :] = 2.0 / math.cos(angle)
        self.assertIsNone(nearest_forward_range(depth, use_height_filter=True))


class DecimationTests(unittest.TestCase):
    def test_subsampling_preserves_the_detected_distance(self):
        """pixel_step exists for CPU headroom; it must not move the answer."""
        depth = floor_with_box(distance=1.50, box_height=0.30)
        full = project_to_base(depth, step=1)
        quarter = project_to_base(depth, step=2)

        def nearest(bundle):
            X, Y, Z = bundle
            keep = (Z >= MIN_HEIGHT) & (Z <= MAX_HEIGHT)
            planar = numpy.hypot(X[keep], Y[keep])
            bearing = numpy.degrees(numpy.arctan2(Y[keep], X[keep]))
            front = numpy.abs(bearing) <= 20.0
            return float(planar[front].min())

        self.assertAlmostEqual(nearest(full), nearest(quarter), delta=0.05)


if __name__ == "__main__":
    unittest.main()
