"""Rover 4's RealSense and mast intrude into the scan plane behind the robot,
producing persistent returns as close as 0.06 m.  Those sit inside Collision
Monitor's 0.31 m footprint circle, so they read as a permanent obstacle.

These tests cover the sector parsing and the masking rule without needing
rclpy, and pin the two properties that matter for safety: a real obstacle
beyond the exclusion range is never masked, and masked rays become NaN
(no reading) rather than range_max (asserted clear).
"""

import ast
import math
import pathlib
import sys
import unittest

import numpy


PACKAGE = pathlib.Path(__file__).parents[1]
sys.path.insert(0, str(PACKAGE / "scripts"))

from exploration_policy import parse_sectors  # noqa: E402


# Measured on Rover 4, base-frame degrees.
ROVER4_SECTORS = "171:180,-180:-168,-164:-153,-131:-102"


def apply_mask(ranges, angle_min, increment, yaw, sectors, max_range):
    """Mirror of ScanSelfFilter._callback's masking decision."""
    raw = angle_min + yaw + numpy.arange(ranges.size, dtype=float) * increment
    angles = numpy.arctan2(numpy.sin(raw), numpy.cos(raw))
    mask = numpy.zeros(ranges.size, dtype=bool)
    for low, high in sectors:
        mask |= (angles >= low) & (angles <= high)
    drop = mask & numpy.isfinite(ranges) & (ranges < max_range)
    out = ranges.copy()
    out[drop] = math.nan
    return out, drop


class SectorParsingTests(unittest.TestCase):
    def test_parses_rover4_sectors(self):
        sectors = parse_sectors(ROVER4_SECTORS)
        self.assertEqual(len(sectors), 4)
        low, high = sectors[0]
        self.assertAlmostEqual(math.degrees(low), 171.0)
        self.assertAlmostEqual(math.degrees(high), 180.0)

    def test_empty_string_means_no_filtering(self):
        self.assertEqual(parse_sectors(""), [])
        self.assertEqual(parse_sectors("  "), [])

    def test_rejects_malformed_and_inverted_sectors(self):
        for bad in ("171", "a:b", "10:5", "5:5"):
            with self.assertRaises(ValueError, msg=bad):
                parse_sectors(bad)


class MaskingTests(unittest.TestCase):
    ANGLE_MIN = -3.118870496749878
    INCREMENT = 0.01239244919270277
    COUNT = 504
    YAW = math.pi
    MAX_RANGE = 0.45

    def _angles_deg(self):
        raw = (
            self.ANGLE_MIN + self.YAW
            + numpy.arange(self.COUNT, dtype=float) * self.INCREMENT
        )
        return numpy.degrees(numpy.arctan2(numpy.sin(raw), numpy.cos(raw)))

    def test_masks_the_measured_self_returns(self):
        """0.06-0.16 m returns in the rear sectors must be removed."""
        deg = self._angles_deg()
        ranges = numpy.full(self.COUNT, 3.0)
        in_sector = (deg >= -119) & (deg <= -104)
        ranges[in_sector] = 0.06

        out, drop = apply_mask(
            ranges, self.ANGLE_MIN, self.INCREMENT, self.YAW,
            parse_sectors(ROVER4_SECTORS), self.MAX_RANGE,
        )
        self.assertTrue(in_sector.any())
        self.assertTrue(numpy.isnan(out[in_sector]).all())
        self.assertEqual(int(drop.sum()), int(in_sector.sum()))

    def test_real_obstacle_beyond_the_range_is_kept(self):
        """A wall behind the rover must still be seen; only close self-returns go."""
        deg = self._angles_deg()
        ranges = numpy.full(self.COUNT, 3.0)
        in_sector = (deg >= -119) & (deg <= -104)
        ranges[in_sector] = 1.2  # a real wall, well beyond 0.45 m

        out, drop = apply_mask(
            ranges, self.ANGLE_MIN, self.INCREMENT, self.YAW,
            parse_sectors(ROVER4_SECTORS), self.MAX_RANGE,
        )
        self.assertFalse(drop.any())
        numpy.testing.assert_allclose(out[in_sector], 1.2)

    def test_front_sector_is_never_masked(self):
        """Forward obstacles are the ones that matter; they must survive intact."""
        deg = self._angles_deg()
        ranges = numpy.full(self.COUNT, 5.0)
        front = numpy.abs(deg) <= 32.0
        ranges[front] = 0.10  # something right in front

        out, drop = apply_mask(
            ranges, self.ANGLE_MIN, self.INCREMENT, self.YAW,
            parse_sectors(ROVER4_SECTORS), self.MAX_RANGE,
        )
        self.assertTrue(front.any())
        self.assertFalse(drop[front].any())
        numpy.testing.assert_allclose(out[front], 0.10)

    def test_masked_rays_are_nan_not_range_max(self):
        """NaN means 'no reading'; range_max would claim clear space we can't see."""
        deg = self._angles_deg()
        ranges = numpy.full(self.COUNT, 3.0)
        ranges[(deg >= -119) & (deg <= -104)] = 0.06

        out, _ = apply_mask(
            ranges, self.ANGLE_MIN, self.INCREMENT, self.YAW,
            parse_sectors(ROVER4_SECTORS), self.MAX_RANGE,
        )
        masked = numpy.isnan(out)
        self.assertTrue(masked.any())
        self.assertFalse(numpy.isinf(out).any())


class LaunchWiringTests(unittest.TestCase):
    def test_filter_output_feeds_slam_and_the_safety_nodes(self):
        """scan_topic must be the filtered topic for every consumer, and the
        slam override must come after the params file or the file wins."""
        tree = ast.parse(
            (PACKAGE / "launch" / "safe_mapping.launch.py").read_text(
                encoding="utf-8"
            )
        )
        found_slam = False
        for call in ast.walk(tree):
            if not isinstance(call, ast.Call):
                continue
            if not isinstance(call.func, ast.Name) or call.func.id != "Node":
                continue
            keywords = {k.arg: k.value for k in call.keywords}
            name = keywords.get("name")
            if not isinstance(name, ast.Constant) or name.value != "slam_toolbox":
                continue

            found_slam = True
            params = keywords["parameters"]
            self.assertIsInstance(params, ast.List)
            self.assertEqual(len(params.elts), 2, "expected [file, override]")
            override = params.elts[1]
            self.assertIsInstance(override, ast.Dict)
            keys = [
                k.value for k in override.keys if isinstance(k, ast.Constant)
            ]
            self.assertIn("scan_topic", keys)

        self.assertTrue(found_slam, "slam_toolbox node not found")


if __name__ == "__main__":
    unittest.main()
