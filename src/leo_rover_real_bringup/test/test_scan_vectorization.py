"""The gate and explorer sector maths were vectorised because the original
per-ray Python loops burned enough CPU to delay /scan delivery to Collision
Monitor, which then discarded the source and stopped constraining motion.

These tests re-implement the ORIGINAL loops as reference and assert the
vectorised versions agree, including the awkward cases: positive infinity as
maximum clearance, readings below the self-filter radius, and out-of-range
readings.
"""

import math
import unittest

import numpy


SELF_FILTER = 0.05
RANGE_MIN = 0.15
RANGE_MAX = 40.0
OUTLIER_POINTS = 5


def _normalize(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def _make_scan(seed, count=501):
    """Deterministic ranges mixing finite, inf, nan and out-of-range values."""
    rng = numpy.random.default_rng(seed)
    ranges = rng.uniform(0.0, 6.0, size=count)
    ranges[::17] = math.inf
    ranges[::29] = math.nan
    ranges[::41] = 0.02          # below the self-filter radius
    ranges[::53] = RANGE_MAX + 5  # beyond range_max
    return ranges


# --- reference implementations (the original loops) ----------------------

def reference_sector(ranges, angle_min, increment, yaw, lo_deg, hi_deg):
    lower, upper = math.radians(lo_deg), math.radians(hi_deg)
    values = []
    angle = angle_min + yaw
    for reading in ranges:
        if lower <= _normalize(angle) <= upper:
            if math.isinf(reading) and reading > 0.0:
                values.append(RANGE_MAX)
            elif (
                math.isfinite(reading)
                and reading >= max(RANGE_MIN, SELF_FILTER)
                and reading <= RANGE_MAX
            ):
                values.append(float(reading))
        angle += increment
    if len(values) < 5:
        return 0.0
    values.sort()
    return values[min(OUTLIER_POINTS, len(values) - 1)]


def reference_gate(ranges, angle_min, increment, yaw):
    finite_points = 0
    rear = []
    angle = angle_min + yaw
    for value in ranges:
        if math.isfinite(value) and RANGE_MIN <= value <= RANGE_MAX:
            finite_points += 1
            if value >= SELF_FILTER:
                x = value * math.cos(angle)
                y = value * math.sin(angle)
                if x < 0.0 and abs(y) <= 0.30:
                    rear.append(-x)
        angle += increment
    return finite_points, sorted(rear)


# --- vectorised implementations (mirroring the shipped code) -------------

def vector_sector(ranges, angle_min, increment, yaw, lo_deg, hi_deg):
    lower, upper = math.radians(lo_deg), math.radians(hi_deg)
    raw = angle_min + yaw + numpy.arange(ranges.size, dtype=float) * increment
    angles = numpy.arctan2(numpy.sin(raw), numpy.cos(raw))

    in_sector = (angles >= lower) & (angles <= upper)
    if not in_sector.any():
        return 0.0
    minimum = max(RANGE_MIN, SELF_FILTER)
    unbounded = in_sector & numpy.isposinf(ranges)
    bounded = (
        in_sector
        & numpy.isfinite(ranges)
        & (ranges >= minimum)
        & (ranges <= RANGE_MAX)
    )
    values = numpy.concatenate((
        ranges[bounded],
        numpy.full(int(numpy.count_nonzero(unbounded)), RANGE_MAX),
    ))
    if values.size < 5:
        return 0.0
    values.sort()
    return float(values[min(OUTLIER_POINTS, values.size - 1)])


def vector_gate(ranges, angle_min, increment, yaw):
    angles = angle_min + yaw + numpy.arange(ranges.size, dtype=float) * increment
    cos, sin = numpy.cos(angles), numpy.sin(angles)
    valid = numpy.isfinite(ranges) & (ranges >= RANGE_MIN) & (ranges <= RANGE_MAX)
    finite_points = int(numpy.count_nonzero(valid))
    usable = valid & (ranges >= SELF_FILTER)
    x, y = ranges * cos, ranges * sin
    rear = usable & (x < 0.0) & (numpy.abs(y) <= 0.30)
    return finite_points, sorted((-x[rear]).tolist())


class VectorisationEquivalenceTests(unittest.TestCase):
    ANGLE_MIN = -3.118870496749878
    INCREMENT = 0.01239244919270277

    SECTORS = [
        (-32.0, 32.0), (25.0, 105.0), (-105.0, -25.0),
        (100.0, 170.0), (-170.0, -100.0), (145.0, 179.9), (-179.9, -145.0),
    ]

    def test_sector_clearance_matches_the_original_loop(self):
        for seed in range(6):
            ranges = _make_scan(seed)
            for yaw in (0.0, math.pi):
                for lo, hi in self.SECTORS:
                    expected = reference_sector(
                        ranges, self.ANGLE_MIN, self.INCREMENT, yaw, lo, hi
                    )
                    actual = vector_sector(
                        ranges, self.ANGLE_MIN, self.INCREMENT, yaw, lo, hi
                    )
                    self.assertAlmostEqual(
                        actual, expected, places=9,
                        msg=f"seed={seed} yaw={yaw} sector=({lo},{hi})",
                    )

    def test_gate_rear_corridor_matches_the_original_loop(self):
        for seed in range(6):
            ranges = _make_scan(seed)
            for yaw in (0.0, math.pi):
                exp_count, exp_rear = reference_gate(
                    ranges, self.ANGLE_MIN, self.INCREMENT, yaw
                )
                act_count, act_rear = vector_gate(
                    ranges, self.ANGLE_MIN, self.INCREMENT, yaw
                )
                self.assertEqual(act_count, exp_count, f"seed={seed} yaw={yaw}")
                self.assertEqual(len(act_rear), len(exp_rear))
                for a, e in zip(act_rear, exp_rear):
                    self.assertAlmostEqual(a, e, places=9)

    def test_negative_infinity_is_not_treated_as_clearance(self):
        """-inf is a dropout, not open space; only +inf means max range."""
        ranges = numpy.full(501, 1.0)
        ranges[100:200] = -math.inf
        for lo, hi in self.SECTORS:
            self.assertAlmostEqual(
                vector_sector(ranges, self.ANGLE_MIN, self.INCREMENT, 0.0, lo, hi),
                reference_sector(ranges, self.ANGLE_MIN, self.INCREMENT, 0.0, lo, hi),
                places=9,
            )


if __name__ == "__main__":
    unittest.main()
