"""The per-bearing reduction takes the NEAREST return in each bin.

That reduction was rewritten from numpy.minimum.at to a sort-based one purely
for speed -- Collision Monitor discards sources it cannot keep up with, so CPU
in this path is a safety property, not a nicety. These tests pin the two
implementations together so the optimisation cannot change what the robot sees.
"""

import unittest

import numpy


BIN_COUNT = 360


def reference_bins(index, planar):
    """The original numpy.minimum.at reduction."""
    ranges = numpy.full(BIN_COUNT, numpy.inf, dtype=numpy.float32)
    numpy.minimum.at(ranges, index, planar)
    return ranges


def sorted_bins(index, planar):
    """The shipped sort-based reduction."""
    ranges = numpy.full(BIN_COUNT, numpy.inf, dtype=numpy.float32)
    order = numpy.argsort(planar, kind="stable")
    bins_sorted = index[order]
    unique_bins, first = numpy.unique(bins_sorted, return_index=True)
    ranges[unique_bins] = planar[order][first]
    return ranges


class BinningEquivalenceTests(unittest.TestCase):
    def test_matches_minimum_at_on_random_data(self):
        for seed in range(8):
            rng = numpy.random.default_rng(seed)
            count = int(rng.integers(50, 5000))
            index = rng.integers(0, BIN_COUNT, size=count)
            planar = rng.uniform(0.45, 6.0, size=count).astype(numpy.float32)
            numpy.testing.assert_allclose(
                sorted_bins(index, planar),
                reference_bins(index, planar),
                err_msg=f"seed={seed}",
            )

    def test_keeps_the_nearest_when_many_points_share_a_bin(self):
        """A far wall behind a near box must not hide the box."""
        index = numpy.zeros(500, dtype=int)
        planar = numpy.linspace(5.0, 1.0, 500).astype(numpy.float32)
        result = sorted_bins(index, planar)
        self.assertAlmostEqual(float(result[0]), 1.0, places=5)

    def test_empty_bins_stay_infinite(self):
        """Infinity means 'nothing seen', which consumers treat as clear.
        A zero here would read as an obstacle at the sensor origin."""
        index = numpy.array([10, 10, 200])
        planar = numpy.array([1.0, 2.0, 3.0], dtype=numpy.float32)
        result = sorted_bins(index, planar)
        self.assertTrue(numpy.isinf(result[0]))
        self.assertTrue(numpy.isinf(result[359]))
        self.assertAlmostEqual(float(result[10]), 1.0, places=5)
        self.assertAlmostEqual(float(result[200]), 3.0, places=5)

    def test_single_point_input(self):
        index = numpy.array([42])
        planar = numpy.array([2.5], dtype=numpy.float32)
        result = sorted_bins(index, planar)
        self.assertAlmostEqual(float(result[42]), 2.5, places=5)
        self.assertEqual(int(numpy.isfinite(result).sum()), 1)


if __name__ == "__main__":
    unittest.main()
