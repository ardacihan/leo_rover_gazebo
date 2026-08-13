#!/usr/bin/env python3

"""Decision tests for frontier-driven exploration."""

import math
import os
import sys
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "scripts"))

from frontier_policy import (
    distance_to,
    heading_error,
    is_blacklisted,
    select_target,
    should_abandon,
    turn_command,
)


class SelectTargetTest(unittest.TestCase):
    def test_no_clusters_means_exploration_is_complete(self):
        self.assertIsNone(select_target([], (0.0, 0.0)))

    def test_prefers_a_large_region_over_a_marginally_closer_scrap(self):
        clusters = [(5, 1.0, 0.0), (400, 2.0, 0.0)]
        self.assertEqual(select_target(clusters, (0.0, 0.0)), (2.0, 0.0))

    def test_prefers_the_nearer_of_two_equal_regions(self):
        clusters = [(100, 5.0, 0.0), (100, 1.0, 0.0)]
        self.assertEqual(select_target(clusters, (0.0, 0.0)), (1.0, 0.0))

    def test_skips_blacklisted_targets(self):
        clusters = [(400, 2.0, 0.0), (100, 1.0, 0.0)]
        chosen = select_target(
            clusters, (0.0, 0.0), blacklist=[(2.0, 0.0)], blacklist_radius=0.5
        )
        self.assertEqual(chosen, (1.0, 0.0))

    def test_all_blacklisted_means_done(self):
        clusters = [(400, 2.0, 0.0)]
        self.assertIsNone(
            select_target(clusters, (0.0, 0.0), blacklist=[(2.0, 0.1)])
        )

    def test_ignores_noise_sized_clusters(self):
        self.assertIsNone(
            select_target([(2, 1.0, 0.0)], (0.0, 0.0), minimum_cells=4)
        )


class GeometryTest(unittest.TestCase):
    def test_heading_error_is_zero_when_already_facing_target(self):
        self.assertAlmostEqual(heading_error((1.0, 0.0), (0.0, 0.0), 0.0), 0.0)

    def test_heading_error_takes_the_short_way_round(self):
        """Facing +170 deg with a target at -170 deg is a 20 deg turn."""
        error = heading_error(
            (-1.0, -0.1), (0.0, 0.0), math.radians(170.0)
        )
        self.assertLess(abs(error), math.radians(30.0))

    def test_distance(self):
        self.assertAlmostEqual(distance_to((3.0, 4.0), (0.0, 0.0)), 5.0)


class TurnCommandTest(unittest.TestCase):
    def test_sign_follows_the_error(self):
        self.assertGreater(turn_command(0.5, 1.0, 0.3), 0.0)
        self.assertLess(turn_command(-0.5, 1.0, 0.3), 0.0)

    def test_clamped_to_the_maximum(self):
        self.assertAlmostEqual(turn_command(10.0, 1.0, 0.3), 0.3)

    def test_small_error_still_produces_a_moving_command(self):
        self.assertAlmostEqual(abs(turn_command(0.001, 1.0, 0.3, 0.10)), 0.10)


class AbandonTest(unittest.TestCase):
    def test_keeps_going_while_progressing(self):
        self.assertIsNone(should_abandon(10.0, 60.0, 1.0, 0.15, 2.0, 8.0))

    def test_abandons_on_time_budget(self):
        self.assertIsNotNone(should_abandon(60.0, 60.0, 5.0, 0.15, 0.0, 8.0))

    def test_abandons_when_stalled(self):
        reason = should_abandon(10.0, 60.0, 0.01, 0.15, 9.0, 8.0)
        self.assertIsNotNone(reason)
        self.assertIn("no progress", reason)

    def test_stall_timer_alone_is_not_enough_if_moving(self):
        self.assertIsNone(should_abandon(10.0, 60.0, 2.0, 0.15, 9.0, 8.0))


class BlacklistTest(unittest.TestCase):
    def test_radius_is_inclusive(self):
        self.assertTrue(is_blacklisted((1.0, 0.0), [(1.4, 0.0)], 0.5))
        self.assertFalse(is_blacklisted((1.0, 0.0), [(1.6, 0.0)], 0.5))


if __name__ == "__main__":
    unittest.main()
