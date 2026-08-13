#!/usr/bin/env python3

"""Decision tests for the exploration coverage helpers."""

import os
import sys
import unittest

import numpy as np

sys.path.insert(
    0, os.path.join(os.path.dirname(__file__), "..", "scripts")
)

from map_coverage import cluster_frontiers, coverage_stats, frontier_mask


class CoverageStatsTest(unittest.TestCase):
    def test_counts_each_category(self):
        grid = np.array([[-1, -1], [0, 100]])
        unknown, free, occupied = coverage_stats(grid)
        self.assertAlmostEqual(unknown, 0.5)
        self.assertAlmostEqual(free, 0.25)
        self.assertAlmostEqual(occupied, 0.25)

    def test_empty_grid_is_safe(self):
        self.assertEqual(coverage_stats(np.empty((0, 0))), (0.0, 0.0, 0.0))


class FrontierMaskTest(unittest.TestCase):
    def test_free_cell_touching_unknown_is_a_frontier(self):
        grid = np.array([[0, 0, -1]])
        self.assertTrue(bool(frontier_mask(grid)[0, 1]))

    def test_interior_free_cell_is_not_a_frontier(self):
        grid = np.array([[0, 0, 0]])
        self.assertFalse(frontier_mask(grid).any())

    def test_unknown_behind_a_wall_is_not_a_frontier(self):
        """The far side of a wall is unknown but unreachable."""
        grid = np.array([[0, 100, -1]])
        self.assertFalse(frontier_mask(grid).any())

    def test_unknown_cells_are_never_frontiers_themselves(self):
        grid = np.array([[-1, -1], [-1, -1]])
        self.assertFalse(frontier_mask(grid).any())


class ClusterTest(unittest.TestCase):
    def test_separate_groups_are_distinct_and_size_ranked(self):
        mask = np.zeros((5, 9), dtype=bool)
        mask[0, 0:2] = True          # 2 cells
        mask[0, 5:9] = True          # 4 cells
        clusters = cluster_frontiers(mask, min_cells=2)
        self.assertEqual(len(clusters), 2)
        self.assertEqual(clusters[0][0], 4)
        self.assertEqual(clusters[1][0], 2)

    def test_small_clusters_are_dropped_as_noise(self):
        mask = np.zeros((3, 3), dtype=bool)
        mask[1, 1] = True
        self.assertEqual(cluster_frontiers(mask, min_cells=4), [])

    def test_diagonal_cells_join_one_cluster(self):
        mask = np.array([[True, False], [False, True]])
        clusters = cluster_frontiers(mask, min_cells=2)
        self.assertEqual(len(clusters), 1)
        self.assertEqual(clusters[0][0], 2)


if __name__ == "__main__":
    unittest.main()
