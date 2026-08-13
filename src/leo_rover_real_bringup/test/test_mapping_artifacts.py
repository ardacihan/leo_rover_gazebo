#!/usr/bin/env python3

import math
import pathlib
import sys
import unittest

import numpy as np


SCRIPTS = pathlib.Path(__file__).parents[1] / "scripts"
sys.path.insert(0, str(SCRIPTS))

from mapping_artifacts import (  # noqa: E402
    encode_png_rgb,
    render_path_overlay,
    world_to_grid,
)


class MappingArtifactTests(unittest.TestCase):
    def test_world_to_rotated_grid(self):
        points = world_to_grid([[1.0, 2.0]], [1.0, 1.0], math.pi / 2.0, 0.5)
        np.testing.assert_array_equal(points[0], [2, 0])

    def test_overlay_marks_route_and_encodes_png(self):
        data = [0] * 100
        image = render_path_overlay(
            data, 10, 10, 1.0, [0.0, 0.0], 0.0, [[1.0, 1.0], [8.0, 8.0]]
        )
        self.assertTrue(np.any(np.all(image == [220, 30, 30], axis=2)))
        encoded = encode_png_rgb(image)
        self.assertTrue(encoded.startswith(b"\x89PNG\r\n\x1a\n"))


if __name__ == "__main__":
    unittest.main()
