#!/usr/bin/env python3

import math
import os
import sys
import tempfile
import unittest


PACKAGE_SRC = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', 'src'))
sys.path.insert(0, PACKAGE_SRC)

from puppy_vr_control_noetic.mapping_artifacts import (
    angle_delta, occupancy_counts, occupancy_to_grayscale,
    quaternion_to_yaw, write_pgm)


class MappingArtifactsTest(unittest.TestCase):

    def test_occupancy_counts(self):
        counts = occupancy_counts([-1, 0, 25, 26, 64, 65, 100])
        self.assertEqual(counts, {
            'unknown': 1,
            'free': 2,
            'occupied': 2,
            'uncertain': 2,
            'known': 6,
        })

    def test_pgm_pixels_flip_vertical_axis(self):
        # OccupancyGrid 첫 행은 아래쪽, PGM 첫 행은 위쪽이다.
        pixels = occupancy_to_grayscale([-1, 0, 100, 50], 2, 2)
        self.assertEqual(list(pixels), [0, 127, 205, 254])
        with tempfile.TemporaryDirectory() as directory:
            path = os.path.join(directory, 'map.pgm')
            write_pgm(path, [-1, 0, 100, 50], 2, 2)
            with open(path, 'rb') as stream:
                self.assertTrue(stream.read().startswith(b'P5\n2 2\n255\n'))

    def test_angles(self):
        yaw = quaternion_to_yaw(0.0, 0.0, math.sin(0.25), math.cos(0.25))
        self.assertAlmostEqual(yaw, 0.5)
        self.assertAlmostEqual(angle_delta(-3.0, 3.0), 0.283185, places=5)


if __name__ == '__main__':
    unittest.main()
