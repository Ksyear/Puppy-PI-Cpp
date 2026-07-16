#!/usr/bin/env python3

import os
import sys
import unittest


PACKAGE_SRC = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', 'src'))
sys.path.insert(0, PACKAGE_SRC)

from puppy_vr_control_noetic.llm_frontier_selector import parse_frontier_choice
from puppy_vr_control_noetic.maze_core import GridMap, choose_lookahead, wrap_angle


class MazeCoreTest(unittest.TestCase):

    def test_astar_avoids_wall(self):
        width, height = 9, 7
        data = [0] * (width * height)
        for y in range(6):
            data[y * width + 4] = 100
        grid = GridMap(width, height, 1.0, 0.0, 0.0, data)
        path = grid.astar((1, 1), (7, 1), grid.inflated_obstacles(0.0))
        self.assertTrue(path)
        self.assertEqual(path[0], (1, 1))
        self.assertEqual(path[-1], (7, 1))
        self.assertTrue(all(grid.value(cell) < 65 for cell in path))
        self.assertTrue(any(cell[1] == 6 for cell in path))

    def test_two_reachable_frontiers(self):
        width, height = 12, 9
        data = [-1] * (width * height)
        for x in range(1, 11):
            for y in range(3, 6):
                data[y * width + x] = 0
            data[2 * width + x] = 100
            data[6 * width + x] = 100
        grid = GridMap(width, height, 1.0, 0.0, 0.0, data)
        candidates = grid.frontier_candidates(
            (5, 4), robot_radius=0.0, min_cluster_size=2,
            min_frontier_distance=0.0)
        self.assertEqual(len(candidates), 2)
        self.assertEqual({item['cell'][0] for item in candidates}, {1, 10})
        self.assertTrue(all(item['path'] for item in candidates))

    def test_world_conversion_and_lookahead(self):
        grid = GridMap(10, 10, 0.5, -2.5, -2.5, [0] * 100)
        self.assertEqual(grid.world_to_cell(0.0, 0.0), (5, 5))
        point = choose_lookahead([(5, 5), (6, 5), (7, 5)], grid, 0.25, 0.25, 0.6)
        self.assertEqual(point, (1.25, 0.25))
        self.assertAlmostEqual(wrap_angle(3.5), -2.783185, places=5)

    def test_llm_choice_is_allowlisted(self):
        self.assertEqual(parse_frontier_choice(
            '설명 {"frontier_id": 2}', [0, 1, 2]), 2)
        self.assertIsNone(parse_frontier_choice('{"frontier_id": 9}', [0, 1, 2]))
        self.assertIsNone(parse_frontier_choice('{"frontier_id": true}', [0, 1]))
        self.assertIsNone(parse_frontier_choice('not json', [0]))


if __name__ == '__main__':
    unittest.main()
