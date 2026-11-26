"""Run from the repository root with unittest discovery (see README)."""

import copy
import unittest

from frontier import find_frontiers


class FrontierTests(unittest.TestCase):
    def test_uniform_maps_have_no_frontiers(self):
        for value in (-1, 0, 100):
            with self.subTest(value=value):
                self.assertEqual(find_frontiers([[value] * 3 for _ in range(2)]), [])

    def test_obstacles_are_not_frontiers(self):
        self.assertEqual(find_frontiers([[0, 100, -1]]), [])

    def test_each_cardinal_direction(self):
        for x, y in ((0, 1), (2, 1), (1, 0), (1, 2)):
            with self.subTest(unknown=(x, y)):
                grid = [[100] * 3 for _ in range(3)]
                grid[1][1] = 0
                grid[y][x] = -1
                self.assertEqual(find_frontiers(grid), [(1, 1)])

    def test_diagonal_unknown_does_not_count(self):
        self.assertEqual(find_frontiers([[0, 100], [100, -1]]), [])

    def test_edges_do_not_wrap(self):
        self.assertEqual(find_frontiers([[0, 100, -1]]), [])
        self.assertEqual(find_frontiers([[0], [100], [-1]]), [])
        self.assertEqual(find_frontiers([[0]]), [])
        self.assertEqual(find_frontiers([[0, -1]]), [(0, 0)])

    def test_coordinates_order_uniqueness_and_input_preservation(self):
        grid = [[100, -1, 100, 100], [-1, 0, -1, 0], [100, 0, 100, -1]]
        original = copy.deepcopy(grid)
        self.assertEqual(find_frontiers(grid), [(1, 1), (3, 1)])
        self.assertEqual(grid, original)

    def test_empty_maps(self):
        for grid in ([], [[]], [[], []]):
            self.assertEqual(find_frontiers(grid), [])

    def test_ragged_maps_rejected(self):
        for grid in ([[0], [0, -1]], [[], [0]]):
            with self.assertRaises(ValueError):
                find_frontiers(grid)

    def test_unsupported_cell_rejected(self):
        with self.assertRaises(ValueError):
            find_frontiers([[0, 50]])


if __name__ == "__main__":
    unittest.main()
