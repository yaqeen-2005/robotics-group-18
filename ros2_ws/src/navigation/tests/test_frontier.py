"""Tests adapted from Xu's frontier tests in commit 398f480."""

import copy
import unittest

from common.cell_attributes import CellAttributeMap
from exploration.frontier import find_frontiers


class FrontierTests(unittest.TestCase):
    def test_uniform_maps_have_no_frontiers(self):
        for value in (-1, 0, 100):
            self.assertEqual(find_frontiers([[value] * 3 for _ in range(2)]), [])

    def test_cardinal_unknown_counts_but_diagonal_does_not(self):
        self.assertEqual(find_frontiers([[0, -1], [100, 100]]), [(0, 0)])
        self.assertEqual(find_frontiers([[0, 100], [100, -1]]), [])

    def test_coordinates_are_unique_and_input_is_not_modified(self):
        grid = [[100, -1, 100], [-1, 0, -1], [100, 0, 100]]
        original = copy.deepcopy(grid)
        self.assertEqual(find_frontiers(grid), [(1, 1)])
        self.assertEqual(grid, original)

    def test_low_confidence_is_treated_as_frontier_information(self):
        grid = [[0, 0, 0]]
        values = [[[1.0, 0.0, 0.0, 0.0] for _ in row] for row in grid]
        attributes = CellAttributeMap(3, 1, values)
        attributes.values[0][2][0] = 0.2
        self.assertEqual(find_frontiers(grid, attributes, 0.5), [(1, 0), (2, 0)])

    def test_invalid_grid_is_rejected(self):
        with self.assertRaises(ValueError):
            find_frontiers([[0], [0, -1]])
        with self.assertRaises(ValueError):
            find_frontiers([[0, 101]])


if __name__ == "__main__":
    unittest.main()
