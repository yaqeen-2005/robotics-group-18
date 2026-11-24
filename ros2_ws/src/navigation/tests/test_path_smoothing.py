import math
import unittest

from common.path_smoothing import smooth_path


class PathSmoothingTest(unittest.TestCase):
    def test_turn_uses_30_and_60_degree_segments_without_mutating_path(self):
        grid = [[0] * 4 for _ in range(4)]
        path = [(0, 1), (1, 1), (1, 2)]
        result = smooth_path(grid, path)
        self.assertEqual(path, [(0, 1), (1, 1), (1, 2)])
        self.assertEqual((result[0], result[-1]), (path[0], path[-1]))
        angles = [round(math.degrees(math.atan2(b[1] - a[1], b[0] - a[0]))) for a, b in zip(result, result[1:])]
        self.assertEqual(angles, [0, 30, 60, 90])

    def test_blocked_corner_falls_back_to_original_bend(self):
        grid = [[0] * 4 for _ in range(4)]
        grid[1][1] = 100
        path = [(0, 1), (1, 1), (1, 2)]
        self.assertEqual(smooth_path(grid, path), path)


if __name__ == "__main__":
    unittest.main()
