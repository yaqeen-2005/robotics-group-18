import math
import unittest

from common.path_smoothing import _clear, sample_waypoints, smooth_path


class PathSmoothingTest(unittest.TestCase):
    def test_single_turn_uses_a_safe_45_degree_hypotenuse(self):
        grid = [[0] * 4 for _ in range(4)]
        path = [(0, 1), (1, 1), (1, 2)]
        result = smooth_path(grid, path)
        self.assertEqual(path, [(0, 1), (1, 1), (1, 2)])
        self.assertEqual((result[0], result[-1]), (path[0], path[-1]))
        angles = [round(math.degrees(math.atan2(b[1] - a[1], b[0] - a[0]))) for a, b in zip(result, result[1:])]
        self.assertEqual(angles, [45])

    def test_blocked_corner_falls_back_to_original_bend(self):
        grid = [[0] * 5 for _ in range(5)]
        grid[1][1] = 100
        path = [(0, 0), (1, 0), (2, 0), (2, 1), (2, 2)]
        self.assertEqual(smooth_path(grid, path), path)

    def test_long_staircase_becomes_a_safe_30_degree_segment(self):
        grid = [[0] * 16 for _ in range(12)]
        path = [(1, 1)]
        for _ in range(6):
            x, y = path[-1]
            path.extend(((x + 1, y), (x + 2, y), (x + 2, y + 1)))
        path.append((13, 8))
        result = smooth_path(grid, path)
        diagonals = [(a, b) for a, b in zip(result, result[1:])
                     if abs(a[0] - b[0]) > 4 and abs(a[1] - b[1]) > 2]
        self.assertTrue(diagonals)
        self.assertEqual(path[0], result[0])
        self.assertEqual(path[-1], result[-1])
        positions = sample_waypoints(result, step=0.8)
        self.assertEqual(positions[0], path[0])
        self.assertEqual(positions[-1], path[-1])
        self.assertTrue(all(_clear(grid, a, b) for a, b in zip(positions, positions[1:])))
        rotated = [(y, x) for x, y in path]
        rotated_result = smooth_path([[0] * 12 for _ in range(16)], rotated)
        self.assertTrue(any(
            math.hypot(b[0] - a[0], b[1] - a[1]) > 4
            and abs(math.degrees(math.atan2(b[1] - a[1], b[0] - a[0])) - 60) < 0.01
            for a, b in zip(rotated_result, rotated_result[1:])
        ))
        grid[3][4] = 100
        blocked_result = smooth_path(grid, path)
        self.assertTrue(all(_clear(grid, a, b) for a, b in zip(blocked_result, blocked_result[1:])))


if __name__ == "__main__":
    unittest.main()
