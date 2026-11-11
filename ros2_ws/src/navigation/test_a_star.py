import unittest

from a_star import plan_path


class AStarTest(unittest.TestCase):
    def test_path_avoids_obstacles_and_reaches_goal(self) -> None:
        grid = [[0, 0, 0], [0, 100, 0], [0, 0, 0]]

        path = plan_path(grid, (0, 0), (2, 2))

        self.assertEqual(path[0], (0, 0))
        self.assertEqual(path[-1], (2, 2))
        self.assertNotIn((1, 1), path)
        self.assertEqual(len(path), 5)

    def test_returns_empty_list_when_no_path_exists(self) -> None:
        grid = [[0, 100, 0], [0, 100, 0], [0, 100, 0]]
        self.assertEqual(plan_path(grid, (0, 0), (2, 2)), [])


if __name__ == "__main__":
    unittest.main()
