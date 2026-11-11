import unittest
from pathlib import Path

from a_star import plan_path
from map_io import load_text_map


class AStarTest(unittest.TestCase):
    def test_demo_map_reaches_goal_without_crossing_obstacles(self) -> None:
        map_path = Path(__file__).parent / "maps" / "demo_map.txt"
        grid, start, goal = load_text_map(map_path)

        path = plan_path(grid, start, goal)

        self.assertEqual(path[0], start)
        self.assertEqual(path[-1], goal)
        self.assertTrue(all(grid[y][x] == 0 for x, y in path))

    def test_returns_empty_list_when_no_path_exists(self) -> None:
        grid = [[0, 100, 0], [0, 100, 0], [0, 100, 0]]
        self.assertEqual(plan_path(grid, (0, 0), (2, 2)), [])


if __name__ == "__main__":
    unittest.main()
