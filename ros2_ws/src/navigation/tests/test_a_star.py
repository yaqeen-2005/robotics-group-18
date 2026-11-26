import unittest
from pathlib import Path

from astar.a_star import plan_path
from common.cell_attributes import (
    BLOCKAGE_PROBABILITY,
    HUMAN_PROBABILITY,
    CellAttributeMap,
    load_cell_attributes,
)
from common.map_io import load_text_map


class AStarTest(unittest.TestCase):
    def test_demo_map_reaches_goal_without_crossing_obstacles(self) -> None:
        map_path = Path(__file__).parent / "maps" / "demo_map.txt"
        grid, start, goal = load_text_map(map_path)

        path = plan_path(grid, start, goal)

        self.assertEqual(path[0], start)
        self.assertEqual(path[-1], goal)
        self.assertTrue(all(grid[y][x] == 0 for x, y in path))

    def test_attribute_grid_matches_map_and_updates_a_cell(self) -> None:
        maps = Path(__file__).parent / "maps"
        grid, _, _ = load_text_map(maps / "demo_map.txt")
        attributes = load_cell_attributes(
            maps / "demo_attributes.txt", len(grid[0]), len(grid)
        )
        attributes.update_cell(2, 1, BLOCKAGE_PROBABILITY, 0.75)
        self.assertEqual(attributes.values[1][2][BLOCKAGE_PROBABILITY], 0.75)

    def test_returns_empty_list_when_no_path_exists(self) -> None:
        grid = [[0, 100, 0], [0, 100, 0], [0, 100, 0]]
        self.assertEqual(plan_path(grid, (0, 0), (2, 2)), [])

    def test_weighted_path_avoids_high_human_risk(self) -> None:
        grid = [[0, 0, 0], [0, 0, 0], [0, 0, 0]]
        values = [[[1.0, 0.0, 0.0, 0.0] for _ in row] for row in grid]
        attributes = CellAttributeMap(3, 3, values)
        attributes.update_cell(1, 1, HUMAN_PROBABILITY, 1.0)

        path = plan_path(grid, (0, 1), (2, 1), attributes)

        self.assertNotIn((1, 1), path)

    def test_turn_weight_can_prefer_a_longer_straighter_path(self) -> None:
        grid = [
            [100, 100, 100, 100, 100, 100, 100],
            [100, 0, 0, 0, 100, 100, 100],
            [0, 0, 100, 0, 0, 0, 0],
            [0, 100, 100, 100, 100, 100, 0],
            [0, 0, 0, 0, 0, 0, 0],
        ]

        shortest = plan_path(grid, (0, 2), (6, 2), weights={"turn": 0.0})
        straighter = plan_path(grid, (0, 2), (6, 2), weights={"turn": 2.0})

        self.assertLess(len(shortest), len(straighter))
        self.assertIn((3, 1), shortest)
        self.assertIn((3, 4), straighter)

    def test_temporarily_blocked_cells_are_avoided(self) -> None:
        grid = [[0, 0, 0], [0, 0, 0]]
        path = plan_path(grid, (0, 0), (2, 0), blocked_cells={(1, 0)})
        self.assertNotIn((1, 0), path)


if __name__ == "__main__":
    unittest.main()
