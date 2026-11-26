import unittest
import math

from common.cell_attributes import CellAttributeMap
from exploration.frontier_explorer import FrontierExplorer


def attributes_for(grid, confidence=1.0):
    values = [[[confidence, 0.0, 0.0, 0.0] for _ in row] for row in grid]
    return CellAttributeMap(len(grid[0]), len(grid), values)


class FrontierExplorerTest(unittest.TestCase):
    def test_explores_when_goal_is_unknown(self):
        grid = [[0, 0, -1], [0, 100, -1], [0, 0, -1]]
        explorer = FrontierExplorer()
        explorer.update_map(grid, attributes_for(grid))
        explorer.update_position((0, 0), 0.9)

        move = explorer.next_move((2, 0))

        self.assertEqual(move["delta"], (1, 0))
        self.assertEqual(move["target"], (1, 0))
        self.assertEqual(move["mode"], "explore")

    def test_returns_none_when_confirmed_goal_is_unreachable(self):
        grid = [[0, 100, 0], [0, 100, 0]]
        explorer = FrontierExplorer()
        explorer.update_map(grid, attributes_for(grid))
        explorer.update_position((0, 0))
        self.assertIsNone(explorer.next_move((2, 0)))

    def test_step_size_changes_next_target(self):
        grid = [[0, 0, 0, 0]]
        explorer = FrontierExplorer(step_size=2)
        explorer.update_map(grid, attributes_for(grid))
        explorer.update_position((0, 0))
        self.assertEqual(explorer.next_move((3, 0))["target"], (2, 0))

    def test_next_move_reports_heading_and_turn(self):
        grid = [[0, 0], [0, 0]]
        explorer = FrontierExplorer()
        explorer.update_map(grid, attributes_for(grid))
        explorer.update_position((0, 1), yaw=0.0)

        move = explorer.next_move((0, 0))

        self.assertAlmostEqual(move["heading_rad"], -math.pi / 2)
        self.assertAlmostEqual(move["turn_rad"], -math.pi / 2)


if __name__ == "__main__":
    unittest.main()
