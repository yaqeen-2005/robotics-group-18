import unittest

from common.path_manager import PathManager
from common.inflate_map import inflate_map


class PathManagerTest(unittest.TestCase):
    def test_non_human_map_change_adopts_new_path_immediately(self):
        manager = PathManager()
        clear = [[0, 0, 0], [0, 0, 0], [0, 0, 0]]
        manager.plan(clear, (0, 1), (2, 1))
        self.assertEqual(manager.raw_path(), [(0, 1), (1, 1), (2, 1)])

        changed = [[0, 0, 0], [0, 100, 0], [0, 0, 0]]
        result = manager.plan(changed, (0, 1), (2, 1), map_changed=True)

        self.assertEqual(result["action"], "REROUTE")
        self.assertNotIn((1, 1), result["path"])

    def test_human_obstacle_waits_below_threshold(self):
        manager = PathManager()
        grid = [[0, 0, 0]]
        manager.plan(grid, (0, 0), (2, 0))

        result = manager.plan(
            grid,
            (0, 0),
            (2, 0),
            human_blocking=True,
            human_distance=1.0,
            waiting_time=1.0,
        )

        self.assertEqual(result["action"], "WAIT")
        self.assertEqual(result["waypoints"], [])

    def test_human_obstacle_reroutes_away_from_old_path(self):
        manager = PathManager()
        grid = [[0, 0, 0], [0, 0, 0], [0, 0, 0]]
        manager.plan(grid, (0, 1), (2, 1))

        result = manager.plan(
            grid,
            (0, 1),
            (2, 1),
            human_blocking=True,
            human_distance=10.0,
            waiting_time=10.0,
        )

        self.assertEqual(result["action"], "REROUTE")
        self.assertNotIn((1, 1), result["path"])
        self.assertEqual(len(manager.previous_paths), 1)

    def test_human_obstacle_keeps_waiting_when_no_alternative_exists(self):
        manager = PathManager()
        grid = [[0, 0, 0]]
        manager.plan(grid, (0, 0), (2, 0))

        result = manager.plan(
            grid,
            (0, 0),
            (2, 0),
            human_blocking=True,
            human_distance=10.0,
            waiting_time=10.0,
        )

        self.assertEqual(result["action"], "WAIT")
        self.assertEqual(result["path"], [(0, 0), (1, 0), (2, 0)])

    def test_human_cell_is_blocked_during_reroute(self):
        manager = PathManager()
        grid = [[0, 0, 0], [0, 0, 0], [0, 0, 0]]
        manager.plan(grid, (0, 1), (2, 1))

        result = manager.plan(
            grid,
            (0, 1),
            (2, 1),
            human_blocking=True,
            human_distance=1.0,
            waiting_time=10.0,
            human_cells={(1, 0)},
        )

        self.assertNotIn((1, 0), result["path"])

    def test_human_and_door_trigger_one_route_around_both(self):
        grid = [[0] * 7 for _ in range(7)]
        manager = PathManager()
        manager.plan(grid, (1, 3), (5, 3))
        inflated, _ = inflate_map(grid, inflation_cells=1,
                                  dynamic_obstacles={(3, 3)})
        result = manager.plan(
            inflated, (1, 3), (5, 3), human_blocking=True,
            human_distance=2, waiting_time=10, human_cells={(3, 3)},
            door_cells={(3, 2)}, door_belief=0.9,
            door_block_rate=0.95,
        )
        self.assertEqual(result["action"], "REROUTE")
        self.assertTrue(all(inflated[y][x] < 50 for x, y in result["path"]))
        self.assertNotIn((3, 2), result["path"])
        self.assertEqual(manager.reroute_count, 1)

    def test_door_reroute_wins_while_human_waits(self):
        grid = [[0] * 7 for _ in range(7)]
        manager = PathManager()
        manager.plan(grid, (1, 3), (5, 3))
        result = manager.plan(
            grid, (1, 3), (5, 3), human_blocking=True,
            human_distance=2, waiting_time=1, human_cells={(2, 3)},
            door_cells={(3, 3)}, door_belief=0.9,
            door_block_rate=0.95,
        )
        self.assertEqual(result["action"], "REROUTE")
        self.assertNotIn((2, 3), result["path"])
        self.assertNotIn((3, 3), result["path"])


if __name__ == "__main__":
    unittest.main()
