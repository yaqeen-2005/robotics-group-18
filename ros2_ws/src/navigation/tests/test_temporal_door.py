import tempfile
import unittest
from pathlib import Path

from common.cell_attributes import CellAttributeMap, DOOR_BELIEF, load_cell_attributes, save_cell_attributes
from common.temporal_traversability import TemporalTraversabilityLayer
from common.special_obstacles import SpecialObstacles
from ml.door_belief import DoorPredictor, MODEL
from common.path_manager import PathManager
from astar.a_star import plan_path


class TemporalDoorTest(unittest.TestCase):
    def test_only_fresh_observations_update_v2_map(self):
        attributes = CellAttributeMap(2, 1, [[[1, 0, 0, 0], [1, 0, 0, 0]]])
        layer = TemporalTraversabilityLayer(attributes)
        self.assertEqual(layer.update(0, {(0, 0): True}), 1)
        self.assertEqual(layer.update(0.2, {(0, 0): False}), 0)
        self.assertEqual(layer.update(0.5, {(0, 0): False}), 1)
        self.assertEqual(attributes.values[0][0][4:9], [2, 1, 0.5, 1, 0])
        self.assertEqual(attributes.values[0][1], [1, 0, 0, 0])
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "attributes.txt"
            # Promote remaining legacy cells before writing a single v3 map.
            attributes.update_cell(1, 0, DOOR_BELIEF, 0)
            save_cell_attributes(path, attributes)
            self.assertEqual(load_cell_attributes(path, 2, 1).values, attributes.values)

    def test_trained_model_and_special_rules(self):
        attributes = CellAttributeMap(1, 1, [[[1, 0, 0, 0]]])
        predictor = DoorPredictor(MODEL)
        value = predictor.predict(attributes, 0, 0)
        self.assertTrue(0 <= value <= 1)
        rules = SpecialObstacles()
        self.assertEqual(rules.decide("door", distance=2, belief=0.9, block_rate=0.3), "WAIT")
        self.assertEqual(rules.decide("door", distance=2, belief=0.9, block_rate=0.95), "REROUTE")
        self.assertEqual(rules.decide("human", distance=1, human_reroute_score=0.7,
                                      backtracks=2), "WAIT")
        self.assertEqual(rules.decide("door", distance=2, belief=0.9, block_rate=0.85,
                                      backtracks=2), "WAIT")

    def test_door_cost_and_replanning_share_astar(self):
        grid = [[0] * 5 for _ in range(3)]
        cells = [[[1, 0, 0, 0] for _ in row] for row in grid]
        cells[1][2] = [1, 0, 0, 0, 20, 18, 0.9, 5, 1, 0.95]
        attributes = CellAttributeMap(5, 3, cells)
        self.assertNotIn((2, 1), plan_path(grid, (0, 1), (4, 1), attributes))
        manager = PathManager()
        manager.plan(grid, (0, 1), (4, 1))
        result = manager.plan(grid, (0, 1), (4, 1), attributes,
                              door_cells={(2, 1)})
        self.assertEqual(result["action"], "REROUTE")
        self.assertNotIn((2, 1), result["path"])
        manager.plan(grid, (1, 1), (4, 1))
        manager.plan(grid, (0, 1), (4, 1))
        self.assertEqual(manager.backtrack_counts[(0, 1)], 1)


if __name__ == "__main__":
    unittest.main()
