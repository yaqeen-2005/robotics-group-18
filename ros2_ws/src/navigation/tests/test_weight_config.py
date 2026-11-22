import unittest
from pathlib import Path

from astar.a_star import DEFAULT_WEIGHTS
from common.weight_config import load_weight_config


class WeightConfigTest(unittest.TestCase):
    def test_generated_profiles_define_every_planner_weight(self):
        root = Path(__file__).parent.parent / "ml" / "weights"
        for name in ("balanced.json", "human_safe.json"):
            weights = load_weight_config(root / name)
            self.assertEqual(set(weights), set(DEFAULT_WEIGHTS))
            self.assertTrue(all(value >= 0 for value in weights.values()))


if __name__ == "__main__":
    unittest.main()
