import tempfile
import unittest
from pathlib import Path

from ml.train_weights import train


class TrainingTest(unittest.TestCase):
    def test_small_training_run_writes_both_profiles(self):
        navigation = Path(__file__).parent.parent
        archive = navigation / "ml" / "training_data" / "factory_busy_v1.zip"
        with tempfile.TemporaryDirectory() as directory:
            report = train(archive, directory, trials=2, task_limit=3, seed=2)
            output = Path(directory)
            self.assertTrue((output / "balanced.json").exists())
            self.assertTrue((output / "human_safe.json").exists())
            self.assertEqual(set(report), {"balanced", "human_safe"})


if __name__ == "__main__":
    unittest.main()
