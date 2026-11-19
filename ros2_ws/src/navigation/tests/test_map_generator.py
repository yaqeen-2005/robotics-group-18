import tempfile
import unittest
from pathlib import Path

from common.cell_attributes import load_cell_attributes
from common.map_io import load_text_map
from ml.generate_maps import generate_template


class MapGeneratorTest(unittest.TestCase):
    def test_generated_map_and_attributes_can_be_loaded(self):
        with tempfile.TemporaryDirectory() as directory:
            output = generate_template("small_aisles_quiet", directory, seed=3)
            grid, _, _ = load_text_map(output / "map.txt")
            attributes = load_cell_attributes(
                output / "attributes.txt", len(grid[0]), len(grid)
            )
            self.assertEqual(attributes.width, len(grid[0]))
            self.assertTrue((output / "metadata.json").exists())


if __name__ == "__main__":
    unittest.main()
