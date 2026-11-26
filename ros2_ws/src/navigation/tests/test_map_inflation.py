import tempfile
import unittest
from pathlib import Path

from common.cell_attributes import (
    CellAttributeMap,
    load_cell_attributes,
    save_cell_attributes,
)
from common.inflate_map import inflate_map
from common.map_io import load_text_map, save_text_map


class MapInflationTest(unittest.TestCase):
    def test_inflates_obstacles_and_copies_source_attributes(self):
        grid = [[0, 0, 0], [0, 100, 0], [0, 0, 0]]
        values = [[[1.0, 0.0, 0.0, 0.0] for _ in row] for row in grid]
        values[1][1] = [0.6, 0.7, 0.8, 0.9]
        attributes = CellAttributeMap(3, 3, values)

        inflated, inflated_attributes = inflate_map(grid, attributes, 1)

        self.assertEqual(inflated, [[0, 100, 0], [100, 100, 100], [0, 100, 0]])
        self.assertEqual(inflated_attributes.values[0][1], [0.6, 0.7, 0.8, 0.9])
        self.assertEqual(grid[0][1], 0)

    def test_protected_points_remain_free(self):
        grid = [[0, 100, 0]]
        inflated, _ = inflate_map(grid, inflation_cells=1, protected_points={(0, 0)})
        self.assertEqual(inflated, [[0, 100, 100]])

    def test_saved_files_can_be_loaded_again(self):
        grid = [[0, 100, 0], [0, 0, 0]]
        attributes = CellAttributeMap(
            3, 2, [[[1.0, 0.0, 0.0, 0.0] for _ in row] for row in grid]
        )
        with tempfile.TemporaryDirectory() as directory:
            map_path = Path(directory) / "map.txt"
            attribute_path = Path(directory) / "attributes.txt"
            save_text_map(map_path, grid, (0, 0), (2, 1))
            save_cell_attributes(attribute_path, attributes)
            loaded, start, goal = load_text_map(map_path)
            loaded_attributes = load_cell_attributes(attribute_path, 3, 2)

        self.assertEqual((loaded, start, goal), (grid, (0, 0), (2, 1)))
        self.assertEqual(loaded_attributes.values, attributes.values)

    def test_walls_and_people_are_inflated_together(self):
        grid = [[0] * 7 for _ in range(7)]
        grid[1][1] = 100
        inflated, _ = inflate_map(grid, inflation_cells=1,
                                  dynamic_obstacles={(3, 3)})
        for point in ((1, 1), (0, 1), (2, 1), (3, 3), (2, 3), (4, 3), (3, 2), (3, 4)):
            self.assertEqual(inflated[point[1]][point[0]], 100)
        self.assertEqual(grid[3][3], 0)


if __name__ == "__main__":
    unittest.main()
