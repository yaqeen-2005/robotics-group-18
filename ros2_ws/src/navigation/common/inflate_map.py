"""Inflate obstacles so grid paths leave room for the robot and its cargo."""

import argparse
from copy import deepcopy

from common.cell_attributes import (
    CellAttributeMap,
    load_cell_attributes,
    save_cell_attributes,
)
from common.map_io import load_text_map, save_text_map


def inflate_map(
    grid,
    attributes=None,
    inflation_cells=1,
    protected_points=None,
    dynamic_obstacles=None,
):
    """Inflate walls and current dynamic obstacle cells in one map pass."""
    if not grid or not grid[0] or any(len(row) != len(grid[0]) for row in grid):
        raise ValueError("grid must be a non-empty rectangle")
    if not isinstance(inflation_cells, int) or inflation_cells < 0:
        raise ValueError("inflation_cells must be a non-negative integer")

    width, height = len(grid[0]), len(grid)
    if attributes is not None and (
        attributes.width != width or attributes.height != height
    ):
        raise ValueError("attribute dimensions do not match grid dimensions")

    protected = set(protected_points or [])
    inflated_grid = deepcopy(grid)
    inflated_attributes = None
    if attributes is not None:
        inflated_attributes = CellAttributeMap(
            width, height, deepcopy(attributes.values)
        )

    obstacles = [
        (x, y)
        for y, row in enumerate(grid)
        for x, cell in enumerate(row)
        if 50 <= cell <= 100
    ]
    for point in dynamic_obstacles or ():
        x, y = point
        if not (0 <= x < width and 0 <= y < height):
            raise ValueError("dynamic obstacle is outside the grid")
        obstacles.append((x, y))
    source_for_cell = {}
    radius_squared = inflation_cells * inflation_cells
    for obstacle_x, obstacle_y in obstacles:
        for dy in range(-inflation_cells, inflation_cells + 1):
            for dx in range(-inflation_cells, inflation_cells + 1):
                if dx * dx + dy * dy > radius_squared:
                    continue
                x, y = obstacle_x + dx, obstacle_y + dy
                if not (0 <= x < width and 0 <= y < height):
                    continue
                if (x, y) in protected or grid[y][x] == -1:
                    continue
                distance = dx * dx + dy * dy
                previous = source_for_cell.get((x, y))
                if previous is None or distance < previous[0]:
                    source_for_cell[(x, y)] = (distance, obstacle_x, obstacle_y)

    for (x, y), (_, source_x, source_y) in source_for_cell.items():
        inflated_grid[y][x] = 100
        if inflated_attributes is not None:
            inflated_attributes.values[y][x] = deepcopy(
                attributes.values[source_y][source_x]
            )
    return inflated_grid, inflated_attributes


def main():
    parser = argparse.ArgumentParser(description="Inflate obstacles in a text map")
    parser.add_argument("map", help="input MAP v1 file")
    parser.add_argument("attributes", help="input ATTRIBUTES v2 file")
    parser.add_argument("--cells", type=int, required=True, help="inflation radius in cells")
    parser.add_argument("--output-map", required=True, help="output MAP v1 file")
    parser.add_argument(
        "--output-attributes", required=True, help="output ATTRIBUTES v2 file"
    )
    args = parser.parse_args()

    grid, start, goal = load_text_map(args.map)
    attributes = load_cell_attributes(args.attributes, len(grid[0]), len(grid))
    inflated_grid, inflated_attributes = inflate_map(
        grid,
        attributes,
        args.cells,
        protected_points={start, goal},
    )
    save_text_map(args.output_map, inflated_grid, start, goal)
    save_cell_attributes(args.output_attributes, inflated_attributes)


if __name__ == "__main__":
    main()
