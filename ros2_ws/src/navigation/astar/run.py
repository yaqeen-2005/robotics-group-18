"""Command-line entry point for testing the A* planner."""

import argparse
import json
from pathlib import Path

from astar.a_star import plan_path
from common.cell_attributes import load_cell_attributes
from common.map_io import load_text_map
from common.weight_config import load_weight_config

DEFAULT_MAP = Path(__file__).parent.parent / "tests" / "maps" / "demo_map.txt"


def main() -> int:
    parser = argparse.ArgumentParser(description="Plan a path through a text map")
    parser.add_argument("map", nargs="?", default=DEFAULT_MAP, help="text map file")
    parser.add_argument("--attributes", help="optional ATTRIBUTES v2 file")
    parser.add_argument("--weights", help="optional JSON A* weight profile")
    parser.add_argument("--no-visual", action="store_true", help="print the path instead of plotting it")
    args = parser.parse_args()

    grid, start, goal = load_text_map(args.map)
    attributes = None
    if args.attributes:
        attributes = load_cell_attributes(args.attributes, len(grid[0]), len(grid))
    weights = load_weight_config(args.weights) if args.weights else None
    path = plan_path(grid, start, goal, attributes, weights)

    if args.no_visual:
        print(json.dumps(path))
    else:
        from astar.visualize import show_path

        show_path(grid, path, start, goal)
    return 0 if path else 1


if __name__ == "__main__":
    raise SystemExit(main())
