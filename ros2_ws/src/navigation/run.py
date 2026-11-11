"""Command-line entry point for testing the A* planner."""

import argparse
import json
from pathlib import Path

from a_star import plan_path
from confidence_io import load_confidence
from map_io import load_text_map

DEFAULT_MAP = Path(__file__).parent / "tests" / "maps" / "demo_map.txt"
DEFAULT_CONFIDENCE = Path(__file__).parent / "tests" / "maps" / "demo_confidence.txt"


def main() -> int:
    parser = argparse.ArgumentParser(description="Plan a path through a text map")
    parser.add_argument("map", nargs="?", default=DEFAULT_MAP, help="text map file")
    parser.add_argument(
        "confidence", nargs="?", default=DEFAULT_CONFIDENCE, help="confidence grid file"
    )
    parser.add_argument("--no-visual", action="store_true", help="print the path instead of plotting it")
    args = parser.parse_args()

    grid, start, goal = load_text_map(args.map)
    load_confidence(args.confidence, len(grid[0]), len(grid))
    path = plan_path(grid, start, goal)

    if args.no_visual:
        print(json.dumps(path))
    else:
        from visualize import show_path

        show_path(grid, path, start, goal)
    return 0 if path else 1


if __name__ == "__main__":
    raise SystemExit(main())
