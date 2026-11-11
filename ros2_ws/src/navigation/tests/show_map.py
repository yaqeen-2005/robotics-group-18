"""Reconstruct and display a flattened MAP v1 file."""

import argparse
import sys
from pathlib import Path

import matplotlib.pyplot as plt
from matplotlib.colors import ListedColormap

NAVIGATION_DIR = Path(__file__).parent.parent
sys.path.insert(0, str(NAVIGATION_DIR))

from map_io import load_text_map  # noqa: E402


def main() -> None:
    parser = argparse.ArgumentParser(description="Display a MAP v1 text file")
    parser.add_argument(
        "map",
        nargs="?",
        default=Path(__file__).parent / "maps" / "demo_map.txt",
    )
    args = parser.parse_args()

    grid, start, goal = load_text_map(args.map)
    display = [[1 if cell < 0 or cell >= 50 else 0 for cell in row] for row in grid]
    plt.imshow(display, cmap=ListedColormap(["white", "black"]))
    plt.scatter(*start, color="limegreen", label="Start")
    plt.scatter(*goal, color="red", label="Goal")
    plt.xticks(range(len(grid[0])))
    plt.yticks(range(len(grid)))
    plt.grid(color="lightgray", linewidth=0.5)
    plt.legend()
    plt.title(f"{len(grid[0])} x {len(grid)} map")
    plt.show()


if __name__ == "__main__":
    main()
