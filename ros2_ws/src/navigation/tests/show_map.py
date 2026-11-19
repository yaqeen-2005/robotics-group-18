"""Display a MAP v1 file and its optional per-cell attribute layers."""

import argparse
import sys
from pathlib import Path

import matplotlib.pyplot as plt
from matplotlib.colors import ListedColormap

NAVIGATION_DIR = Path(__file__).parent.parent
sys.path.insert(0, str(NAVIGATION_DIR))

from cell_attributes import ATTRIBUTE_NAMES, load_cell_attributes  # noqa: E402
from map_io import load_text_map  # noqa: E402


def main() -> None:
    parser = argparse.ArgumentParser(description="Display a MAP v1 text file")
    parser.add_argument(
        "map",
        nargs="?",
        default=Path(__file__).parent / "maps" / "demo_map.txt",
    )
    parser.add_argument(
        "attributes",
        nargs="?",
        default=Path(__file__).parent / "maps" / "demo_attributes.txt",
    )
    args = parser.parse_args()

    grid, start, goal = load_text_map(args.map)
    attributes = load_cell_attributes(args.attributes, len(grid[0]), len(grid))
    display = [[1 if cell < 0 or cell >= 50 else 0 for cell in row] for row in grid]
    figure, axes = plt.subplots(1, 4, figsize=(16, 4))
    axes[0].imshow(display, cmap=ListedColormap(["white", "black"]))
    axes[0].scatter(*start, color="limegreen", label="Start")
    axes[0].scatter(*goal, color="red", label="Goal")
    axes[0].legend()
    axes[0].set_title(f"{len(grid[0])} x {len(grid)} map")

    for dimension, name in enumerate(ATTRIBUTE_NAMES):
        image = axes[dimension + 1].imshow(
            attributes.layer(dimension), cmap="viridis", vmin=0.0, vmax=1.0
        )
        axes[dimension + 1].set_title(name.replace("_", " ").title())
        figure.colorbar(image, ax=axes[dimension + 1])

    for axis in axes:
        axis.set_xticks(range(len(grid[0])))
        axis.set_yticks(range(len(grid)))
        axis.grid(color="lightgray", linewidth=0.5)
    figure.tight_layout()
    plt.show()


if __name__ == "__main__":
    main()
