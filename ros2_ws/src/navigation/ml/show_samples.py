"""Compare generated obstacle maps and human-traffic probability layers."""

import argparse
from pathlib import Path

import matplotlib.pyplot as plt
from matplotlib.colors import ListedColormap

from common.cell_attributes import HUMAN_PROBABILITY, load_cell_attributes
from common.map_io import load_text_map


def main():
    parser = argparse.ArgumentParser(description="Compare generated map samples")
    parser.add_argument(
        "root", nargs="?", default=Path(__file__).parent / "samples"
    )
    args = parser.parse_args()

    samples = sorted(Path(args.root).glob("seed_*/large_open_busy"))
    if not samples:
        raise ValueError("no large_open_busy samples found")

    figure, axes = plt.subplots(len(samples), 2, figsize=(12, 3 * len(samples)))
    for row, sample in enumerate(samples):
        grid, start, goal = load_text_map(sample / "map.txt")
        attributes = load_cell_attributes(
            sample / "attributes.txt", len(grid[0]), len(grid)
        )
        obstacle_view = [
            [1 if cell >= 50 else 0 for cell in grid_row] for grid_row in grid
        ]
        axes[row][0].imshow(
            obstacle_view, cmap=ListedColormap(["white", "black"]), vmin=0, vmax=1
        )
        axes[row][0].scatter(*start, color="limegreen")
        axes[row][0].scatter(*goal, color="red")
        axes[row][0].set_title(f"{sample.parent.name}: obstacles")
        image = axes[row][1].imshow(
            attributes.layer(HUMAN_PROBABILITY), cmap="magma", vmin=0, vmax=1
        )
        axes[row][1].set_title(f"{sample.parent.name}: human probability")
        figure.colorbar(image, ax=axes[row][1])

    figure.tight_layout()
    plt.show()


if __name__ == "__main__":
    main()
