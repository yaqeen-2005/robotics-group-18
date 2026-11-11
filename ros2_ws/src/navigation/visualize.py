"""Run a small A* example and display it with matplotlib."""

import matplotlib.pyplot as plt
from matplotlib.colors import ListedColormap

from a_star import plan_path


def show_path(grid: list[list[int]], path: list[tuple[int, int]]) -> None:
    display = [[1 if cell < 0 or cell >= 50 else 0 for cell in row] for row in grid]
    for x, y in path:
        display[y][x] = 2
    sx, sy = path[0]
    gx, gy = path[-1]
    display[sy][sx] = 3
    display[gy][gx] = 4

    plt.imshow(display, cmap=ListedColormap(["white", "black", "dodgerblue", "limegreen", "red"]))
    plt.xticks(range(len(grid[0])))
    plt.yticks(range(len(grid)))
    plt.grid(color="lightgray", linewidth=0.5)
    plt.title("A* path: green=start, red=goal")
    plt.show()


if __name__ == "__main__":
    demo_grid = [
        [0, 0, 0, 0, 0, 0, 0, 0],
        [0, 0, 100, 100, 100, 0, 0, 0],
        [0, 0, 0, 0, 100, 0, 0, 0],
        [0, 100, 100, 0, 100, 0, 100, 0],
        [0, 0, 0, 0, 0, 0, 100, 0],
        [0, 0, 0, 0, 0, 0, 0, 0],
    ]
    show_path(demo_grid, plan_path(demo_grid, (0, 0), (7, 5)))
