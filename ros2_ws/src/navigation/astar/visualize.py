"""Display a grid and planned path with matplotlib."""

import matplotlib.pyplot as plt
from matplotlib.colors import ListedColormap

def show_path(
    grid: list[list[int]],
    path: list[tuple[int, int]],
    start: tuple[int, int],
    goal: tuple[int, int],
) -> None:
    display = [[1 if cell < 0 or cell >= 50 else 0 for cell in row] for row in grid]
    for x, y in path:
        display[y][x] = 2
    sx, sy = start
    gx, gy = goal
    display[sy][sx] = 3
    display[gy][gx] = 4

    plt.imshow(display, cmap=ListedColormap(["white", "black", "dodgerblue", "limegreen", "red"]))
    plt.xticks(range(len(grid[0])))
    plt.yticks(range(len(grid)))
    plt.grid(color="lightgray", linewidth=0.5)
    plt.title("A* path: green=start, red=goal" if path else "No path found")
    plt.show()
