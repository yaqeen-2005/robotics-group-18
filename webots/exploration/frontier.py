"""Detect exploration frontiers in an occupancy grid, without simulator access."""


def find_frontiers(grid):
    """Return free cells bordering unknown space as row-ordered (x, y) pairs.

    The rectangular grid uses grid[y][x]: -1 unknown, 0 free, 100 occupied.
    Only the four cardinal neighbours count; outside the grid is not unknown.
    Empty grids return an empty list. Ragged grids and unsupported values raise
    ValueError. Input cells are never modified.
    """
    if len(grid) == 0:
        return []
    width = len(grid[0])
    if any(len(row) != width for row in grid):
        raise ValueError("grid must be rectangular")
    if any(cell not in (-1, 0, 100) for row in grid for cell in row):
        raise ValueError("grid cells must be -1 (unknown), 0 (free), or 100 (occupied)")

    height = len(grid)
    frontiers = []
    for y, row in enumerate(grid):
        for x, cell in enumerate(row):
            if cell != 0:
                continue
            for dx, dy in ((-1, 0), (1, 0), (0, -1), (0, 1)):
                nx, ny = x + dx, y + dy
                if 0 <= nx < width and 0 <= ny < height and grid[ny][nx] == -1:
                    frontiers.append((x, y))
                    break
    return frontiers
