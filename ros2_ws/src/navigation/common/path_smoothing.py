"""Smooth right-angle grid turns into 30°/60° segments for route output only.

The input A* path is never changed. Each corner is checked against the occupancy
grid independently; an unsafe corner keeps its original right-angle bend.
"""

from math import sqrt


def _clear(grid, start, end):
    # Sample densely enough to catch every grid cell crossed by these short segments.
    steps = max(1, int(max(abs(end[0] - start[0]), abs(end[1] - start[1])) * 100))
    for step in range(steps + 1):
        fraction = step / steps
        x = int(start[0] + (end[0] - start[0]) * fraction + 0.5)
        y = int(start[1] + (end[1] - start[1]) * fraction + 0.5)
        if not (0 <= y < len(grid) and 0 <= x < len(grid[0]) and 0 <= grid[y][x] < 50):
            return False
    return True


def smooth_path(grid, path, radius=0.4):
    """Return float waypoints; only 90° bends get 30°/60° shortcuts.

    ``radius`` is in cells and must stay below half a cell so adjacent bends
    cannot overlap. Use an already inflated grid to account for robot width.
    """
    if not 0 < radius < 0.5:
        raise ValueError("radius must be between 0 and 0.5 cells")
    if len(path) < 3:
        return list(path)
    output = [path[0]]
    first_length = (sqrt(3) - 1) / (sqrt(3) - 1 / sqrt(3))
    for before, corner, after in zip(path, path[1:], path[2:]):
        incoming = (corner[0] - before[0], corner[1] - before[1])
        outgoing = (after[0] - corner[0], after[1] - corner[1])
        if abs(incoming[0]) + abs(incoming[1]) != 1 or abs(outgoing[0]) + abs(outgoing[1]) != 1 or incoming[0] * outgoing[0] + incoming[1] * outgoing[1] != 0:
            output.append(corner)
            continue
        entry = (corner[0] - radius * incoming[0], corner[1] - radius * incoming[1])
        exit_point = (corner[0] + radius * outgoing[0], corner[1] + radius * outgoing[1])
        middle = (
            entry[0] + radius * (first_length * incoming[0] + (1 - first_length) * outgoing[0]),
            entry[1] + radius * (first_length * incoming[1] + (1 - first_length) * outgoing[1]),
        )
        if all(_clear(grid, a, b) for a, b in ((output[-1], entry), (entry, middle), (middle, exit_point))):
            output.extend((entry, middle, exit_point))
        else:
            output.append(corner)
    output.append(path[-1])
    return output
