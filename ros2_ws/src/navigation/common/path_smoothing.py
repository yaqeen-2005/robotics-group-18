"""Smooth right-angle grid turns into 30°/60° segments for route output only.

The input A* path is never changed. Each corner is checked against the occupancy
grid independently; an unsafe corner keeps its original right-angle bend.
"""

from math import hypot, sqrt


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


def _long_diagonal(grid, path, first, last):
    start, end = path[first], path[last]
    dx, dy = end[0] - start[0], end[1] - start[1]
    if not dx or not dy or last - first < 6:
        return None
    if abs(dx) >= abs(dy):
        projected = (end[0], start[1] + (1 if dy > 0 else -1) * abs(dx) / sqrt(3))
        error = abs(projected[1] - end[1])
    else:
        projected = (start[0] + (1 if dx > 0 else -1) * abs(dy) / sqrt(3), end[1])
        error = abs(projected[0] - end[0])
    if error > 0.45:
        return None
    length = sqrt(dx * dx + dy * dy)
    if any(abs(dx * (point[1] - start[1]) - dy * (point[0] - start[0])) / length > 2.1
           for point in path[first + 1:last]):
        return None
    if _clear(grid, start, projected) and _clear(grid, projected, end):
        return projected
    return None


def smooth_path(grid, path, radius=0.4):
    """Return collision-checked 30°/60° shortcuts and rounded grid bends.

    ``radius`` is in cells and must stay below half a cell so adjacent bends
    cannot overlap. Use an already inflated grid to account for robot width.
    """
    if not 0 < radius < 0.5:
        raise ValueError("radius must be between 0 and 0.5 cells")
    if len(path) < 3:
        return list(path)
    route = [path[0]]
    index = 0
    while index < len(path) - 1:
        shortcut = None
        for end in range(len(path) - 1, index + 5, -1):
            shortcut = _long_diagonal(grid, path, index, end)
            if shortcut is not None:
                route.extend((shortcut, path[end]))
                index = end
                break
        if shortcut is None:
            index += 1
            route.append(path[index])
    path = route
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


def sample_waypoints(waypoints, step=0.8):
    """Return continuous positions along the route actually sent for execution."""
    if step <= 0:
        raise ValueError("step must be positive")
    if not waypoints:
        return []
    positions = [tuple(waypoints[0])]
    for start, end in zip(waypoints, waypoints[1:]):
        length = hypot(end[0] - start[0], end[1] - start[1])
        segments = max(1, int(length / step + 0.999999))
        for index in range(1, segments + 1):
            fraction = index / segments
            positions.append((start[0] + (end[0] - start[0]) * fraction,
                              start[1] + (end[1] - start[1]) * fraction))
    return positions
