"""Smooth right-angle grid turns into 30°/60° segments for route output only.

The input A* path is never changed. Each corner is checked against the occupancy
grid independently; an unsafe corner keeps its original right-angle bend.
"""

from math import cos, hypot, radians, sin, sqrt


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


def _turn(path, index):
    before, current, after = path[index - 1:index + 2]
    ax, ay = current[0] - before[0], current[1] - before[1]
    bx, by = after[0] - current[0], after[1] - current[1]
    cross = ax * by - ay * bx
    return (cross > 0) - (cross < 0)


def _ray_hit(origin, direction, start, end):
    dx, dy = direction
    if start[0] == end[0] and abs(dx) > 1e-9:
        distance = (start[0] - origin[0]) / dx
        point = (start[0], origin[1] + distance * dy)
        inside = min(start[1], end[1]) - 1e-9 <= point[1] <= max(start[1], end[1]) + 1e-9
    elif start[1] == end[1] and abs(dy) > 1e-9:
        distance = (start[1] - origin[1]) / dy
        point = (origin[0] + distance * dx, start[1])
        inside = min(start[0], end[0]) - 1e-9 <= point[0] <= max(start[0], end[0]) + 1e-9
    else:
        return None
    return (point, distance) if inside and distance > 2 else None


def _zigzag_shortcut(grid, path, anchor):
    origin = path[anchor]
    best = None
    for angle in range(0, 360, 30):
        if angle % 90 == 0:
            continue
        direction = (cos(radians(angle)), sin(radians(angle)))
        for index in range(anchor + 2, len(path) - 1):
            hit = _ray_hit(origin, direction, path[index], path[index + 1])
            if hit is None:
                continue
            point, distance = hit
            original_length = index - anchor + hypot(point[0] - path[index][0], point[1] - path[index][1])
            if original_length - distance < 0.5:
                continue
            if _clear(grid, origin, point) and _clear(grid, point, path[index + 1]):
                candidate = (index, distance, point)
                if best is None or candidate[:2] > best[:2]:
                    best = candidate
    return best


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
    index = 1
    last_turn = 0
    while index < len(path) - 1:
        turn = _turn(path, index)
        if turn and last_turn and turn != last_turn:
            shortcut = _zigzag_shortcut(grid, path, index)
            if shortcut is not None:
                end, _, point = shortcut
                route.append(path[index])
                route.append(point)
                if hypot(point[0] - path[end + 1][0], point[1] - path[end + 1][1]) > 1e-9:
                    route.append(path[end + 1])
                index = end + 2
                last_turn = 0
                continue
        if turn:
            last_turn = turn
        route.append(path[index])
        index += 1
    route.append(path[-1])
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
