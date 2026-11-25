"""Shortcut zigzags and isolated right-angle turns for execution output.

The input A* path is never changed. Each corner is checked against the occupancy
grid independently; an unsafe corner keeps its original right-angle bend.
"""

from math import cos, hypot, radians, sin


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


def smooth_path(grid, path, radius=None):
    """Return safe 30°/60° zigzag shortcuts or a 45° single-turn shortcut.

    An isolated 90° bend uses equal legs from the bend along its two straight
    runs. ``radius`` optionally caps those legs in cells. The original path
    remains untouched; use an inflated grid when robot width matters.
    """
    if radius is not None and radius <= 0:
        raise ValueError("radius must be positive")
    if len(path) < 3:
        return list(path)
    route = [path[0]]
    index = 1
    last_turn = 0
    last_turn_index = None
    last_turn_route_index = None
    while index < len(path) - 1:
        turn = _turn(path, index)
        if turn and last_turn and turn != last_turn:
            # The first bend may provide a clear ray even when the second
            # bend is too close to the wall or the end of the staircase.
            previous_shortcut = _zigzag_shortcut(grid, path, last_turn_index)
            current_shortcut = _zigzag_shortcut(grid, path, index)
            anchor_is_previous = previous_shortcut is not None and (
                current_shortcut is None or previous_shortcut[0] >= current_shortcut[0]
            )
            shortcut = previous_shortcut if anchor_is_previous else current_shortcut
            if shortcut is not None:
                end, _, point = shortcut
                if anchor_is_previous:
                    route = route[:last_turn_route_index + 1]
                else:
                    route.append(path[index])
                route.append(point)
                if hypot(point[0] - path[end + 1][0], point[1] - path[end + 1][1]) > 1e-9:
                    route.append(path[end + 1])
                index = end + 2
                last_turn = 0
                last_turn_index = None
                continue
        if turn:
            last_turn = turn
            last_turn_index = index
            last_turn_route_index = len(route)
        route.append(path[index])
        index += 1
    route.append(path[-1])
    turns = [index for index in range(1, len(route) - 1) if _turn(route, index)]
    if len(turns) != 1:
        return route
    corner_index = turns[0]
    before, corner, after = route[0], route[corner_index], route[-1]
    incoming = (corner[0] - before[0], corner[1] - before[1])
    outgoing = (after[0] - corner[0], after[1] - corner[1])
    first_leg = hypot(*incoming)
    second_leg = hypot(*outgoing)
    if (not first_leg or not second_leg or incoming[0] * outgoing[0] + incoming[1] * outgoing[1] != 0
            or not ((incoming[0] == 0 and outgoing[1] == 0)
                    or (incoming[1] == 0 and outgoing[0] == 0))):
        return route
    leg = min(first_leg, second_leg, radius if radius is not None else float("inf"))
    entry = (corner[0] - leg * incoming[0] / first_leg,
             corner[1] - leg * incoming[1] / first_leg)
    exit_point = (corner[0] + leg * outgoing[0] / second_leg,
                  corner[1] + leg * outgoing[1] / second_leg)
    candidate = [before, entry, exit_point, after]
    if all(_clear(grid, a, b) for a, b in zip(candidate, candidate[1:])):
        return [point for index, point in enumerate(candidate)
                if index == 0 or hypot(point[0] - candidate[index - 1][0],
                                       point[1] - candidate[index - 1][1]) > 1e-9]
    return route


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
