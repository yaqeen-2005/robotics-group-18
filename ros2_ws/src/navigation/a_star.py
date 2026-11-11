"""A* path planning on a 2D occupancy grid."""

from heapq import heappop, heappush
from itertools import count

GridPoint = tuple[int, int]


def plan_path(
    grid: list[list[int]], start: GridPoint, goal: GridPoint
) -> list[GridPoint]:
    """Return a shortest 4-connected path, or an empty list if none exists.

    Points use ``(x, y)`` coordinates and cells are read as ``grid[y][x]``.
    Values from 0 to 49 are free; 50 to 100 and -1 (unknown) are blocked.
    """
    if not grid or not grid[0] or any(len(row) != len(grid[0]) for row in grid):
        raise ValueError("grid must be a non-empty rectangle")

    width, height = len(grid[0]), len(grid)

    def is_free(point: GridPoint) -> bool:
        x, y = point
        return 0 <= x < width and 0 <= y < height and 0 <= grid[y][x] < 50

    if not is_free(start) or not is_free(goal):
        raise ValueError("start and goal must be free cells inside the grid")

    frontier: list[tuple[int, int, GridPoint]] = []
    order = count()
    heappush(frontier, (0, next(order), start))
    came_from: dict[GridPoint, GridPoint | None] = {start: None}
    cost = {start: 0}

    while frontier:
        _, _, current = heappop(frontier)
        if current == goal:
            path = []
            while current is not None:
                path.append(current)
                current = came_from[current]
            return path[::-1]

        x, y = current
        for neighbour in ((x + 1, y), (x - 1, y), (x, y + 1), (x, y - 1)):
            if not is_free(neighbour):
                continue
            new_cost = cost[current] + 1
            if neighbour not in cost or new_cost < cost[neighbour]:
                cost[neighbour] = new_cost
                came_from[neighbour] = current
                heuristic = abs(goal[0] - neighbour[0]) + abs(goal[1] - neighbour[1])
                heappush(frontier, (new_cost + heuristic, next(order), neighbour))

    return []
