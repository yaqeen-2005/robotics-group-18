"""A* path planning on a 2D occupancy grid."""

from heapq import heappop, heappush
from itertools import count

from common.cell_attributes import (
    BLOCKAGE_PROBABILITY,
    CONFIDENCE,
    HUMAN_PROBABILITY,
    ROBOT_ENCOUNTER_PROBABILITY,
    DOOR_BELIEF,
    BLOCK_RATE,
    CellAttributeMap,
)

GridPoint = tuple[int, int]

# Default hand-tuned weights. A trained model can provide replacements.
DEFAULT_WEIGHTS = {
    "distance": 1.0,
    "blockage": 4.0,
    "human": 6.0,
    "uncertainty": 3.0,
    "robot_encounter": 4.0,
    "turn": 0.5,
    "heuristic": 1.0,
    "door": 8.0,
    "repeat": 2.0,
}


def plan_path(
    grid: list[list[int]],
    start: GridPoint,
    goal: GridPoint,
    attributes: CellAttributeMap | None = None,
    weights: dict | None = None,
    start_direction: GridPoint | None = None,
    blocked_cells=None,
    repeat_counts=None,
) -> list[GridPoint]:
    """Return the lowest-cost 4-connected path, or an empty list.

    Points use ``(x, y)`` coordinates and cells are read as ``grid[y][x]``.
    Values from 0 to 49 are free; 50 to 100 and -1 (unknown) are blocked.
    """
    if not grid or not grid[0] or any(len(row) != len(grid[0]) for row in grid):
        raise ValueError("grid must be a non-empty rectangle")

    width, height = len(grid[0]), len(grid)
    if attributes is not None and (
        attributes.width != width or attributes.height != height
    ):
        raise ValueError("attribute dimensions do not match grid dimensions")

    active_weights = DEFAULT_WEIGHTS.copy()
    if weights is not None:
        unknown_names = set(weights) - set(DEFAULT_WEIGHTS)
        if unknown_names:
            raise ValueError(f"unknown weight names: {sorted(unknown_names)}")
        active_weights.update(weights)
    if any(value < 0 for value in active_weights.values()):
        raise ValueError("weights must not be negative")
    if start_direction not in (None, (1, 0), (-1, 0), (0, 1), (0, -1)):
        raise ValueError("start_direction must be a cardinal direction or None")

    blocked = set(blocked_cells or [])

    def is_free(point: GridPoint) -> bool:
        x, y = point
        return (
            0 <= x < width
            and 0 <= y < height
            and 0 <= grid[y][x] < 50
            and point not in blocked
        )

    if not is_free(start) or not is_free(goal):
        raise ValueError("start and goal must be free cells inside the grid")

    initial_direction = start_direction or (0, 0)
    start_state = (start[0], start[1], initial_direction[0], initial_direction[1])
    frontier = []
    order = count()
    heappush(frontier, (0, next(order), start_state))
    came_from = {start_state: None}
    cost = {start_state: 0}

    while frontier:
        _, _, current_state = heappop(frontier)
        x, y, direction_x, direction_y = current_state
        if (x, y) == goal:
            path = []
            while current_state is not None:
                path.append((current_state[0], current_state[1]))
                current_state = came_from[current_state]
            return path[::-1]

        for move_x, move_y in ((1, 0), (-1, 0), (0, 1), (0, -1)):
            neighbour = (x + move_x, y + move_y)
            if not is_free(neighbour):
                continue
            step_cost = active_weights["distance"]
            if (direction_x, direction_y) != (0, 0) and (
                move_x,
                move_y,
            ) != (direction_x, direction_y):
                multiplier = 2 if (move_x, move_y) == (-direction_x, -direction_y) else 1
                step_cost += active_weights["turn"] * multiplier
            if attributes is not None:
                nx, ny = neighbour
                cell = attributes.values[ny][nx]
                step_cost += active_weights["blockage"] * cell[BLOCKAGE_PROBABILITY]
                step_cost += active_weights["human"] * cell[HUMAN_PROBABILITY]
                step_cost += active_weights["uncertainty"] * (1.0 - cell[CONFIDENCE])
                step_cost += (
                    active_weights["robot_encounter"]
                    * cell[ROBOT_ENCOUNTER_PROBABILITY]
                )
                if len(cell) > DOOR_BELIEF:
                    # Closed fraction estimates expected delay at a door-like cell.
                    step_cost += active_weights["door"] * cell[DOOR_BELIEF] * cell[BLOCK_RATE]
            if repeat_counts:
                step_cost += active_weights["repeat"] * repeat_counts.get(neighbour, 0)
            neighbour_state = (neighbour[0], neighbour[1], move_x, move_y)
            new_cost = cost[current_state] + step_cost
            if neighbour_state not in cost or new_cost < cost[neighbour_state]:
                cost[neighbour_state] = new_cost
                came_from[neighbour_state] = current_state
                distance = abs(goal[0] - neighbour[0]) + abs(goal[1] - neighbour[1])
                heuristic = (
                    distance
                    * active_weights["distance"]
                    * active_weights["heuristic"]
                )
                heappush(
                    frontier,
                    (new_cost + heuristic, next(order), neighbour_state),
                )

    return []
