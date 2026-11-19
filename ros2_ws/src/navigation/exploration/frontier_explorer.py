"""Choose simple reachable frontier targets for unknown-map exploration."""

from astar.a_star import plan_path
from common.cell_attributes import CONFIDENCE, CellAttributeMap


class FrontierExplorer:
    def __init__(self, step_size: int = 1, confidence_threshold: float = 0.5):
        self.set_step_size(step_size)
        self.set_confidence_threshold(confidence_threshold)
        self.grid = None
        self.attributes = None
        self.position = None
        self.position_confidence = None

    def set_step_size(self, step_size: int) -> None:
        if step_size < 1:
            raise ValueError("step_size must be at least 1")
        self.step_size = step_size

    def set_confidence_threshold(self, threshold: float) -> None:
        if not 0.0 <= threshold <= 1.0:
            raise ValueError("confidence_threshold must be between 0.0 and 1.0")
        self.confidence_threshold = threshold

    def update_map(
        self, grid: list[list[int]], attributes: CellAttributeMap
    ) -> None:
        if not grid or not grid[0] or any(len(row) != len(grid[0]) for row in grid):
            raise ValueError("grid must be a non-empty rectangle")
        if attributes.width != len(grid[0]) or attributes.height != len(grid):
            raise ValueError("attribute dimensions do not match grid dimensions")
        self.grid = grid
        self.attributes = attributes

    def update_position(
        self, position: tuple[int, int], confidence: float = 1.0
    ) -> None:
        if not 0.0 <= confidence <= 1.0:
            raise ValueError("position confidence must be between 0.0 and 1.0")
        self.position = position
        self.position_confidence = confidence

    def next_move(self, goal: tuple[int, int] | None = None) -> dict | None:
        """Move toward a goal, or explore uncertain map cells if A* cannot reach it."""
        self._check_ready()
        if goal is not None:
            try:
                path = plan_path(self.grid, self.position, goal, self.attributes)
            except ValueError:
                path = []
            if len(path) > 1:
                return self._move_from_path(path, "goal")
            if path or not self._needs_exploration():
                return None
        return self.explore()

    def explore(self) -> dict | None:
        """Move toward the nearest reachable unknown or low-confidence frontier."""
        self._check_ready()
        candidates = self._frontier_candidates()
        candidates.sort(key=self._distance_from_position)
        for candidate in candidates:
            path = plan_path(self.grid, self.position, candidate, self.attributes)
            if len(path) > 1:
                return self._move_from_path(path, "explore")
        return None

    def _move_from_path(self, path: list[tuple[int, int]], mode: str) -> dict:
        target = path[min(self.step_size, len(path) - 1)]
        return {
            "delta": (target[0] - self.position[0], target[1] - self.position[1]),
            "target": target,
            "mode": mode,
        }

    def _needs_exploration(self) -> bool:
        for y, row in enumerate(self.grid):
            for x, cell in enumerate(row):
                confidence = self.attributes.values[y][x][CONFIDENCE]
                if cell == -1 or confidence < self.confidence_threshold:
                    return True
        return False

    def _frontier_candidates(self) -> list[tuple[int, int]]:
        candidates = []
        height = len(self.grid)
        width = len(self.grid[0])
        for y, row in enumerate(self.grid):
            for x, cell in enumerate(row):
                if not 0 <= cell < 50 or (x, y) == self.position:
                    continue
                confidence = self.attributes.values[y][x][CONFIDENCE]
                neighbours = ((x + 1, y), (x - 1, y), (x, y + 1), (x, y - 1))
                next_to_uncertain = False
                for nx, ny in neighbours:
                    if 0 <= nx < width and 0 <= ny < height and (
                        self.grid[ny][nx] == -1
                        or self.attributes.values[ny][nx][CONFIDENCE]
                        < self.confidence_threshold
                    ):
                        next_to_uncertain = True
                        break
                if confidence < self.confidence_threshold or next_to_uncertain:
                    candidates.append((x, y))
        return candidates

    def _distance_from_position(self, point: tuple[int, int]) -> int:
        return abs(point[0] - self.position[0]) + abs(point[1] - self.position[1])

    def _check_ready(self) -> None:
        if self.grid is None or self.attributes is None or self.position is None:
            raise ValueError("map and current position must be updated first")
