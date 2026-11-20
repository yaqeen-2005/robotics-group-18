"""Detect exploration frontiers in an occupancy grid.

Adapted from Xu's frontier algorithm in commit 398f480. The original
four-neighbour frontier rule is retained and extended to low-confidence cells.
"""

from common.cell_attributes import CONFIDENCE, CellAttributeMap


def find_frontiers(
    grid,
    attributes: CellAttributeMap | None = None,
    confidence_threshold: float = 0.5,
):
    """Return free cells bordering unknown or low-confidence map space."""
    if len(grid) == 0:
        return []
    width = len(grid[0])
    if any(len(row) != width for row in grid):
        raise ValueError("grid must be rectangular")
    if any(
        cell != -1 and not 0 <= cell <= 100 for row in grid for cell in row
    ):
        raise ValueError("grid cells must be -1 or between 0 and 100")
    if attributes is not None and (
        attributes.width != width or attributes.height != len(grid)
    ):
        raise ValueError("attribute dimensions do not match grid dimensions")
    if not 0.0 <= confidence_threshold <= 1.0:
        raise ValueError("confidence_threshold must be between 0.0 and 1.0")

    height = len(grid)
    frontiers = []
    for y, row in enumerate(grid):
        for x, cell in enumerate(row):
            if not 0 <= cell < 50:
                continue
            if (
                attributes is not None
                and attributes.values[y][x][CONFIDENCE] < confidence_threshold
            ):
                frontiers.append((x, y))
                continue
            for dx, dy in ((-1, 0), (1, 0), (0, -1), (0, 1)):
                nx, ny = x + dx, y + dy
                if not 0 <= nx < width or not 0 <= ny < height:
                    continue
                unknown = grid[ny][nx] == -1
                low_confidence = (
                    attributes is not None
                    and attributes.values[ny][nx][CONFIDENCE]
                    < confidence_threshold
                )
                if unknown or low_confidence:
                    frontiers.append((x, y))
                    break
    return frontiers
