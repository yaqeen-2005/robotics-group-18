"""Read human-editable text maps for local A* testing."""

from pathlib import Path

from a_star import GridPoint

SYMBOLS = {".": 0, "0": 0, "#": 100, "1": 100, "?": -1, "S": 0, "G": 0}


def load_text_map(path: str | Path) -> tuple[list[list[int]], GridPoint, GridPoint]:
    """Load a map using '.', '#', '?', 'S', and 'G' characters."""
    lines = [line.strip().replace(" ", "") for line in Path(path).read_text().splitlines() if line.strip()]
    if not lines or any(len(line) != len(lines[0]) for line in lines):
        raise ValueError("map must be a non-empty rectangle")

    grid: list[list[int]] = []
    start = goal = None
    for y, line in enumerate(lines):
        row = []
        for x, symbol in enumerate(line):
            if symbol not in SYMBOLS:
                raise ValueError(f"unsupported map symbol {symbol!r} at ({x}, {y})")
            if symbol == "S":
                if start is not None:
                    raise ValueError("map must contain exactly one S")
                start = (x, y)
            elif symbol == "G":
                if goal is not None:
                    raise ValueError("map must contain exactly one G")
                goal = (x, y)
            row.append(SYMBOLS[symbol])
        grid.append(row)

    if start is None or goal is None:
        raise ValueError("map must contain exactly one S and one G")
    return grid, start, goal
