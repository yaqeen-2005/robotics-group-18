"""Read human-editable text maps for local A* testing."""

from pathlib import Path

from a_star import GridPoint

SYMBOLS = {".": 0, "0": 0, "#": 100, "1": 100, "?": -1, "S": 0, "G": 0}


def load_text_map(path: str | Path) -> tuple[list[list[int]], GridPoint, GridPoint]:
    """Load a versioned map using '.', '#', '?', 'S', and 'G' characters."""
    lines = [line.strip() for line in Path(path).read_text().splitlines() if line.strip()]
    try:
        kind, version, width_text, height_text = lines[0].split()
        width, height = int(width_text), int(height_text)
    except (IndexError, ValueError) as error:
        raise ValueError("map header must be: MAP v1 <width> <height>") from error
    if kind != "MAP" or version != "v1":
        raise ValueError("only MAP v1 is supported")

    symbols = "".join(line.replace(" ", "") for line in lines[1:])
    if len(symbols) != width * height:
        raise ValueError(f"map data must contain exactly {width * height} cells")
    lines = [symbols[offset : offset + width] for offset in range(0, len(symbols), width)]

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
