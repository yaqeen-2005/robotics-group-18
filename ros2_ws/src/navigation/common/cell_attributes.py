"""Store, load, and update independent per-cell map attributes.

CellAttributeMap keeps four floating-point values for every map cell: map
confidence, historical blockage probability, human encounter probability, and
other-robot encounter probability. All values are in [0.0, 1.0].
"""

from pathlib import Path

ATTRIBUTE_NAMES = (
    "confidence",
    "blockage_probability",
    "human_probability",
    "robot_encounter_probability",
)
CONFIDENCE = 0
BLOCKAGE_PROBABILITY = 1
HUMAN_PROBABILITY = 2
ROBOT_ENCOUNTER_PROBABILITY = 3


class CellAttributeMap:
    def __init__(self, width: int, height: int, values: list[list[list[float]]]):
        self.width = width
        self.height = height
        self.values = values

    def update_cell(self, x: int, y: int, dimension: int, value: float) -> None:
        """Replace one attribute value at (x, y)."""
        if not (0 <= x < self.width and 0 <= y < self.height):
            raise IndexError("cell coordinates are outside the map")
        if not 0 <= dimension < len(ATTRIBUTE_NAMES):
            raise IndexError("attribute dimension must be 0, 1, 2, or 3")
        if not 0.0 <= value <= 1.0:
            raise ValueError("attribute value must be between 0.0 and 1.0")
        self.values[y][x][dimension] = value

    def layer(self, dimension: int) -> list[list[float]]:
        """Return one attribute dimension as a two-dimensional grid."""
        if not 0 <= dimension < len(ATTRIBUTE_NAMES):
            raise IndexError("attribute dimension must be 0, 1, 2, or 3")
        return [[cell[dimension] for cell in row] for row in self.values]


def load_cell_attributes(
    path: str | Path, expected_width: int, expected_height: int
) -> CellAttributeMap:
    lines = [
        line.strip() for line in Path(path).read_text().splitlines() if line.strip()
    ]
    try:
        kind, version, width_text, height_text, dimensions_text = lines[0].split()
        width = int(width_text)
        height = int(height_text)
        dimensions = int(dimensions_text)
    except (IndexError, ValueError) as error:
        raise ValueError(
            "attribute header must be: ATTRIBUTES v2 <width> <height> 4"
        ) from error

    if kind != "ATTRIBUTES" or version != "v2" or dimensions != 4:
        raise ValueError("only ATTRIBUTES v2 with 4 dimensions is supported")
    if (width, height) != (expected_width, expected_height):
        raise ValueError("attribute dimensions do not match map dimensions")

    try:
        flat = [float(value) for line in lines[1:] for value in line.split()]
    except ValueError as error:
        raise ValueError("attributes must contain only numbers") from error
    expected_count = width * height * dimensions
    if len(flat) != expected_count:
        raise ValueError(f"attribute data must contain exactly {expected_count} values")
    if any(not 0.0 <= value <= 1.0 for value in flat):
        raise ValueError("attribute values must be between 0.0 and 1.0")

    cells = [
        flat[offset : offset + dimensions]
        for offset in range(0, len(flat), dimensions)
    ]
    values = [
        cells[offset : offset + width] for offset in range(0, len(cells), width)
    ]
    return CellAttributeMap(width, height, values)
