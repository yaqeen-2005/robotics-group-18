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
    "observation_count",
    "block_count",
    "block_rate",
    "switch_count",
    "last_state",
    "door_belief",
)
CONFIDENCE = 0
BLOCKAGE_PROBABILITY = 1
HUMAN_PROBABILITY = 2
ROBOT_ENCOUNTER_PROBABILITY = 3
OBSERVATION_COUNT = 4
BLOCK_COUNT = 5
BLOCK_RATE = 6
SWITCH_COUNT = 7
LAST_STATE = 8  # -1 unknown, 0 free, 1 blocked
DOOR_BELIEF = 9
TEMPORAL_DEFAULTS = [0, 0, 0.0, 0, -1, 0.0]


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
            raise IndexError("attribute dimension is outside the feature map")
        _validate_feature(dimension, value)
        if len(self.values[y][x]) == 4:
            self.values[y][x].extend(TEMPORAL_DEFAULTS)
        self.values[y][x][dimension] = value

    def layer(self, dimension: int) -> list[list[float]]:
        """Return one attribute dimension as a two-dimensional grid."""
        if not 0 <= dimension < len(ATTRIBUTE_NAMES):
            raise IndexError("attribute dimension is outside the feature map")
        return [[(cell + TEMPORAL_DEFAULTS)[dimension] for cell in row] for row in self.values]


def _validate_feature(dimension, value):
    if dimension in (OBSERVATION_COUNT, BLOCK_COUNT, SWITCH_COUNT):
        valid = isinstance(value, (int, float)) and value >= 0 and value == int(value)
    elif dimension == LAST_STATE:
        valid = value in (-1, 0, 1)
    else:
        valid = isinstance(value, (int, float)) and 0 <= value <= 1
    if not valid:
        raise ValueError(f"invalid {ATTRIBUTE_NAMES[dimension]} value: {value}")


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

    if kind != "ATTRIBUTES" or (version, dimensions) not in (("v2", 4), ("v3", 10)):
        raise ValueError("expected ATTRIBUTES v2 with 4 or v3 with 10 dimensions")
    if (width, height) != (expected_width, expected_height):
        raise ValueError("attribute dimensions do not match map dimensions")

    try:
        flat = [float(value) for line in lines[1:] for value in line.split()]
    except ValueError as error:
        raise ValueError("attributes must contain only numbers") from error
    expected_count = width * height * dimensions
    if len(flat) != expected_count:
        raise ValueError(f"attribute data must contain exactly {expected_count} values")
    for index, value in enumerate(flat):
        _validate_feature(index % dimensions, value)

    cells = [
        flat[offset : offset + dimensions]
        for offset in range(0, len(flat), dimensions)
    ]
    values = [
        cells[offset : offset + width] for offset in range(0, len(cells), width)
    ]
    return CellAttributeMap(width, height, values)


def save_cell_attributes(path: str | Path, attributes: CellAttributeMap) -> None:
    """Save attributes using the existing flattened ATTRIBUTES v2 format."""
    flat = []
    dimensions = 10 if any(len(cell) == 10 for row in attributes.values for cell in row) else 4
    if len(attributes.values) != attributes.height:
        raise ValueError("attribute height does not match its values")
    for row in attributes.values:
        if len(row) != attributes.width:
            raise ValueError("attribute width does not match its values")
        for cell in row:
            if len(cell) not in (4, len(ATTRIBUTE_NAMES)):
                raise ValueError("every cell must contain four or ten attributes")
            for dimension, value in enumerate(cell):
                _validate_feature(dimension, value)
            flat.extend(f"{value:g}" for value in (cell + TEMPORAL_DEFAULTS if dimensions == 10 and len(cell) == 4 else cell))
    header = f"ATTRIBUTES {'v2' if dimensions == 4 else 'v3'} {attributes.width} {attributes.height} {dimensions}"
    Path(path).write_text(f"{header}\n{' '.join(flat)}\n")
