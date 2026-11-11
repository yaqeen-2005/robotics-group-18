"""Read and validate per-cell confidence data without interpreting it."""

from pathlib import Path


def load_confidence(
    path: str | Path, expected_width: int, expected_height: int
) -> list[list[float]]:
    """Load a CONFIDENCE v1 grid and verify that it matches the map size."""
    lines = [line.strip() for line in Path(path).read_text().splitlines() if line.strip()]
    try:
        kind, version, width_text, height_text = lines[0].split()
        width, height = int(width_text), int(height_text)
    except (IndexError, ValueError) as error:
        raise ValueError(
            "confidence header must be: CONFIDENCE v1 <width> <height>"
        ) from error

    if kind != "CONFIDENCE" or version != "v1":
        raise ValueError("only CONFIDENCE v1 is supported")
    if (width, height) != (expected_width, expected_height):
        raise ValueError("confidence dimensions do not match map dimensions")

    try:
        flat_values = [float(value) for line in lines[1:] for value in line.split()]
    except ValueError as error:
        raise ValueError("confidence data must contain only numbers") from error
    if len(flat_values) != width * height:
        raise ValueError(
            f"confidence data must contain exactly {width * height} values"
        )
    values = [
        flat_values[offset : offset + width]
        for offset in range(0, len(flat_values), width)
    ]
    if any(not 0.0 <= value <= 1.0 for row in values for value in row):
        raise ValueError("confidence values must be between 0.0 and 1.0")
    return values
