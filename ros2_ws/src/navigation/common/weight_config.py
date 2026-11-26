"""Load editable A* weight profiles without coupling them to the planner."""

import json
from pathlib import Path

from astar.a_star import DEFAULT_WEIGHTS


def load_weight_config(path):
    data = json.loads(Path(path).read_text())
    weights = data.get("weights", data)
    unknown = set(weights) - set(DEFAULT_WEIGHTS)
    if unknown:
        raise ValueError(
            f"unknown weight names: {sorted(unknown)}"
        )
    if any(not isinstance(value, (int, float)) or value < 0 for value in weights.values()):
        raise ValueError("all A* weights must be non-negative numbers")
    return {name: float(weights.get(name, default)) for name, default in DEFAULT_WEIGHTS.items()}
