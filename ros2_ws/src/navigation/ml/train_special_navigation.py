"""Fit door route and waiting thresholds on reproducible synthetic trials."""

import json
import random
from pathlib import Path

from astar.a_star import DEFAULT_WEIGHTS, plan_path
from common.cell_attributes import CellAttributeMap, TEMPORAL_DEFAULTS

OUTPUT = Path(__file__).parent / "weights" / "special_navigation.json"


def train(seed=18, trials=400, output=OUTPUT):
    rng = random.Random(seed)
    scenarios = [(rng.uniform(0.1, 0.9), rng.randint(1, 9)) for _ in range(trials)]
    grid = [[0] * 9 for _ in range(3)]
    # Direct route crosses (4,1); via the top/bottom row takes two extra steps.
    candidates = []
    for door_weight in (0, 2, 4, 6, 8, 12):
        error = 0.0
        for open_rate, delay in scenarios:
            cells = [[[1, 0, 0, 0] + TEMPORAL_DEFAULTS.copy() for _ in row] for row in grid]
            cells[1][4][6] = 1 - open_rate
            cells[1][4][9] = 1.0
            attributes = CellAttributeMap(9, 3, cells)
            path = plan_path(grid, (0, 1), (8, 1), attributes,
                             weights={"door": door_weight})
            predicted = (4, 1) in path
            direct_cost = 8 + (1 - open_rate) * delay
            detour_cost = 10
            error += (direct_cost if predicted else detour_cost) - min(direct_cost, detour_cost)
        candidates.append((error / trials, door_weight))
    regret, door_weight = min(candidates)
    repeat_cases = [(rng.randint(0, 3), rng.uniform(0.5, 3)) for _ in range(trials)]
    repeat_scores = []
    for repeat_weight in (0, 0.5, 1, 2, 3, 4):
        regret_total = 0.0
        for count, actual_cost in repeat_cases:
            path = plan_path(grid, (0, 1), (8, 1),
                             weights={"repeat": repeat_weight},
                             repeat_counts={(4, 1): count})
            direct_cost = 8 + count * actual_cost
            detour_cost = 10
            regret_total += (direct_cost if (4, 1) in path else detour_cost) - min(direct_cost, detour_cost)
        repeat_scores.append((regret_total / trials, repeat_weight))
    repeat_regret, repeat_weight = min(repeat_scores)
    wait_cases = [(rng.uniform(0.1, 0.9), rng.randint(1, 10), rng.randint(2, 6)) for _ in range(trials)]
    scores = []
    for threshold in (0.2, 0.3, 0.4, 0.5, 0.6, 0.7):
        for wait_limit in (2, 4, 6):
            regret_total = 0
            for open_rate, delay, detour in wait_cases:
                wait_cost = (1 - open_rate) * delay
                predicted = wait_cost if open_rate >= threshold and delay <= wait_limit else detour
                regret_total += predicted - min(wait_cost, detour)
            scores.append((regret_total / trials, threshold, wait_limit))
    wait_regret, threshold, wait_limit = min(scores)
    profile = {
        "weights": {**DEFAULT_WEIGHTS, "door": door_weight, "repeat": repeat_weight},
        "door_open_rate": threshold,
        "door_wait_limit": wait_limit,
        "training": {"seed": seed, "trials": trials, "route_regret": round(regret, 3),
                     "repeat_regret": round(repeat_regret, 3),
                     "decision_regret": round(wait_regret, 3), "synthetic": True},
    }
    output = Path(output)
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(profile, indent=2) + "\n")
    return profile


if __name__ == "__main__":
    print(json.dumps(train(), indent=2))
