"""Fit door route and waiting thresholds on reproducible synthetic trials."""

import json
import random
from pathlib import Path

from astar.a_star import DEFAULT_WEIGHTS, plan_path
from common.cell_attributes import CellAttributeMap, TEMPORAL_DEFAULTS

OUTPUT = Path(__file__).parent / "weights" / "special_navigation.json"


def train(seed=18, trials=400, output=OUTPUT):
    rng = random.Random(seed)
    scenarios = []
    for _ in range(trials):
        grid = [[0] * 25 for _ in range(11)]
        for x in range(3, 22):
            for base in (2, 8):
                y = base + (x // 4) % 2 * rng.choice((-1, 0, 1))
                if 0 < y < 10 and rng.random() < 0.75:
                    grid[y][x] = 100
        # An irregular gap around the centre door changes the detour length.
        for y in (4, 6):
            if rng.random() < 0.5:
                grid[y][12] = 100
        door = (12, 5)
        start, goal = (2, 5), (22, 5)
        direct = len(plan_path(grid, start, goal)) - 1
        detour = len(plan_path(grid, start, goal, blocked_cells={door})) - 1
        scenarios.append((grid, rng.uniform(0.1, 0.9), rng.randint(1, 9), direct, detour))
    candidates = []
    for door_weight in (0, 2, 4, 6, 8, 12):
        error = 0.0
        for grid, open_rate, delay, direct, detour in scenarios:
            cells = [[[1, 0, 0, 0] + TEMPORAL_DEFAULTS.copy() for _ in row] for row in grid]
            cells[5][12][6] = 1 - open_rate
            cells[5][12][9] = 1.0
            attributes = CellAttributeMap(25, 11, cells)
            path = plan_path(grid, (2, 5), (22, 5), attributes,
                             weights={"door": door_weight})
            direct_cost = direct + (1 - open_rate) * delay
            error += (direct_cost if door in path else detour) - min(direct_cost, detour)
        candidates.append((error / trials, door_weight))
    regret, door_weight = min(candidates)
    repeat_cases = [(scenario[0], scenario[3], scenario[4], rng.randint(0, 3), rng.uniform(0.5, 3)) for scenario in scenarios]
    repeat_scores = []
    for repeat_weight in (0, 0.5, 1, 2, 3, 4):
        regret_total = 0.0
        for grid, direct, detour, count, actual_cost in repeat_cases:
            path = plan_path(grid, (2, 5), (22, 5),
                             weights={"repeat": repeat_weight},
                             repeat_counts={(12, 5): count})
            direct_cost = direct + count * actual_cost
            regret_total += (direct_cost if (12, 5) in path else detour) - min(direct_cost, detour)
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
    backtrack_cases = [(rng.uniform(0.1, 0.9), rng.randint(1, 10),
                        rng.randint(2, 6), rng.randint(0, 3)) for _ in range(trials)]
    backtrack_scores = []
    for backtrack_weight in (0, 0.025, 0.05, 0.075, 0.1):
        regret_total = 0.0
        for open_rate, delay, detour, backtracks in backtrack_cases:
            wait_cost = (1 - open_rate) * delay
            detour_cost = detour + backtracks
            chosen_wait = (open_rate >= max(0.1, threshold - backtrack_weight * backtracks)
                           and delay <= wait_limit + backtracks)
            regret_total += (wait_cost if chosen_wait else detour_cost) - min(wait_cost, detour_cost)
        backtrack_scores.append((regret_total / trials, backtrack_weight))
    backtrack_regret, backtrack_weight = min(backtrack_scores)
    profile = {
        "weights": {**DEFAULT_WEIGHTS, "door": door_weight, "repeat": repeat_weight},
        "door_open_rate": threshold,
        "door_wait_limit": wait_limit,
        "backtrack_wait_weight": backtrack_weight,
        "training": {"seed": seed, "trials": trials, "route_regret": round(regret, 3),
                     "repeat_regret": round(repeat_regret, 3),
                     "decision_regret": round(wait_regret, 3),
                     "backtrack_regret": round(backtrack_regret, 3), "synthetic": True},
    }
    output = Path(output)
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(profile, indent=2) + "\n")
    return profile


if __name__ == "__main__":
    print(json.dumps(train(), indent=2))
