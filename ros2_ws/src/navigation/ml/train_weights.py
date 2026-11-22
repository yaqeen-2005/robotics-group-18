"""Train editable A* weight profiles with a small standard-library search."""

import argparse
import json
import random
import tempfile
import zipfile
from pathlib import Path

from astar.a_star import DEFAULT_WEIGHTS, plan_path
from common.cell_attributes import (
    BLOCKAGE_PROBABILITY,
    CONFIDENCE,
    HUMAN_PROBABILITY,
    ROBOT_ENCOUNTER_PROBABILITY,
    load_cell_attributes,
)
from common.map_io import load_text_map
from ml.prepare_training_data import DEFAULT_ARCHIVE

DEFAULT_OUTPUT = Path(__file__).parent / "weights"
DEFAULT_SEED = 18
DEFAULT_TRIALS = 40
DEFAULT_TASK_LIMIT = 120

OBJECTIVES = {
    "balanced": {
        "distance": 1.0,
        "blockage": 4.0,
        "human": 6.0,
        "uncertainty": 3.0,
        "robot_encounter": 4.0,
        "turn": 0.5,
    },
    "human_safe": {
        "distance": 1.0,
        "blockage": 4.0,
        "human": 14.0,
        "uncertainty": 3.0,
        "robot_encounter": 4.0,
        "turn": 0.75,
    },
}


def train(archive_path=DEFAULT_ARCHIVE, output=DEFAULT_OUTPUT, trials=DEFAULT_TRIALS, task_limit=DEFAULT_TASK_LIMIT, seed=DEFAULT_SEED):
    output = Path(output)
    output.mkdir(parents=True, exist_ok=True)
    rng = random.Random(seed)
    with tempfile.TemporaryDirectory() as directory:
        with zipfile.ZipFile(archive_path) as archive:
            archive.extractall(directory)
        roots = list(Path(directory).glob("*/map.txt"))
        if len(roots) != 1:
            raise ValueError("training archive must contain exactly one map")
        root = roots[0].parent
        grid, _, _ = load_text_map(root / "map.txt")
        attributes = load_cell_attributes(
            root / "attributes.txt", len(grid[0]), len(grid)
        )
        tasks = [
            json.loads(line)
            for line in (root / "tasks.jsonl").read_text().splitlines()[:task_limit]
        ]

        candidates = [DEFAULT_WEIGHTS.copy()]
        for _ in range(trials - 1):
            candidates.append(
                {
                    "distance": rng.uniform(0.5, 2.0),
                    "blockage": rng.uniform(0.0, 10.0),
                    "human": rng.uniform(0.0, 18.0),
                    "uncertainty": rng.uniform(0.0, 8.0),
                    "robot_encounter": rng.uniform(0.0, 10.0),
                    "turn": rng.uniform(0.0, 3.0),
                    "heuristic": 1.0,
                }
            )

        report = {}
        for name, objective in OBJECTIVES.items():
            scored = [
                (_evaluate(grid, attributes, tasks, weights, objective), weights)
                for weights in candidates
            ]
            score, best = min(scored, key=lambda item: item[0])
            profile = {
                "name": name,
                "weights": {key: round(value, 4) for key, value in best.items()},
                "training": {
                    "archive": Path(archive_path).name,
                    "seed": seed,
                    "trials": trials,
                    "tasks": len(tasks),
                    "objective": objective,
                    "score": round(score, 4),
                },
            }
            (output / f"{name}.json").write_text(json.dumps(profile, indent=2) + "\n")
            report[name] = profile["training"]
        (output / "training_report.json").write_text(json.dumps(report, indent=2) + "\n")
    return report


def _evaluate(grid, attributes, tasks, weights, objective):
    total = 0.0
    for task in tasks:
        path = plan_path(
            grid,
            tuple(task["start"]),
            tuple(task["goal"]),
            attributes,
            weights,
        )
        if not path:
            total += 100000.0
            continue
        metrics = _path_metrics(path, attributes)
        total += sum(metrics[name] * value for name, value in objective.items())
    return total / len(tasks)


def _path_metrics(path, attributes):
    metrics = {
        "distance": len(path) - 1,
        "blockage": 0.0,
        "human": 0.0,
        "uncertainty": 0.0,
        "robot_encounter": 0.0,
        "turn": 0,
    }
    previous_direction = None
    for index, (x, y) in enumerate(path):
        cell = attributes.values[y][x]
        metrics["blockage"] += cell[BLOCKAGE_PROBABILITY]
        metrics["human"] += cell[HUMAN_PROBABILITY]
        metrics["uncertainty"] += 1.0 - cell[CONFIDENCE]
        metrics["robot_encounter"] += cell[ROBOT_ENCOUNTER_PROBABILITY]
        if index == 0:
            continue
        direction = (x - path[index - 1][0], y - path[index - 1][1])
        if previous_direction is not None and direction != previous_direction:
            metrics["turn"] += 1
        previous_direction = direction
    return metrics


def main():
    parser = argparse.ArgumentParser(description="Train two A* weight profiles")
    parser.add_argument("archive", nargs="?", default=DEFAULT_ARCHIVE)
    parser.add_argument("--output", default=DEFAULT_OUTPUT)
    parser.add_argument("--trials", type=int, default=DEFAULT_TRIALS)
    parser.add_argument("--tasks", type=int, default=DEFAULT_TASK_LIMIT)
    parser.add_argument("--seed", type=int, default=DEFAULT_SEED)
    args = parser.parse_args()
    if args.trials < 1 or args.tasks < 1:
        raise ValueError("trials and tasks must be at least 1")
    train(args.archive, args.output, args.trials, args.tasks, args.seed)
    print(args.output)


if __name__ == "__main__":
    main()
