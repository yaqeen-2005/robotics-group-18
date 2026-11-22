"""Build a reproducible compressed training set for one busy factory map."""

import argparse
import json
import random
import tempfile
import zipfile
from pathlib import Path

from common.map_io import load_text_map
from ml.generate_maps import generate_template, shortest_path

DEFAULT_ARCHIVE = Path(__file__).parent / "training_data" / "factory_busy_v1.zip"
DEFAULT_TASK_COUNT = 5000
DEFAULT_SEED = 18


def prepare_training_archive(output=DEFAULT_ARCHIVE, task_count=DEFAULT_TASK_COUNT, seed=DEFAULT_SEED):
    output = Path(output)
    output.parent.mkdir(parents=True, exist_ok=True)
    rng = random.Random(seed)
    with tempfile.TemporaryDirectory() as directory:
        root = generate_template("training_factory_busy", directory, seed)
        grid, _, _ = load_text_map(root / "map.txt")
        free_cells = [
            (x, y)
            for y, row in enumerate(grid)
            for x, value in enumerate(row)
            if value == 0
        ]
        tasks_path = root / "tasks.jsonl"
        written = 0
        with tasks_path.open("w") as stream:
            while written < task_count:
                start, goal = rng.sample(free_cells, 2)
                baseline = shortest_path(grid, start, goal)
                if len(baseline) < 10:
                    continue
                stream.write(
                    json.dumps(
                        {
                            "task_id": written,
                            "start": start,
                            "goal": goal,
                            "shortest_steps": len(baseline) - 1,
                        },
                        separators=(",", ":"),
                    )
                    + "\n"
                )
                written += 1
        (root / "TRAINING_DATA.md").write_text(
            "# Factory busy training data\n\n"
            "Generated from the `training_factory_busy` 40 x 30 aisle template.\n"
            "`tasks.jsonl` contains start/goal grid coordinates and an unweighted "
            "shortest-path length for each task. Risk labels are the four values in "
            "`attributes.txt`.\n"
        )
        with zipfile.ZipFile(output, "w", zipfile.ZIP_DEFLATED) as archive:
            for path in root.rglob("*"):
                if path.is_file():
                    archive.write(path, Path(root.name) / path.relative_to(root))
    return output


def main():
    parser = argparse.ArgumentParser(description="Create compressed A* training data")
    parser.add_argument("--output", default=DEFAULT_ARCHIVE)
    parser.add_argument("--tasks", type=int, default=DEFAULT_TASK_COUNT)
    parser.add_argument("--seed", type=int, default=DEFAULT_SEED)
    args = parser.parse_args()
    if args.tasks < 1:
        raise ValueError("tasks must be at least 1")
    print(prepare_training_archive(args.output, args.tasks, args.seed))


if __name__ == "__main__":
    main()
