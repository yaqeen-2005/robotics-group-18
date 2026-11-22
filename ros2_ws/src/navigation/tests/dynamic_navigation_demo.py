"""Run a deterministic SLAM-to-navigation demo with a moving human."""

import argparse
import json
import tempfile
from pathlib import Path

from common.cell_attributes import CellAttributeMap
from common.event_attributes import apply_new_events
from common.event_store import EventStore
from common.human_activity_report import build_human_activity_report
from common.path_manager import PathManager

WIDTH = 18
HEIGHT = 12
START = (2, 5)
GOAL = (15, 5)
HUMAN_ROUTE = (
    [(11, 3), (10, 3), (9, 3), (8, 3)]
    + [(8, 3)] * 8
    + [(9, 3), (10, 3), (11, 3), (12, 3)]
)


def make_demo_map():
    grid = [[0 for _ in range(WIDTH)] for _ in range(HEIGHT)]
    for x in range(WIDTH):
        grid[0][x] = 100
        grid[HEIGHT - 1][x] = 100
    for y in range(HEIGHT):
        grid[y][0] = 100
        grid[y][WIDTH - 1] = 100
    for y in range(1, HEIGHT - 1):
        if y not in (3, 8):
            grid[y][8] = 100
    return grid


def make_attributes():
    values = [
        [[1.0, 0.05, 0.0, 0.0] for _ in range(WIDTH)]
        for _ in range(HEIGHT)
    ]
    return CellAttributeMap(WIDTH, HEIGHT, values)


def run_demo(max_ticks=30):
    grid = make_demo_map()
    attributes = make_attributes()
    manager = PathManager()
    robot = START
    waiting_time = 0.0
    frames = []

    with tempfile.TemporaryDirectory() as directory:
        store = EventStore(Path(directory) / "events.db")
        for tick in range(max_ticks):
            human = HUMAN_ROUTE[min(tick, len(HUMAN_ROUTE) - 1)]
            remaining = manager._remaining_path(robot)
            human_blocking = human in remaining
            distance = abs(robot[0] - human[0]) + abs(robot[1] - human[1])
            event_id = None
            human_flag = False
            if human_blocking:
                event_id = store.record_human_detection(
                    "demo-robot",
                    "demo-run",
                    "demo-map",
                    human,
                    distance=distance,
                )
                update = apply_new_events(store, attributes, "demo-map")
                human_flag = update["human_detected"]
                waiting_time += 1.0
            else:
                waiting_time = 0.0

            result = manager.plan(
                grid,
                robot,
                GOAL,
                attributes,
                human_blocking=human_blocking,
                human_distance=distance,
                waiting_time=waiting_time,
            )
            if event_id is not None:
                store.annotate_event(
                    event_id,
                    decision=result["action"],
                    reroute_score=result["reroute_score"],
                )

            path = result["path"]
            if result["action"] in {"MOVE", "REROUTE"} and len(path) > 1:
                robot = tuple(path[1])
            frames.append(
                {
                    "tick": tick,
                    "robot": robot,
                    "human": human,
                    "path": path,
                    "action": result["action"],
                    "human_detected": human_flag,
                    "reroute_score": result["reroute_score"],
                    "reroute_count": result["reroute_count"],
                }
            )
            if robot == GOAL:
                break

        report = build_human_activity_report(store.read_events(), "demo-map")
    return {"grid": grid, "start": START, "goal": GOAL, "frames": frames, "report": report}


def main():
    parser = argparse.ArgumentParser(description="Run the dynamic navigation demo")
    parser.add_argument("--output", help="optional JSON trace output")
    parser.add_argument("--ticks", type=int, default=30)
    args = parser.parse_args()
    demo = run_demo(args.ticks)
    text = json.dumps(demo, indent=2)
    if args.output:
        output = Path(args.output)
        output.parent.mkdir(parents=True, exist_ok=True)
        output.write_text(text + "\n")
    else:
        print(text)


if __name__ == "__main__":
    main()
