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

WIDTH = 24
HEIGHT = 16
START = (2, 8)
GOAL = (21, 8)
HUMAN_ROUTE = (
    [(17, 5), (16, 5), (15, 5), (14, 5), (13, 5), (12, 5)]
    + [(12, 5)] * 8
    + [(13, 5), (14, 5), (15, 5), (16, 5)]
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
        if y not in (5, 12):
            grid[y][12] = 100
    for left, right in ((4, 9), (15, 20)):
        for top in (2, 6, 10):
            for y in range(top, top + 2):
                for x in range(left, right + 1):
                    grid[y][x] = 100
    return grid


def make_attributes():
    values = [
        [[1.0, 0.05, 0.0, 0.0] for _ in range(WIDTH)]
        for _ in range(HEIGHT)
    ]
    return CellAttributeMap(WIDTH, HEIGHT, values)


def run_demo(max_ticks=45):
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


def show_demo(demo, save_gif=None):
    """Animate the real demo trace with Matplotlib."""
    import matplotlib.pyplot as plt
    from matplotlib.animation import FuncAnimation, PillowWriter
    from matplotlib.colors import ListedColormap

    grid = demo["grid"]
    frames = demo["frames"]
    display = [[1 if cell >= 50 or cell < 0 else 0 for cell in row] for row in grid]
    figure, axis = plt.subplots(figsize=(13, 8))
    axis.imshow(display, cmap=ListedColormap(["white", "black"]), vmin=0, vmax=1)
    axis.set_xticks(range(0, WIDTH, 2))
    axis.set_yticks(range(0, HEIGHT, 2))
    axis.set_xticks([value - 0.5 for value in range(WIDTH + 1)], minor=True)
    axis.set_yticks([value - 0.5 for value in range(HEIGHT + 1)], minor=True)
    axis.grid(which="minor", color="lightgray", linewidth=0.5)
    axis.tick_params(which="minor", bottom=False, left=False)
    axis.set_xlabel("Grid x")
    axis.set_ylabel("Grid y")

    start = demo["start"]
    goal = demo["goal"]
    axis.scatter(*start, s=380, marker="*", color="orange", edgecolor="black", zorder=6)
    axis.scatter(*goal, s=380, marker="*", color="limegreen", edgecolor="black", zorder=6)
    axis.text(start[0], start[1] + 1.0, "A PICKUP", ha="center", weight="bold")
    axis.text(goal[0], goal[1] + 1.0, "B DELIVERY", ha="center", weight="bold")

    route_line, = axis.plot([], [], color="dodgerblue", linewidth=4, marker=".", zorder=3)
    robot_marker = axis.scatter([], [], s=330, marker="s", color="royalblue", edgecolor="yellow", linewidth=2.5, zorder=7, label="Robot")
    human_marker = axis.scatter([], [], s=280, marker="o", color="red", edgecolor="white", linewidth=2, zorder=7, label="Person")
    robot_label = axis.text(0, 0, "ROBOT", ha="center", weight="bold", zorder=8)
    human_label = axis.text(0, 0, "PERSON", ha="center", weight="bold", zorder=8)
    axis.legend(loc="upper center", bbox_to_anchor=(0.5, 1.13), ncol=2)

    def update(index):
        frame = frames[index]
        path = frame["path"]
        route_line.set_data([point[0] for point in path], [point[1] for point in path])
        route_line.set_color("limegreen" if frame["reroute_count"] else "dodgerblue")
        robot = frame["robot"]
        human = frame["human"]
        robot_marker.set_offsets([robot])
        human_marker.set_offsets([human])
        robot_label.set_position((robot[0], robot[1] - 0.65))
        human_label.set_position((human[0], human[1] - 0.65))
        score = frame["reroute_score"]
        score_text = "inactive" if score is None else f"{score:.2f} / 0.60"
        axis.set_title(
            f"Step {index + 1}/{len(frames)} | {frame['action']} | "
            f"reroute score: {score_text} | reroutes: {frame['reroute_count']}",
            color="darkred" if frame["action"] == "WAIT" else "black",
            weight="bold",
        )
        return route_line, robot_marker, human_marker, robot_label, human_label

    animation = FuncAnimation(
        figure,
        update,
        frames=len(frames),
        interval=550,
        repeat=False,
        blit=False,
    )
    update(0)
    figure.tight_layout()
    if save_gif:
        animation.save(save_gif, writer=PillowWriter(fps=2))
    else:
        plt.show()
    return animation


def main():
    parser = argparse.ArgumentParser(description="Run the dynamic navigation demo")
    parser.add_argument("--output", help="optional JSON trace output")
    parser.add_argument("--ticks", type=int, default=45)
    parser.add_argument("--show", action="store_true", help="open a Matplotlib animation")
    parser.add_argument("--save-gif", help="save the Matplotlib animation as a GIF")
    args = parser.parse_args()
    demo = run_demo(args.ticks)
    text = json.dumps(demo, indent=2)
    if args.output:
        output = Path(args.output)
        output.parent.mkdir(parents=True, exist_ok=True)
        output.write_text(text + "\n")
    if args.show or args.save_gif:
        show_demo(demo, args.save_gif)
    elif not args.output:
        print(text)


if __name__ == "__main__":
    main()
