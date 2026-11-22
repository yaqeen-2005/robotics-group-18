"""Run deterministic random A-to-B tasks with moving people."""

import argparse
import json
import random
import tempfile
from pathlib import Path

from astar.a_star import plan_path
from common.cell_attributes import CellAttributeMap
from common.event_attributes import apply_new_events
from common.event_store import EventStore
from common.human_activity_report import build_human_activity_report
from common.path_manager import PathManager

WIDTH = 24
HEIGHT = 16
LEFT_STATIONS = [(2, 4), (2, 8), (2, 12)]
RIGHT_STATIONS = [(21, 4), (21, 8), (21, 12)]
DEFAULT_SEED = 18
DEFAULT_TASKS = 3
MAX_TICKS_PER_TASK = 80
DEFAULT_HUMAN_ATTENTION_DISTANCE = 6


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


def make_tasks(seed=DEFAULT_SEED, count=DEFAULT_TASKS):
    if not 1 <= count <= len(LEFT_STATIONS):
        raise ValueError(f"task count must be between 1 and {len(LEFT_STATIONS)}")
    rng = random.Random(seed)
    left = rng.sample(LEFT_STATIONS, count)
    right = rng.sample(RIGHT_STATIONS, count)
    tasks = []
    for index in range(count):
        start, goal = (left[index], right[index])
        if index % 2:
            start, goal = goal, start
        tasks.append({"task_id": index + 1, "start": start, "goal": goal})
    return tasks


def blocking_route(path):
    crossing_index = next(
        index for index, point in enumerate(path) if point[0] == 12
    )
    crossing = path[crossing_index]
    approach = list(reversed(path[crossing_index + 1 : crossing_index + 5]))
    if not approach:
        approach = list(path[max(0, crossing_index - 4) : crossing_index])
    return approach + [crossing] * 9


def background_people(tick):
    left_y = 2 + tick % 12
    right_y = 13 - tick % 12
    return [(2, left_y), (21, right_y)]


def keep_people_out_of_robot(planned_people, previous_people, robot, grid):
    """Make simulated pedestrians yield instead of entering the robot cell."""
    safe_people = []
    for index, planned in enumerate(planned_people):
        if planned != robot:
            safe_people.append(planned)
            continue
        previous = previous_people[index] if previous_people else planned
        if previous != robot:
            safe_people.append(previous)
            continue
        x, y = planned
        alternatives = ((x + 1, y), (x - 1, y), (x, y + 1), (x, y - 1))
        safe_people.append(
            next(
                point
                for point in alternatives
                if 0 <= point[0] < WIDTH
                and 0 <= point[1] < HEIGHT
                and grid[point[1]][point[0]] == 0
                and point != robot
            )
        )
    return safe_people


def run_demo(
    seed=DEFAULT_SEED,
    task_count=DEFAULT_TASKS,
    attention_distance=DEFAULT_HUMAN_ATTENTION_DISTANCE,
):
    grid = make_demo_map()
    attributes = make_attributes()
    tasks = make_tasks(seed, task_count)
    frames = []
    global_tick = 0

    with tempfile.TemporaryDirectory() as directory:
        store = EventStore(Path(directory) / "events.db")
        for task in tasks:
            manager = PathManager()
            robot = task["start"]
            goal = task["goal"]
            initial_path = plan_path(grid, robot, goal, attributes)
            person_route = blocking_route(initial_path)
            waiting_time = 0.0
            previous_people = []
            manager.plan(grid, robot, goal, attributes)

            for task_tick in range(MAX_TICKS_PER_TASK):
                blocker = person_route[min(task_tick, len(person_route) - 1)]
                planned_people = [blocker, *background_people(global_tick)]
                people = keep_people_out_of_robot(
                    planned_people, previous_people, robot, grid
                )
                previous_people = people
                remaining = manager._remaining_path(robot)
                people_on_path = []
                for index, person in enumerate(people):
                    if person in remaining[1:]:
                        distance = abs(robot[0] - person[0]) + abs(robot[1] - person[1])
                        people_on_path.append((distance, index, person))
                nearest = min(people_on_path, default=None)
                human_blocking = (
                    nearest is not None and nearest[0] <= attention_distance
                )
                distance = nearest[0] if nearest is not None else attention_distance + 1
                nearest_index = nearest[1] if nearest is not None else None
                nearest_position = nearest[2] if nearest is not None else None
                event_id = None
                human_flag = False
                if human_blocking:
                    event_id = store.record_human_detection(
                        "demo-robot",
                        "demo-run",
                        "demo-map",
                        nearest_position,
                        distance=distance,
                        task_id=f"task-{task['task_id']}",
                    )
                    update = apply_new_events(store, attributes, "demo-map")
                    human_flag = update["human_detected"]
                    waiting_time += 1.0
                else:
                    waiting_time = 0.0

                result = manager.plan(
                    grid,
                    robot,
                    goal,
                    attributes,
                    human_blocking=human_blocking,
                    human_distance=distance,
                    waiting_time=waiting_time,
                    human_cells={nearest_position} if human_blocking else None,
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
                        "tick": global_tick,
                        "task_tick": task_tick,
                        "task_id": task["task_id"],
                        "task_count": len(tasks),
                        "start": task["start"],
                        "goal": goal,
                        "robot": robot,
                        "humans": people,
                        "nearest_human": nearest_index,
                        "human_distance": distance if nearest is not None else None,
                        "attention_distance": attention_distance,
                        "blocking_human": nearest_index if human_blocking else None,
                        "path": path,
                        "action": result["action"],
                        "human_detected": human_flag,
                        "reroute_score": result["reroute_score"],
                        "reroute_count": result["reroute_count"],
                    }
                )
                global_tick += 1
                if robot == goal:
                    break

        report = build_human_activity_report(store.read_events(), "demo-map")
    return {"grid": grid, "tasks": tasks, "frames": frames, "report": report}


def show_demo(demo, save_gif=None):
    """Animate real A* decisions with obvious waiting feedback."""
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

    route_line, = axis.plot([], [], color="dodgerblue", linewidth=4, marker=".", zorder=3)
    robot_marker = axis.scatter([], [], s=360, marker="s", color="royalblue", edgecolor="yellow", linewidth=2.5, zorder=7, label="Robot")
    human_markers = axis.scatter([], [], s=230, marker="o", color="red", edgecolor="white", linewidth=2, zorder=7, label="People")
    start_marker = axis.scatter([], [], s=380, marker="*", color="orange", edgecolor="black", zorder=6)
    goal_marker = axis.scatter([], [], s=380, marker="*", color="limegreen", edgecolor="black", zorder=6)
    robot_label = axis.text(0, 0, "ROBOT", ha="center", weight="bold", zorder=8)
    start_label = axis.text(0, 0, "A PICKUP", ha="center", weight="bold", zorder=8)
    goal_label = axis.text(0, 0, "B DELIVERY", ha="center", weight="bold", zorder=8)
    wait_banner = axis.text(
        WIDTH / 2,
        1,
        "",
        ha="center",
        va="center",
        fontsize=18,
        weight="bold",
        color="black",
        zorder=10,
    )
    axis.legend(loc="upper center", bbox_to_anchor=(0.5, 1.13), ncol=2)

    def update(index):
        frame = frames[index]
        path = frame["path"]
        route_line.set_data([point[0] for point in path], [point[1] for point in path])
        route_line.set_color("limegreen" if frame["reroute_count"] else "dodgerblue")
        robot = frame["robot"]
        humans = frame["humans"]
        start = frame["start"]
        goal = frame["goal"]
        robot_marker.set_offsets([robot])
        human_markers.set_offsets(humans)
        human_markers.set_facecolors(
            [
                "darkred" if index == frame["blocking_human"] else "red"
                for index in range(len(humans))
            ]
        )
        human_markers.set_edgecolors(
            [
                "gold" if index == frame["nearest_human"] else "white"
                for index in range(len(humans))
            ]
        )
        start_marker.set_offsets([start])
        goal_marker.set_offsets([goal])
        robot_label.set_position((robot[0], robot[1] - 0.65))
        start_label.set_position((start[0], start[1] + 1.0))
        goal_label.set_position((goal[0], goal[1] + 1.0))

        waiting = frame["action"] == "WAIT"
        rerouting = frame["action"] == "REROUTE"
        robot_marker.set_facecolor("orange" if waiting else "royalblue")
        robot_marker.set_sizes([520 if waiting else 360])
        if waiting:
            wait_banner.set_text("WAITING FOR PERSON")
            wait_banner.set_bbox({"facecolor": "gold", "edgecolor": "darkred", "pad": 8})
        elif rerouting:
            wait_banner.set_text("REROUTING AROUND PERSON")
            wait_banner.set_bbox({"facecolor": "limegreen", "edgecolor": "black", "pad": 8})
        else:
            wait_banner.set_text("")
            wait_banner.set_bbox(None)

        score = frame["reroute_score"]
        score_text = "inactive" if score is None else f"{score:.2f} / 0.60"
        distance = frame["human_distance"]
        person_text = (
            "none on path"
            if distance is None
            else f"{distance} cells (react <= {frame['attention_distance']})"
        )
        axis.set_title(
            f"Task {frame['task_id']}/{frame['task_count']} | step {frame['task_tick'] + 1} | "
            f"{frame['action']}\nNearest path person: {person_text} | reroute score: {score_text}",
            color="darkred" if waiting else "black",
            weight="bold",
        )
        return route_line, robot_marker, human_markers, wait_banner

    animation = FuncAnimation(
        figure,
        update,
        frames=len(frames),
        interval=500,
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
    parser = argparse.ArgumentParser(description="Run random dynamic navigation tasks")
    parser.add_argument("--output", help="optional JSON trace output")
    parser.add_argument("--seed", type=int, default=DEFAULT_SEED)
    parser.add_argument("--tasks", type=int, default=DEFAULT_TASKS)
    parser.add_argument(
        "--attention-distance",
        type=int,
        default=DEFAULT_HUMAN_ATTENTION_DISTANCE,
        help="only react to the nearest person on the path within this many cells",
    )
    parser.add_argument("--show", action="store_true", help="open a Matplotlib animation")
    parser.add_argument("--save-gif", help="save the Matplotlib animation as a GIF")
    args = parser.parse_args()
    demo = run_demo(args.seed, args.tasks, args.attention_distance)
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
