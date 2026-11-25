"""Simulate warehouse observations while the navigation modules choose each route."""

import argparse
import json
from math import sqrt
from pathlib import Path

from common.cell_attributes import CellAttributeMap, DOOR_BELIEF
from common.live_navigator import LiveNavigator
from common.path_smoothing import _zigzag_shortcut
from common.temporal_traversability import TemporalTraversabilityLayer
from ml.door_belief import DoorPredictor

WIDTH, HEIGHT = 64, 30
START, GOAL = (4, 10), (43, 10)
DIAGONAL_GOAL = (59, 19)
CORNER_GOAL = (10, 16)
SENSOR_RANGE = 7
BACKGROUND_PEOPLE = ((9, 18), (29, 17), (40, 15))
DOOR_GROUPS = {
    (wall, passage): {(wall, y) for y in (range(8, 12) if passage == "upper" else range(21, 25))}
    for wall in (23, 35) for passage in ("upper", "lower")
}
DOOR = set().union(*DOOR_GROUPS.values())
PERSON = (26, 10)


def people_at(trip, tick):
    moving = (
        (9 + (tick // 5) % 3, 18),
        (29, 17 + (tick // 7) % 2),
        (40 + (tick // 6) % 2, 15),
    )
    return ((26 + (tick // 4) % 2, 10),) + moving if trip == 1 else moving


def in_range(robot, point):
    return (point[0] - robot[0]) ** 2 + (point[1] - robot[1]) ** 2 <= SENSOR_RANGE ** 2


def line_of_sight(grid, robot, point):
    distance = sqrt((point[0] - robot[0]) ** 2 + (point[1] - robot[1]) ** 2)
    if distance == 0:
        return True
    for index in range(1, int(distance * 8)):
        fraction = index / (distance * 8)
        x = int(robot[0] + (point[0] - robot[0]) * fraction + 0.5)
        y = int(robot[1] + (point[1] - robot[1]) * fraction + 0.5)
        if (x, y) != tuple(point) and grid[y][x] >= 50:
            return False
    return True


def visible_people(robot, people, grid=None):
    return {point for point in people
            if in_range(robot, point) and (grid is None or line_of_sight(grid, robot, point))}


def visible_map_cells(robot, grid):
    left = max(0, int(robot[0] - SENSOR_RANGE - 1))
    right = min(WIDTH, int(robot[0] + SENSOR_RANGE + 2))
    top = max(0, int(robot[1] - SENSOR_RANGE - 1))
    bottom = min(HEIGHT, int(robot[1] + SENSOR_RANGE + 2))
    return {(x, y) for y in range(top, bottom) for x in range(left, right)
            if in_range(robot, (x, y)) and line_of_sight(grid, robot, (x, y))}


def make_map():
    grid = [[0] * WIDTH for _ in range(HEIGHT)]
    for y in range(HEIGHT):
        for x in range(WIDTH):
            if x in (0, WIDTH - 1) or y in (0, HEIGHT - 1):
                grid[y][x] = 100
    for wall in (23, 35):
        for y in range(1, HEIGHT - 1):
            if y not in range(8, 12) and y not in range(21, 25):
                grid[y][wall] = 100
    # Slanted, uneven shelf ends; they do not close either passage.
    for base in (4, 32):
        for x in range(base, base + 10):
            top = 3 + (x - base) // 3
            bottom = 26 - (x - base) // 4
            for y in range(top, top + 2):
                grid[y][x] = 100
            for y in range(bottom, bottom + 2):
                grid[y][x] = 100
    for x in range(29, 32):
        grid[22][x] = 100
    # A 30° corridor forces the grid planner to produce a large staircase.
    for x in range(44, WIDTH - 1):
        centre_y = 10 + (x - GOAL[0]) / sqrt(3)
        for y in range(1, HEIGHT - 1):
            if abs(y - centre_y) > 2.0:
                grid[y][x] = 100
    return grid


def make_door_attributes():
    attributes = CellAttributeMap(WIDTH, HEIGHT, [[[1, 0, 0, 0] for _ in range(WIDTH)] for _ in range(HEIGHT)])
    layer = TemporalTraversabilityLayer(attributes)
    for tick in range(36):
        blocked = (tick // 5) % 2 == 0
        layer.update(tick * 0.5, {point: blocked for point in DOOR})
    layer.predict(DoorPredictor(), DOOR)
    return attributes


def run_demo():
    grid = make_map()
    attributes = make_door_attributes()
    belief = min(attributes.values[y][x][DOOR_BELIEF] for x, y in DOOR)
    frames = []
    crossings = []
    for trip, (origin, destination) in enumerate(
        ((START, GOAL), (GOAL, DIAGONAL_GOAL),
         (DIAGONAL_GOAL, START), (START, CORNER_GOAL)), 1
    ):
        navigator = LiveNavigator(origin, destination, sensor_range=SENSOR_RANGE)
        for tick in range(280):
            people = people_at(trip, tick)
            closed = (DOOR_GROUPS[(23, "upper")] | DOOR_GROUPS[(35, "upper")]
                      if trip == 1 and tick >= 19 else
                      DOOR_GROUPS[(23, "lower")] | DOOR_GROUPS[(35, "lower")]
                      if trip == 3 else set())
            observed_grid = [row[:] for row in grid]
            for x, y in closed:
                observed_grid[y][x] = 100
            seen = visible_people(navigator.robot, people, observed_grid)
            previous = navigator.robot
            result = navigator.step(observed_grid, seen, closed, attributes)
            robot = result["robot"]
            for group, cells in DOOR_GROUPS.items():
                if ((round(previous[0]), round(previous[1])) not in cells
                        and (round(robot[0]), round(robot[1])) in cells):
                    crossings.append((trip, group))
            frames.append({**result, "closed": closed, "previous_robot": previous,
                           "tick": tick, "trip": trip, "destination": destination,
                           "crossings": len(crossings), "people": people,
                           "observed_people": seen,
                           "visible_people": visible_people(robot, people, observed_grid),
                           "visible_cells": visible_map_cells(robot, observed_grid),
                           "world": observed_grid})
            if robot == destination:
                break
        if robot != destination:
            raise RuntimeError(f"trip {trip} stalled at {robot}: {[(f['robot'], f['action']) for f in frames[-5:]]}")
    return grid, frames, belief


def save_debug_paths(frames, output, trip=1):
    """Save the actual A*, smoothing, and executed positions from this run."""
    def same_direction(a, b):
        return a[0] * b[1] == a[1] * b[0] and a[0] * b[0] + a[1] * b[1] > 0

    selected = [frame for frame in frames if trip == 0 or frame["trip"] == trip]
    if not selected:
        raise ValueError(f"no frames for trip {trip}")
    steps = []
    for frame in selected:
        path = frame["path"]
        turn_indices = [index for index in range(1, len(path) - 1)
                        if (path[index][0] - path[index - 1][0],
                            path[index][1] - path[index - 1][1]) !=
                        (path[index + 1][0] - path[index][0],
                         path[index + 1][1] - path[index][1])]
        turns = [path[index] for index in turn_indices]
        kept = []
        for index in turn_indices:
            if path[index] not in frame["waypoints"]:
                continue
            waypoint_index = frame["waypoints"].index(path[index])
            if not 0 < waypoint_index < len(frame["waypoints"]) - 1:
                continue
            before, corner, after = frame["waypoints"][waypoint_index - 1:waypoint_index + 2]
            if (same_direction((corner[0] - before[0], corner[1] - before[1]),
                               (path[index][0] - path[index - 1][0],
                                path[index][1] - path[index - 1][1]))
                    and same_direction((after[0] - corner[0], after[1] - corner[1]),
                                       (path[index + 1][0] - path[index][0],
                                        path[index + 1][1] - path[index][1]))):
                kept.append(index)
        steps.append({
            "trip": frame["trip"], "tick": frame["tick"],
            "planned_this_tick": frame["planned"], "action": frame["action"],
            "position_before": frame["previous_robot"],
            "position_after": frame["robot"],
            "astar_path": path,
            "smoothed_waypoints": frame["waypoints"],
            "raw_turns": turns,
            "turns_kept_in_waypoints": [path[index] for index in kept],
            "kept_turn_analysis": [
                {"point": path[index],
                 "safe_30_or_60_shortcut_from_here":
                     _zigzag_shortcut(frame["grid"], path, index) is not None,
                 "single_turn_45_rule_applies": len(turn_indices) == 1}
                for index in kept
            ],
            "observed_people": sorted(frame["observed_people"]),
            "closed_doors": sorted(frame["closed"]),
        })
    output = Path(output)
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps({"description": "A* route, smoothed output, and actual robot step",
                                  "trip": trip, "steps": steps}, indent=2), encoding="utf-8")
    return output


def show_demo(grid, frames, belief, output=None):
    import matplotlib.pyplot as plt
    import numpy as np
    from matplotlib.animation import FuncAnimation, PillowWriter
    from matplotlib.colors import ListedColormap
    from matplotlib.patches import Circle

    figure, axis = plt.subplots(figsize=(15, 8))
    axis.imshow([[int(cell >= 50) for cell in row] for row in grid],
                cmap=ListedColormap(["white", "black"]), vmin=0, vmax=1)
    fog = axis.imshow(np.ones((HEIGHT, WIDTH)),
                      cmap=ListedColormap([(0, 0, 0, 0), (0.55, 0.55, 0.55, 1)]),
                      vmin=0, vmax=1, zorder=2)
    axis.set_xticks(range(0, WIDTH, 4))
    axis.set_yticks(range(0, HEIGHT, 4))
    axis.grid(color="#dddddd", linewidth=0.4)
    axis.scatter(*START, marker="*", s=350, color="orange", zorder=5)
    axis.scatter(*GOAL, marker="*", s=350, color="limegreen", zorder=5)
    axis.scatter(*DIAGONAL_GOAL, marker="*", s=350, color="deepskyblue", zorder=5)
    axis.scatter(*CORNER_GOAL, marker="*", s=350, color="violet", zorder=5)
    axis.text(*START, "A", ha="center", va="bottom", weight="bold")
    axis.text(*GOAL, "B", ha="center", va="bottom", weight="bold")
    axis.text(*DIAGONAL_GOAL, "C", ha="center", va="bottom", weight="bold")
    axis.text(*CORNER_GOAL, "D", ha="center", va="bottom", weight="bold")
    door_markers = axis.scatter([], [], marker="s", s=170, color="turquoise", zorder=5)
    person_marker = axis.scatter([], [], marker="o", s=250, edgecolor="white", linewidth=2, zorder=6)
    robot_marker = axis.scatter([], [], marker="s", s=300, color="royalblue", edgecolor="yellow", linewidth=2, zorder=7)
    raw_line, = axis.plot([], [], color="gray", linestyle="--", linewidth=2.5,
                          marker=".", markersize=4, zorder=3)
    line, = axis.plot([], [], color="limegreen", linewidth=4, zorder=4)
    sensor_circle = Circle(START, SENSOR_RANGE, fill=False, linestyle="--",
                           edgecolor="dodgerblue", linewidth=1.5, zorder=6)
    axis.add_patch(sensor_circle)
    banner = axis.text(WIDTH / 2, 2, "", ha="center", fontsize=17, weight="bold",
                       bbox={"facecolor": "gold", "pad": 5}, zorder=9)

    def update(index):
        frame = frames[index]
        fog.set_data([[0 if (x, y) in frame["visible_cells"] else 1
                       for x in range(WIDTH)] for y in range(HEIGHT)])
        raw_path = frame["raw_path"]
        path = frame["trail"]
        raw_line.set_data([x for x, _ in raw_path], [y for _, y in raw_path])
        line.set_data([x for x, _ in path], [y for _, y in path])
        robot_marker.set_offsets([frame["robot"]])
        robot_marker.set_facecolor("orange" if frame["action"] == "WAIT" else "royalblue")
        sensor_circle.center = frame["robot"]
        people = frame["people"]
        person_marker.set_offsets(people if people else np.empty((0, 2)))
        person_marker.set_facecolors([
            "red" if point in frame["visible_people"] else
            (0, 0, 0, 0) if in_range(frame["robot"], point) else "gray"
            for point in people
        ])
        person_marker.set_edgecolors([
            "gold" if point in frame["visible_people"] else
            (0, 0, 0, 0) if in_range(frame["robot"], point) else "white"
            for point in people
        ])
        door_markers.set_offsets(sorted(DOOR))
        door_colors = [
            "gray" if not in_range(frame["robot"], point) else
            (0, 0, 0, 0) if not line_of_sight(frame["world"], frame["robot"], point) else
            "purple" if point in frame["closed"] else "turquoise"
            for point in sorted(DOOR)
        ]
        door_markers.set_facecolors(door_colors)
        door_markers.set_edgecolors(door_colors)
        banner.set_text(("WAITING FOR PERSON" if frame["human_active"] else "WAITING FOR DOOR")
                        if frame["action"] == "WAIT" else
                        "REROUTE: PERSON + DOOR" if frame["action"] == "REROUTE" and frame["combined"] else
                        "REROUTING" if frame["action"] == "REROUTE" else "")
        axis.set_title(f"Group 18 | trip {frame['trip']}/4 | step {frame['tick'] + 1} | {frame['action']}\n"
                       f"Gray dashed = raw A* | green = travelled | blue circle = sensor range | "
                       f"doors crossed {frame['crossings']}")
        return fog, raw_line, line, robot_marker, person_marker, door_markers, sensor_circle, banner

    animation = FuncAnimation(figure, update, frames=len(frames), interval=350, repeat=False)
    update(0)
    figure.tight_layout()
    if output:
        animation.save(output, writer=PillowWriter(fps=3))
    else:
        plt.show()
    return animation


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--save-gif")
    parser.add_argument("--debug-paths", nargs="?", const=".runtime/door_human_paths.json",
                        metavar="FILE", help="write routes and actual positions to JSON")
    parser.add_argument("--debug-trip", type=int, choices=range(5), default=1,
                        help="trip 1-4 to include in debug JSON; 0 includes all (default: 1)")
    parser.add_argument("--show", action="store_true", help="also open the animation window")
    args = parser.parse_args()
    grid, frames, belief = run_demo()
    print(f"frames={len(frames)} belief={belief:.3f} door_crossings={frames[-1]['crossings']} "
          f"waits={sum(frame['action'] == 'WAIT' for frame in frames)} "
          f"reroutes={sum(frame['action'] == 'REROUTE' for frame in frames)}")
    if args.debug_paths:
        print(f"debug_paths={save_debug_paths(frames, args.debug_paths, args.debug_trip)}")
    if args.save_gif or args.show or not args.debug_paths:
        show_demo(grid, frames, belief, args.save_gif)
