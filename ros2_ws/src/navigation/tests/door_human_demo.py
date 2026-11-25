"""Animate two trips through two layers of door-like warehouse passages."""

import argparse

from astar.a_star import plan_path
from common.cell_attributes import CellAttributeMap, DOOR_BELIEF
from common.inflate_map import inflate_map
from common.path_manager import PathManager
from common.path_smoothing import smooth_path
from common.special_obstacles import SpecialObstacles
from common.temporal_traversability import TemporalTraversabilityLayer
from ml.door_belief import DoorPredictor

WIDTH, HEIGHT = 48, 30
START, GOAL = (4, 10), (43, 10)
DOOR_GROUPS = {
    (wall, passage): {(wall, y) for y in (range(8, 12) if passage == "upper" else range(21, 25))}
    for wall in (23, 35) for passage in ("upper", "lower")
}
DOOR = set().union(*DOOR_GROUPS.values())
PERSON = (26, 10)


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
    for trip, (origin, destination) in enumerate(((START, GOAL), (GOAL, START)), 1):
        manager = PathManager(special_obstacles=SpecialObstacles(door_distance=4))
        manager.plan(grid, origin, destination)
        robot = origin
        waiting = 0
        for tick in range(140):
            remaining = manager._remaining_path(robot)
            human_active = trip == 1 and PERSON in remaining[1:] and abs(robot[0] - PERSON[0]) + abs(robot[1] - PERSON[1]) <= 6
            waiting = waiting + 1 if human_active else 0
            closed = (DOOR_GROUPS[(23, "upper")] | DOOR_GROUPS[(35, "upper")]
                      if trip == 1 and tick >= 19 else
                      DOOR_GROUPS[(23, "lower")] | DOOR_GROUPS[(35, "lower")]
                      if trip == 2 else set())
            dynamic = {PERSON} if human_active else set()
            observed_grid = [row[:] for row in grid]
            for x, y in closed:
                observed_grid[y][x] = 100
            inflated, _ = inflate_map(observed_grid, inflation_cells=1,
                                      dynamic_obstacles=dynamic,
                                      protected_points={robot})
            relevant_doors = closed.intersection(remaining)
            assert inflated[robot[1]][robot[0]] < 50, (trip, tick, robot)
            assert inflated[destination[1]][destination[0]] < 50
            result = manager.plan(
                inflated, robot, destination, attributes,
                human_blocking=human_active,
                human_distance=abs(robot[0] - PERSON[0]) + abs(robot[1] - PERSON[1]),
                waiting_time=waiting,
                human_cells=dynamic,
                door_cells=relevant_doors or None,
            )
            previous = robot
            if result["action"] in ("MOVE", "REROUTE") and len(result["path"]) > 1:
                robot = result["path"][1]
            for group, cells in DOOR_GROUPS.items():
                if previous not in cells and robot in cells:
                    crossings.append((trip, group))
            frames.append({"robot": robot, "path": result["path"], "action": result["action"],
                           "closed": closed, "human_active": human_active,
                           "tick": tick, "trip": trip, "destination": destination,
                           "reroutes": manager.reroute_count, "crossings": len(crossings),
                           "combined": human_active and bool(relevant_doors),
                           "grid": inflated})
            if robot == destination:
                break
        if robot != destination:
            raise RuntimeError(f"trip {trip} stalled at {robot}: {[(f['robot'], f['action']) for f in frames[-5:]]}")
    for index, frame in enumerate(frames):
        frame["visual_robot"] = frame["robot"]
        if index == 0 or index + 1 == len(frames):
            continue
        before, after = frames[index - 1], frames[index + 1]
        if before["trip"] != frame["trip"] or after["trip"] != frame["trip"]:
            continue
        points = [before["robot"], frame["robot"], after["robot"]]
        smoothed = smooth_path(frame["grid"], points)
        if len(smoothed) == 5:
            frame["visual_robot"] = smoothed[2]
    return grid, frames, belief


def show_demo(grid, frames, belief, output=None):
    import matplotlib.pyplot as plt
    import numpy as np
    from matplotlib.animation import FuncAnimation, PillowWriter
    from matplotlib.colors import ListedColormap

    figure, axis = plt.subplots(figsize=(13, 8))
    axis.imshow([[int(cell >= 50) for cell in row] for row in grid],
                cmap=ListedColormap(["white", "black"]), vmin=0, vmax=1)
    axis.set_xticks(range(0, WIDTH, 4))
    axis.set_yticks(range(0, HEIGHT, 4))
    axis.grid(color="#dddddd", linewidth=0.4)
    axis.scatter(*START, marker="*", s=350, color="orange", zorder=5)
    axis.scatter(*GOAL, marker="*", s=350, color="limegreen", zorder=5)
    axis.text(*START, "A", ha="center", va="bottom", weight="bold")
    axis.text(*GOAL, "B", ha="center", va="bottom", weight="bold")
    door_markers = axis.scatter([], [], marker="s", s=170, color="turquoise", zorder=5)
    person_marker = axis.scatter([], [], marker="o", s=300, color="red", edgecolor="gold", linewidth=2, zorder=6)
    robot_marker = axis.scatter([], [], marker="s", s=300, color="royalblue", edgecolor="yellow", linewidth=2, zorder=7)
    line, = axis.plot([], [], color="limegreen", linewidth=3, zorder=4)
    from matplotlib.collections import LineCollection
    diagonal = LineCollection([], colors="magenta", linewidths=4, zorder=5)
    axis.add_collection(diagonal)
    banner = axis.text(WIDTH / 2, 2, "", ha="center", fontsize=17, weight="bold",
                       bbox={"facecolor": "gold", "pad": 5}, zorder=9)

    def update(index):
        frame = frames[index]
        path = smooth_path(frame["grid"], frame["path"])
        line.set_data([x for x, _ in path], [y for _, y in path])
        diagonal.set_segments([(a, b) for a, b in zip(path, path[1:])
                               if abs(a[0] - b[0]) > 0.01 and abs(a[1] - b[1]) > 0.01])
        robot_marker.set_offsets([frame["visual_robot"]])
        robot_marker.set_facecolor("orange" if frame["action"] == "WAIT" else "royalblue")
        person_marker.set_offsets([PERSON] if frame["trip"] == 1 else np.empty((0, 2)))
        door_markers.set_offsets(sorted(DOOR))
        door_markers.set_facecolors(["purple" if point in frame["closed"] else "turquoise"
                                     for point in sorted(DOOR)])
        banner.set_text("WAITING FOR PERSON" if frame["action"] == "WAIT" else
                        "REROUTE: PERSON + DOOR" if frame["action"] == "REROUTE" and frame["combined"] else
                        "REROUTING" if frame["action"] == "REROUTE" else "")
        axis.set_title(f"Group 18 | trip {frame['trip']}/2 | step {frame['tick'] + 1} | {frame['action']}\n"
                       f"Door belief {belief:.2f} | doors crossed {frame['crossings']} | "
                       "cyan open / purple closed / magenta safe 30°/60° turns")
        return line, diagonal, robot_marker, person_marker, door_markers, banner

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
    args = parser.parse_args()
    grid, frames, belief = run_demo()
    print(f"frames={len(frames)} belief={belief:.3f} door_crossings={frames[-1]['crossings']} "
          f"waits={sum(frame['action'] == 'WAIT' for frame in frames)} "
          f"reroutes={sum(frame['action'] == 'REROUTE' for frame in frames)}")
    show_demo(grid, frames, belief, args.save_gif)
