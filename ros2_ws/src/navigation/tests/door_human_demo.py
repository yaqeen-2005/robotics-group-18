"""Animate two trips through two layers of door-like warehouse passages."""

import argparse
from math import sqrt

from astar.a_star import plan_path
from common.cell_attributes import CellAttributeMap, DOOR_BELIEF
from common.inflate_map import inflate_map
from common.path_manager import PathManager
from common.path_smoothing import sample_waypoints
from common.special_obstacles import SpecialObstacles
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


def people_for_trip(trip):
    return ((PERSON,) if trip == 1 else ()) + BACKGROUND_PEOPLE


def visible_people(robot, people):
    return {point for point in people
            if (point[0] - robot[0]) ** 2 + (point[1] - robot[1]) ** 2 <= SENSOR_RANGE ** 2}


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
            if abs(y - centre_y) > 3.2:
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
        manager = PathManager(special_obstacles=SpecialObstacles(door_distance=4))
        people = people_for_trip(trip)
        if trip in (2, 4):
            inflated, _ = inflate_map(grid, inflation_cells=1,
                                      dynamic_obstacles=visible_people(origin, people),
                                      protected_points={origin})
            # This task intentionally allows A*'s natural grid staircase so
            # the turn-reversal shortcut can be seen and tested clearly.
            result = manager.plan(inflated, origin, destination, attributes,
                                  weights={"turn": 0.0} if trip == 2 else None)
            positions = sample_waypoints(result["waypoints"])
            raw = manager.raw_path(origin)
            for tick, position in enumerate(positions):
                frames.append({"robot": position, "path": result["path"],
                               "waypoints": result["waypoints"], "action": "MOVE",
                               "closed": set(), "human_active": False,
                               "tick": tick, "trip": trip, "destination": destination,
                               "reroutes": 0, "crossings": len(crossings),
                               "combined": False, "grid": inflated,
                               "raw_path": raw, "trail": positions[:tick + 1],
                               "people": people,
                               "visible_people": visible_people(position, people)})
            if not frames or frames[-1]["robot"] != destination:
                raise RuntimeError(f"trip {trip} did not reach its destination")
            continue
        manager.plan(grid, origin, destination)
        robot = origin
        trail = [robot]
        waiting = 0
        for tick in range(140):
            remaining = manager._remaining_path(robot)
            seen = visible_people(robot, people)
            blocking_people = seen.intersection(remaining[1:])
            nearest_person = min(blocking_people,
                                 key=lambda point: abs(robot[0] - point[0]) + abs(robot[1] - point[1]),
                                 default=None)
            human_active = nearest_person is not None and (abs(robot[0] - nearest_person[0])
                                                            + abs(robot[1] - nearest_person[1]) <= 6)
            waiting = waiting + 1 if human_active else 0
            closed = (DOOR_GROUPS[(23, "upper")] | DOOR_GROUPS[(35, "upper")]
                      if trip == 1 and tick >= 19 else
                      DOOR_GROUPS[(23, "lower")] | DOOR_GROUPS[(35, "lower")]
                      if trip == 3 else set())
            dynamic = seen
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
                human_distance=(abs(robot[0] - nearest_person[0]) + abs(robot[1] - nearest_person[1])
                                if nearest_person is not None else SENSOR_RANGE + 1),
                waiting_time=waiting,
                human_cells={nearest_person} if human_active else None,
                door_cells=relevant_doors or None,
            )
            previous = robot
            if result["action"] in ("MOVE", "REROUTE") and len(result["path"]) > 1:
                robot = result["path"][1]
            if robot != previous:
                trail.append(robot)
            for group, cells in DOOR_GROUPS.items():
                if previous not in cells and robot in cells:
                    crossings.append((trip, group))
            frames.append({"robot": robot, "path": result["path"],
                           "waypoints": result["waypoints"], "action": result["action"],
                           "closed": closed, "human_active": human_active,
                           "tick": tick, "trip": trip, "destination": destination,
                           "reroutes": manager.reroute_count, "crossings": len(crossings),
                           "combined": human_active and bool(relevant_doors),
                           "grid": inflated, "raw_path": manager.raw_path(previous),
                           "trail": list(trail), "people": people,
                           "visible_people": visible_people(robot, people)})
            if robot == destination:
                break
        if robot != destination:
            raise RuntimeError(f"trip {trip} stalled at {robot}: {[(f['robot'], f['action']) for f in frames[-5:]]}")
    return grid, frames, belief


def show_demo(grid, frames, belief, output=None):
    import matplotlib.pyplot as plt
    import numpy as np
    from matplotlib.animation import FuncAnimation, PillowWriter
    from matplotlib.colors import ListedColormap
    from matplotlib.patches import Circle

    figure, axis = plt.subplots(figsize=(15, 8))
    axis.imshow([[int(cell >= 50) for cell in row] for row in grid],
                cmap=ListedColormap(["white", "black"]), vmin=0, vmax=1)
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
        raw_path = frame["raw_path"]
        path = frame["trail"]
        raw_line.set_data([x for x, _ in raw_path], [y for _, y in raw_path])
        line.set_data([x for x, _ in path], [y for _, y in path])
        robot_marker.set_offsets([frame["robot"]])
        robot_marker.set_facecolor("orange" if frame["action"] == "WAIT" else "royalblue")
        sensor_circle.center = frame["robot"]
        people = frame["people"]
        person_marker.set_offsets(people if people else np.empty((0, 2)))
        person_marker.set_facecolors(["red" if point in frame["visible_people"] else "gray"
                                      for point in people])
        person_marker.set_edgecolors(["gold" if point in frame["visible_people"] else "white"
                                      for point in people])
        door_markers.set_offsets(sorted(DOOR))
        door_markers.set_facecolors([
            "gray" if (point[0] - frame["robot"][0]) ** 2 +
            (point[1] - frame["robot"][1]) ** 2 > SENSOR_RANGE ** 2 else
            "purple" if point in frame["closed"] else "turquoise"
            for point in sorted(DOOR)
        ])
        banner.set_text("WAITING FOR PERSON" if frame["action"] == "WAIT" else
                        "REROUTE: PERSON + DOOR" if frame["action"] == "REROUTE" and frame["combined"] else
                        "REROUTING" if frame["action"] == "REROUTE" else "")
        axis.set_title(f"Group 18 | trip {frame['trip']}/4 | step {frame['tick'] + 1} | {frame['action']}\n"
                       f"Gray dashed = raw A* | green = travelled | blue circle = sensor range | "
                       f"doors crossed {frame['crossings']}")
        return raw_line, line, robot_marker, person_marker, door_markers, sensor_circle, banner

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
