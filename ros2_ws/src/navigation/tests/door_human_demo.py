"""Show a trained door belief and a person jointly triggering one reroute."""

import argparse

from astar.a_star import plan_path
from common.cell_attributes import CellAttributeMap, DOOR_BELIEF
from common.inflate_map import inflate_map
from common.path_manager import PathManager
from common.temporal_traversability import TemporalTraversabilityLayer
from ml.door_belief import DoorPredictor

WIDTH, HEIGHT = 48, 30
START, GOAL = (4, 10), (43, 10)
DOOR = {(23, y) for y in range(8, 12)}
PERSON = (19, 10)


def make_map():
    grid = [[0] * WIDTH for _ in range(HEIGHT)]
    for y in range(HEIGHT):
        for x in range(WIDTH):
            if x in (0, WIDTH - 1) or y in (0, HEIGHT - 1):
                grid[y][x] = 100
    for y in range(1, HEIGHT - 1):
        if y not in range(8, 12) and y not in range(21, 25):
            grid[y][23] = 100
    # Slanted, uneven shelf ends; they do not close either passage.
    for base in (4, 32):
        for x in range(base, base + 10):
            top = 3 + (x - base) // 3
            bottom = 26 - (x - base) // 4
            for y in range(top, top + 2):
                grid[y][x] = 100
            for y in range(bottom, bottom + 2):
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
    manager = PathManager()
    manager.plan(grid, START, GOAL)
    robot = START
    frames = []
    waiting = 0
    for tick in range(110):
        remaining = manager._remaining_path(robot)
        human_active = PERSON in remaining[1:] and abs(robot[0] - PERSON[0]) + abs(robot[1] - PERSON[1]) <= 6
        if human_active:
            waiting += 1
        else:
            waiting = 0
        door_closed = tick >= 14
        # A current sensor observation, not a historical SLAM occupancy value.
        dynamic = {PERSON} if human_active else set()
        observed_grid = [row[:] for row in grid]
        if door_closed:
            for x, y in DOOR:
                observed_grid[y][x] = 100
        inflated, _ = inflate_map(observed_grid, inflation_cells=1,
                                  dynamic_obstacles=dynamic,
                                  protected_points={robot})
        door_on_path = door_closed and bool(DOOR.intersection(remaining))
        assert inflated[robot[1]][robot[0]] < 50, (tick, robot)
        assert inflated[GOAL[1]][GOAL[0]] < 50, (tick, GOAL)
        result = manager.plan(
            inflated, robot, GOAL, attributes,
            human_blocking=human_active,
            human_distance=abs(robot[0] - PERSON[0]) + abs(robot[1] - PERSON[1]),
            waiting_time=waiting,
            human_cells=dynamic,
            door_cells=DOOR if door_on_path else None,
        )
        if result["action"] in ("MOVE", "REROUTE") and len(result["path"]) > 1:
            robot = result["path"][1]
        frames.append({"robot": robot, "path": result["path"], "action": result["action"],
                       "door_closed": door_closed, "human_active": human_active,
                       "tick": tick, "reroutes": manager.reroute_count})
        if robot == GOAL:
            break
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
    axis.text(*START, "PICKUP", ha="center", va="bottom", weight="bold")
    axis.text(*GOAL, "DELIVERY", ha="center", va="bottom", weight="bold")
    door_markers = axis.scatter([], [], marker="s", s=170, color="purple", zorder=5)
    person_marker = axis.scatter([], [], marker="o", s=300, color="red", edgecolor="gold", linewidth=2, zorder=6)
    robot_marker = axis.scatter([], [], marker="s", s=300, color="royalblue", edgecolor="yellow", linewidth=2, zorder=7)
    line, = axis.plot([], [], color="limegreen", linewidth=3, zorder=4)
    banner = axis.text(WIDTH / 2, 2, "", ha="center", fontsize=17, weight="bold",
                       bbox={"facecolor": "gold", "pad": 5}, zorder=9)

    def update(index):
        frame = frames[index]
        path = frame["path"]
        line.set_data([x for x, _ in path], [y for _, y in path])
        robot_marker.set_offsets([frame["robot"]])
        robot_marker.set_facecolor("orange" if frame["action"] == "WAIT" else "royalblue")
        person_marker.set_offsets([PERSON])
        door_markers.set_offsets(sorted(DOOR) if frame["door_closed"] else np.empty((0, 2)))
        banner.set_text({"WAIT": "WAITING FOR PERSON", "REROUTE": "REROUTE: PERSON + DOOR"}.get(frame["action"], ""))
        axis.set_title(f"Group 18 | step {frame['tick'] + 1} | {frame['action']} | "
                       f"trained door belief {belief:.2f} | reroutes {frame['reroutes']}")
        return line, robot_marker, person_marker, door_markers, banner

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
    print(f"frames={len(frames)} belief={belief:.3f} "
          f"waits={sum(frame['action'] == 'WAIT' for frame in frames)} "
          f"reroutes={sum(frame['action'] == 'REROUTE' for frame in frames)}")
    show_demo(grid, frames, belief, args.save_gif)
