"""Run PathManager against successive observations and execute safe waypoints."""

from common.inflate_map import inflate_map
from common.path_manager import PathManager
from common.path_smoothing import _clear, sample_waypoints


class LiveNavigator:
    def __init__(self, start, goal, manager=None, attention_distance=6, sensor_range=7):
        self.robot = start
        self.goal = goal
        self.manager = manager or PathManager()
        self.attention_distance = attention_distance
        self.sensor_range = sensor_range
        self.waiting = 0
        self.pending = []
        self.active_path = []
        self.active_waypoints = []
        self.trail = [start]

    def step(self, world, observed_people, closed_cells=(), attributes=None):
        """Take one checked motion step; observations contain only currently seen people."""
        robot = self.robot
        observed_people = set(observed_people)
        closed_cells = set(closed_cells)
        remaining = self.active_path if self.pending else self.manager._remaining_path(robot)
        blocking_people = {
            person for person in observed_people
            if any((point[0] - person[0]) ** 2 + (point[1] - person[1]) ** 2 <= 1
                   for point in remaining[1:])
        }
        nearest = min(blocking_people,
                      key=lambda point: abs(robot[0] - point[0]) + abs(robot[1] - point[1]),
                      default=None)
        human_distance = (abs(robot[0] - nearest[0]) + abs(robot[1] - nearest[1])
                          if nearest is not None else self.sensor_range + 1)
        human_active = nearest is not None and human_distance <= self.attention_distance
        relevant_doors = closed_cells.intersection(remaining)
        self.waiting = self.waiting + 1 if human_active or relevant_doors else 0
        inflated, _ = inflate_map(
            world, inflation_cells=1, dynamic_obstacles=observed_people,
            protected_points={(round(robot[0]), round(robot[1]))},
        )
        if inflated[round(robot[1])][round(robot[0])] >= 50:
            raise ValueError("robot is inside an obstacle")
        if inflated[self.goal[1]][self.goal[0]] >= 50:
            raise ValueError("goal is blocked")

        if self.pending:
            result = {"action": "MOVE", "path": self.active_path,
                      "waypoints": self.active_waypoints}
            planned = False
        else:
            result = self.manager.plan(
                inflated, robot, self.goal, attributes,
                human_blocking=human_active, human_distance=human_distance,
                waiting_time=self.waiting,
                human_cells={nearest} if human_active else None,
                door_cells=relevant_doors or None,
            )
            planned = True
            if result["action"] in ("MOVE", "REROUTE") and result["path"]:
                previous_route_matches = (
                    robot in self.active_path
                    and self.active_path[self.active_path.index(robot):] == result["path"]
                    and robot in self.active_waypoints
                    and result["action"] != "REROUTE"
                )
                if previous_route_matches:
                    result["waypoints"] = self.active_waypoints[
                        self.active_waypoints.index(robot):
                    ]
                else:
                    self.active_path = result["path"]
                    self.active_waypoints = result["waypoints"]

        previous = robot
        if result["action"] in ("MOVE", "REROUTE") and len(result["path"]) > 1:
            if not self.pending:
                waypoints, path = result["waypoints"], result["path"]
                if len(waypoints) > 1 and waypoints[1] != path[1]:
                    rejoin = next((index for index, point in enumerate(waypoints[1:], 1)
                                   if point in path[1:]), None)
                    if rejoin is not None:
                        self.pending = sample_waypoints(waypoints[:rejoin + 1])[1:]
            if self.pending:
                next_position = self.pending[0]
                if _clear(inflated, robot, next_position) and all(
                    (next_position[0] - person[0]) ** 2 +
                    (next_position[1] - person[1]) ** 2 >= 1
                    for person in observed_people
                ):
                    robot = self.pending.pop(0)
                    if not self.pending and all(abs(value - round(value)) < 1e-9 for value in robot):
                        robot = (round(robot[0]), round(robot[1]))
                else:
                    result["action"] = "WAIT"
            else:
                robot = result["path"][1]

        self.robot = robot
        if robot != previous:
            self.trail.append(robot)
        raw_path = (self.manager.raw_path(previous)
                    if not self.pending and previous in self.manager.current_path
                    else self.manager.raw_path())
        return {
            "robot": robot, "action": result["action"],
            "path": result["path"], "waypoints": result["waypoints"],
            "raw_path": raw_path, "grid": inflated, "trail": list(self.trail),
            "planned": planned, "human_active": human_active,
            "combined": human_active and bool(relevant_doors),
            "reroutes": self.manager.reroute_count,
        }
