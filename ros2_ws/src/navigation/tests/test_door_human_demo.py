import json
import tempfile
import unittest
from math import hypot
from pathlib import Path

from tests.door_human_demo import (
    CORNER_GOAL, DIAGONAL_GOAL, GOAL, PERSON, START, DOOR_GROUPS,
    in_range, line_of_sight, make_map, people_at, run_demo, save_debug_paths,
)
from common.path_smoothing import _clear


class DoorHumanDemoTest(unittest.TestCase):
    def test_all_trips_use_live_navigation_and_safe_execution(self):
        _, frames, belief = run_demo()
        self.assertGreater(belief, 0.6)
        for trip, destination in enumerate((GOAL, DIAGONAL_GOAL, START, CORNER_GOAL), 1):
            trip_frames = [frame for frame in frames if frame["trip"] == trip]
            self.assertEqual(trip_frames[-1]["robot"], destination)
            self.assertTrue(trip_frames[0]["planned"])
            if trip != 4:
                self.assertGreater(sum(frame["planned"] for frame in trip_frames), 1)
            self.assertTrue(all(
                _clear(frame["grid"], before["robot"], frame["robot"])
                for before, frame in zip(trip_frames, trip_frames[1:])
            ))
        self.assertEqual(frames[-1]["crossings"], 4)
        entries = [(frame["trip"], round(frame["robot"][0])) for previous, frame in zip(frames, frames[1:])
                   if frame["crossings"] > previous["crossings"]]
        self.assertEqual(entries, [(1, 23), (1, 35), (3, 35), (3, 23)])
        self.assertTrue(any(frame["action"] == "WAIT" and frame["human_active"] for frame in frames))
        self.assertTrue(any(frame["action"] == "REROUTE" and frame["closed"] for frame in frames))
        self.assertTrue(all(
            (frame["robot"][0] - person[0]) ** 2 +
            (frame["robot"][1] - person[1]) ** 2 >= 1
            for frame in frames for person in frame["people"]
        ))
        self.assertTrue(all((round(frame["robot"][0]), round(frame["robot"][1])) not in frame["closed"]
                            for frame in frames))
        self.assertNotIn(PERSON, frames[0]["visible_people"])
        self.assertTrue(any(frame["human_active"] for frame in frames))
        self.assertTrue(all(len(frame["people"]) >= 3 for frame in frames))
        self.assertNotEqual(people_at(1, 0), people_at(1, 8))
        self.assertTrue(all(frame["visible_people"].issubset(frame["visible_cells"])
                            for frame in frames))
        self.assertTrue(all(frame["trail"][-1] == frame["robot"] for frame in frames))
        first_trip = [frame for frame in frames if frame["trip"] == 1]
        self.assertTrue(any(
            27 <= before["robot"][0] <= 35
            and before["robot"][0] != frame["robot"][0]
            and before["robot"][1] != frame["robot"][1]
            for before, frame in zip(first_trip, first_trip[1:])
        ))
        for trip in (2, 3):
            trip_frames = [frame for frame in frames if frame["trip"] == trip]
            self.assertTrue(any(
                hypot(b[0] - a[0], b[1] - a[1]) > 8
                for frame in trip_frames for a, b in zip(frame["waypoints"], frame["waypoints"][1:])
                if a[0] != b[0] and a[1] != b[1]
            ))

    def test_debug_paths_record_planned_and_executed_routes(self):
        _, frames, _ = run_demo()
        with tempfile.TemporaryDirectory() as directory:
            report = save_debug_paths(frames, Path(directory) / "paths.json")
            steps = json.loads(report.read_text(encoding="utf-8"))["steps"]
        first_trip = [frame for frame in frames if frame["trip"] == 1]
        self.assertEqual(len(steps), len(first_trip))
        self.assertEqual(steps[0]["astar_path"], [list(point) for point in first_trip[0]["path"]])
        tunnel_plan = next(step for step in steps if step["tick"] == 21)
        self.assertIn([20, 22], tunnel_plan["turns_kept_in_waypoints"])
        self.assertNotIn([27, 22], tunnel_plan["turns_kept_in_waypoints"])
        self.assertTrue(any(
            step["position_before"][0] != step["position_after"][0]
            and step["position_before"][1] != step["position_after"][1]
            for step in steps
        ))

    def test_closed_door_hides_a_person_inside_sensor_radius(self):
        grid = make_map()
        for x, y in DOOR_GROUPS[(23, "upper")]:
            grid[y][x] = 100
        robot, person = (20, 10), (26, 10)
        self.assertTrue(in_range(robot, person))
        self.assertFalse(line_of_sight(grid, robot, person))


if __name__ == "__main__":
    unittest.main()
