import unittest
from math import atan2, degrees, hypot

from tests.door_human_demo import (
    CORNER_GOAL, DIAGONAL_GOAL, GOAL, PERSON, START, DOOR_GROUPS,
    in_range, line_of_sight, make_map, people_at, run_demo,
)
from common.path_smoothing import _clear, smooth_path


class DoorHumanDemoTest(unittest.TestCase):
    def test_trained_vertical_door_and_joint_reroute(self):
        _, frames, belief = run_demo()
        self.assertGreater(belief, 0.6)
        self.assertEqual(frames[-1]["robot"], CORNER_GOAL)
        self.assertTrue(any(frame["trip"] == 1 and frame["robot"] == GOAL for frame in frames))
        self.assertTrue(any(frame["trip"] == 2 and frame["robot"] == DIAGONAL_GOAL for frame in frames))
        self.assertTrue(any(frame["trip"] == 3 and frame["robot"] == START for frame in frames))
        self.assertEqual(frames[-1]["crossings"], 4)
        entries = [(frame["trip"], round(frame["robot"][0])) for previous, frame in zip(frames, frames[1:])
                   if frame["crossings"] > previous["crossings"]]
        self.assertEqual(entries, [(1, 23), (1, 35), (3, 35), (3, 23)])
        self.assertTrue(any(frame["action"] == "WAIT" for frame in frames))
        self.assertTrue(any(frame["action"] == "WAIT" and frame["human_active"] for frame in frames))
        self.assertTrue(any(frame["action"] == "REROUTE" and frame["closed"] for frame in frames))
        self.assertTrue(all(frame["trip"] != 1 or frame["robot"] not in frame["people"] for frame in frames))
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
            a["robot"][0] != b["robot"][0] and a["robot"][1] != b["robot"][1]
            for a, b in zip(first_trip, first_trip[1:])
        ))
        self.assertTrue(all(
            _clear(frame["grid"], before["robot"], frame["robot"])
            for before, frame in zip(first_trip, first_trip[1:])
            if before["robot"] != frame["robot"]
        ))
        diagonal_trip = next(frame for frame in frames if frame["trip"] == 2)
        smoothed = diagonal_trip["waypoints"]
        actual_positions = [frame["robot"] for frame in frames if frame["trip"] == 2]
        self.assertTrue(any(position not in diagonal_trip["path"] for position in actual_positions))
        self.assertTrue(all(_clear(diagonal_trip["grid"], a, b)
                            for a, b in zip(actual_positions, actual_positions[1:])))
        long_segments = [(a, b) for a, b in zip(smoothed, smoothed[1:])
                         if hypot(b[0] - a[0], b[1] - a[1]) > 10]
        self.assertEqual(len(long_segments), 1)
        anchor, hit = long_segments[0]
        self.assertIn(anchor, diagonal_trip["path"])
        self.assertNotEqual(anchor, diagonal_trip["path"][0])
        self.assertAlmostEqual(degrees(atan2(hit[1] - anchor[1], hit[0] - anchor[0])), 30)
        self.assertTrue(any(
            a[0] == b[0] == hit[0] and min(a[1], b[1]) <= hit[1] <= max(a[1], b[1])
            or a[1] == b[1] == hit[1] and min(a[0], b[0]) <= hit[0] <= max(a[0], b[0])
            for a, b in zip(diagonal_trip["path"], diagonal_trip["path"][1:])
        ))
        self.assertGreater(sum(anchor[0] < x < hit[0] for x, y in actual_positions), 10)
        self.assertTrue(all(
            abs((x - anchor[0]) * (hit[1] - anchor[1])
                - (y - anchor[1]) * (hit[0] - anchor[0])) < 1e-6
            for x, y in actual_positions if anchor[0] < x < hit[0]
        ))
        self.assertTrue(all(_clear(diagonal_trip["grid"], a, b)
                            for a, b in zip(smoothed, smoothed[1:])))
        corner_trip = next(frame for frame in frames if frame["trip"] == 4)
        self.assertNotEqual(corner_trip["raw_path"], corner_trip["waypoints"])
        self.assertEqual(len(corner_trip["waypoints"]), 2)
        self.assertAlmostEqual(
            abs(degrees(atan2(CORNER_GOAL[1] - START[1], CORNER_GOAL[0] - START[0]))), 45
        )
        self.assertTrue(all(_clear(corner_trip["grid"], a, b)
                            for a, b in zip(corner_trip["trail"], corner_trip["trail"][1:])))
        return_trip = next(frame for frame in frames if frame["trip"] == 3)
        self.assertTrue(any(
            hypot(b[0] - a[0], b[1] - a[1]) > 8
            and abs(abs(degrees(atan2(b[1] - a[1], b[0] - a[0]))) - 150) < 0.01
            for a, b in zip(return_trip["waypoints"], return_trip["waypoints"][1:])
        ))
        self.assertTrue(any(
            a[0] != b[0] and a[1] != b[1]
            for frame in frames for a, b in zip(
                smooth_path(frame["grid"], frame["path"]),
                smooth_path(frame["grid"], frame["path"])[1:],
            )
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
