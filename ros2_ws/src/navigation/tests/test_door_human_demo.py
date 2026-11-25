import unittest
from math import atan2, degrees, hypot

from tests.door_human_demo import DIAGONAL_GOAL, GOAL, PERSON, START, run_demo
from common.path_smoothing import _clear, smooth_path


class DoorHumanDemoTest(unittest.TestCase):
    def test_trained_vertical_door_and_joint_reroute(self):
        _, frames, belief = run_demo()
        self.assertGreater(belief, 0.6)
        self.assertEqual(frames[-1]["robot"], START)
        self.assertTrue(any(frame["trip"] == 1 and frame["robot"] == GOAL for frame in frames))
        self.assertTrue(any(frame["trip"] == 2 and frame["robot"] == DIAGONAL_GOAL for frame in frames))
        self.assertEqual(frames[-1]["crossings"], 4)
        entries = [(frame["trip"], frame["robot"][0]) for previous, frame in zip(frames, frames[1:])
                   if frame["crossings"] > previous["crossings"]]
        self.assertEqual(entries, [(1, 23), (1, 35), (3, 35), (3, 23)])
        self.assertTrue(any(frame["action"] == "WAIT" for frame in frames))
        self.assertTrue(any(frame["action"] == "REROUTE" and frame["combined"]
                            for frame in frames))
        self.assertTrue(all(frame["trip"] != 1 or frame["robot"] != PERSON for frame in frames))
        self.assertTrue(all(frame["robot"] not in frame["closed"]
                            for frame in frames))
        self.assertTrue(any(frame["visual_robot"] != frame["robot"] for frame in frames))
        diagonal_trip = next(frame for frame in frames if frame["trip"] == 2)
        smoothed = smooth_path(diagonal_trip["grid"], diagonal_trip["path"])
        self.assertTrue(any(hypot(b[0] - a[0], b[1] - a[1]) > 15
                            and abs(degrees(atan2(b[1] - a[1], b[0] - a[0])) - 30) < 0.01
                            for a, b in zip(smoothed, smoothed[1:])))
        self.assertTrue(all(_clear(diagonal_trip["grid"], a, b)
                            for a, b in zip(smoothed, smoothed[1:])))
        self.assertTrue(any(
            a[0] != b[0] and a[1] != b[1]
            for frame in frames for a, b in zip(
                smooth_path(frame["grid"], frame["path"]),
                smooth_path(frame["grid"], frame["path"])[1:],
            )
        ))


if __name__ == "__main__":
    unittest.main()
