import unittest

from tests.door_human_demo import DOOR, GOAL, PERSON, run_demo


class DoorHumanDemoTest(unittest.TestCase):
    def test_trained_vertical_door_and_joint_reroute(self):
        _, frames, belief = run_demo()
        self.assertGreater(belief, 0.6)
        self.assertEqual(frames[-1]["robot"], GOAL)
        self.assertTrue(any(frame["action"] == "WAIT" for frame in frames))
        self.assertTrue(any(frame["action"] == "REROUTE" and frame["human_active"]
                            and frame["door_closed"] for frame in frames))
        self.assertTrue(all(frame["robot"] != PERSON for frame in frames))
        self.assertTrue(all(not frame["door_closed"] or frame["robot"] not in DOOR
                            for frame in frames))


if __name__ == "__main__":
    unittest.main()
