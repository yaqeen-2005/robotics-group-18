import unittest

from tests.dynamic_navigation_demo import run_demo


class DynamicDemoTest(unittest.TestCase):
    def test_demo_waits_reroutes_and_reaches_goal(self):
        demo = run_demo()
        actions = [frame["action"] for frame in demo["frames"]]
        self.assertIn("WAIT", actions)
        self.assertIn("REROUTE", actions)
        self.assertEqual(demo["frames"][-1]["robot"], demo["frames"][-1]["goal"])
        self.assertEqual({frame["task_id"] for frame in demo["frames"]}, {1, 2, 3})
        for task in demo["tasks"]:
            final = [frame for frame in demo["frames"] if frame["task_id"] == task["task_id"]][-1]
            self.assertEqual(final["robot"], final["goal"])
        self.assertTrue(
            all(frame["robot"] not in frame["humans"] for frame in demo["frames"])
        )
        for previous, current in zip(demo["frames"], demo["frames"][1:]):
            if previous["task_id"] != current["task_id"]:
                continue
            for old_human, new_human in zip(previous["humans"], current["humans"]):
                self.assertFalse(
                    previous["robot"] == new_human
                    and current["robot"] == old_human,
                    "robot and person must not pass through each other",
                )
        self.assertTrue(
            all(
                frame["human_distance"] <= frame["attention_distance"]
                for frame in demo["frames"]
                if frame["action"] == "WAIT"
            )
        )
        self.assertTrue(demo["report"])


if __name__ == "__main__":
    unittest.main()
