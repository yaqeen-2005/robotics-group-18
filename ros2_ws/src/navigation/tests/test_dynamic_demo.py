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
        self.assertTrue(demo["report"])


if __name__ == "__main__":
    unittest.main()
