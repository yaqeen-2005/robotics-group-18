import unittest

from tests.dynamic_navigation_demo import GOAL, run_demo


class DynamicDemoTest(unittest.TestCase):
    def test_demo_waits_reroutes_and_reaches_goal(self):
        demo = run_demo()
        actions = [frame["action"] for frame in demo["frames"]]
        self.assertIn("WAIT", actions)
        self.assertIn("REROUTE", actions)
        self.assertEqual(tuple(demo["frames"][-1]["robot"]), GOAL)
        self.assertTrue(demo["report"])


if __name__ == "__main__":
    unittest.main()
