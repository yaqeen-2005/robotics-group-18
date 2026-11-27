import math
from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'controllers' / 'pallet_robot'))
from odometry import WheelOdometry


class WheelOdometryTests(unittest.TestCase):
    def setUp(self):
        self.odom = WheelOdometry(0.15, 0.82)
        self.assertFalse(self.odom.update(4, 5, 1))

    def assertPose(self, x, y, theta):
        for actual, expected in zip(self.odom.get_pose(), (x, y, theta)):
            self.assertAlmostEqual(actual, expected)

    def test_stationary(self):
        self.odom.update(4, 5, 2)
        self.assertPose(0, 0, 0)
        self.assertEqual(self.odom.get_velocity(), (0, 0))

    def test_forward(self):
        self.assertTrue(self.odom.update(6, 7, 2))
        self.assertPose(0.3, 0, 0)
        self.assertAlmostEqual(self.odom.get_velocity()[0], 0.3)

    def test_reverse(self):
        self.odom.update(2, 3, 2)
        self.assertPose(-0.3, 0, 0)

    def test_left_turn(self):
        self.odom.update(3, 6, 2)
        self.assertPose(0, 0, 0.3 / 0.82)
        self.assertAlmostEqual(self.odom.get_velocity()[1], 0.3 / 0.82)

    def test_right_turn(self):
        self.odom.update(5, 4, 2)
        self.assertPose(0, 0, -0.3 / 0.82)

    def test_arc(self):
        self.odom.update(4, 5 + 0.82 * math.pi / (2 * 0.15), 2)
        self.assertPose(0.41, 0.41, math.pi / 2)

    def test_turn_then_forward(self):
        turn = 0.82 * math.pi / (4 * 0.15)
        self.odom.update(4 - turn, 5 + turn, 2)
        self.odom.update(6 - turn, 7 + turn, 3)
        self.assertPose(0, 0.3, math.pi / 2)

    def test_invalid_and_duplicate_samples(self):
        self.assertFalse(self.odom.update(math.nan, 6, 2))
        self.assertFalse(self.odom.update(5, 6, 1))
        self.odom.update(6, 7, 3)
        self.assertPose(0.3, 0, 0)
        self.assertAlmostEqual(self.odom.get_velocity()[0], 0.15)


if __name__ == '__main__':
    unittest.main()
