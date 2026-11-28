import math
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'controllers' / 'pallet_robot'))
from sensor_interface import SensorInterface
from odometry import WheelOdometry


class SensorInterfaceTests(unittest.TestCase):
    def setUp(self):
        self.sensors = SensorInterface(360, math.pi, 0.05, 12.0)
        self.odom = WheelOdometry(0.15, 0.911)
        self.odom.update(0, 0, 0)
        self.updated = self.odom.update(1, 1, 0.25)

    def capture(self, scan):
        return self.sensors.capture(0.25, scan, self.odom, self.updated)

    def test_frame_pose_velocity_and_time(self):
        frame = self.capture([2.0] * 360)
        self.assertEqual(frame.time_s, 0.25)
        self.assertAlmostEqual(frame.odom_pose[0], 0.15)
        self.assertAlmostEqual(frame.velocity[0], 0.6)
        self.assertEqual(len(frame.ranges_m), 360)
        self.assertEqual(len(frame.points_lidar_m), 360)

    def test_angles_and_orientation(self):
        frame = self.capture([2.0] * 360)
        self.assertAlmostEqual(math.degrees(frame.angles_rad[0]), 89.75)
        self.assertAlmostEqual(math.degrees(frame.angles_rad[-1]), -89.75)
        self.assertAlmostEqual(math.degrees(frame.angles_rad[180]), -0.25)
        self.assertGreater(frame.points_lidar_m[0][1], 0)
        self.assertLess(frame.points_lidar_m[-1][1], 0)

    def test_offset(self):
        frame = self.capture([2.0] * 360)
        for (x, y), (bx, by) in zip(frame.points_lidar_m, frame.points_base_m):
            self.assertAlmostEqual(bx - x, 0.52)
            self.assertAlmostEqual(by, y)

    def test_invalid_ranges_retained_but_not_hits(self):
        scan = [float('inf'), float('nan'), 0.01, 13.0] + [2.0] * 356
        frame = self.capture(scan)
        self.assertTrue(math.isinf(frame.ranges_m[0]))
        self.assertTrue(math.isnan(frame.ranges_m[1]))
        self.assertEqual(len(frame.points_lidar_m), 356)
        scan[-1] = 1.0
        self.assertEqual(frame.ranges_m[-1], 2.0)

    def test_no_stale_or_malformed_frames(self):
        self.assertIsNone(self.sensors.capture(0.25, [2.0]*360, self.odom, False))
        self.assertIsNone(self.capture([2.0]*359))

    def test_no_hits_is_valid_scan(self):
        frame = self.capture([float('inf')]*360)
        self.assertEqual(frame.points_base_m, ())
        self.assertEqual(len(frame.ranges_m), 360)


if __name__ == '__main__':
    unittest.main()
