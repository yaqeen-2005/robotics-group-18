import math
from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'controllers' / 'pallet_robot'))
from cargo_profiles import CARGO_PROFILES, CargoProfile
from motion_controller import MotionController, _ramp


class MotionTests(unittest.TestCase):
    def test_profile_limits(self):
        for name, p in CARGO_PROFILES.items():
            for sign in (-1, 1):
                with self.subTest(profile=name, sign=sign):
                    m = MotionController(name)
                    for _ in range(100):
                        v, w = m.update(sign * 10, sign * 10, 0.032)
                    self.assertAlmostEqual(v, sign * p.max_speed)
                    self.assertAlmostEqual(w, sign * p.max_turn_speed)

    def test_acceleration(self):
        for name, p in CARGO_PROFILES.items():
            m = MotionController(name)
            v, w = m.update(10, 10, 0.1)
            self.assertAlmostEqual(v, p.max_acceleration * 0.1)
            self.assertAlmostEqual(w, p.max_angular_acceleration * 0.1)

    def test_normal_deceleration(self):
        m = MotionController('fragile')
        m.update(10, 10, 5)
        v, w = m.update(0, 0, 0.1)
        self.assertAlmostEqual(v, 0.235)
        self.assertAlmostEqual(w, 0.17)
        self.assertEqual(m.update(0, 0, 5), (0, 0))

    def test_reverse(self):
        m = MotionController()
        m.update(10, 0, 5)
        self.assertAlmostEqual(m.update(-10, 0, 0.5)[0], 0.35)
        self.assertAlmostEqual(m.update(-10, 0, 1)[0], -0.15)

    def test_asymmetric_reversal(self):
        self.assertAlmostEqual(_ramp(0.4, -0.6, 0.2, 0.4, 1.5), -0.1)

    def test_switch_at_rest(self):
        m = MotionController()
        m.set_profile('fragile')
        self.assertEqual(m.profile_name, 'fragile')
        self.assertEqual(m.update(10, 10, 5), (0.25, 0.2))

    def test_reject_moving_switch(self):
        m = MotionController()
        m.update(1, 0, 0.1)
        with self.assertRaises(RuntimeError):
            m.set_profile('wide')
        self.assertEqual(m.profile_name, 'empty')

    def test_emergency_stop(self):
        m = MotionController()
        m.update(1, 1, 5)
        self.assertEqual(m.stop(), (0, 0))
        self.assertEqual(m.command, (0, 0))
        self.assertAlmostEqual(m.update(1, 0, 0.1)[0], 0.05)

    def test_invalid_inputs(self):
        m = MotionController()
        for args in [(math.nan, 0, 1), (0, math.inf, 1), (0, 0, 0), (0, 0, -1), (0, 0, math.nan)]:
            with self.assertRaises(ValueError):
                m.update(*args)
        with self.assertRaises(KeyError):
            m.set_profile('unknown')

    def test_invalid_profile(self):
        with self.assertRaises(ValueError):
            CargoProfile(0.6, 0.0, 0.5, 0.5, 1)


if __name__ == '__main__':
    unittest.main()
