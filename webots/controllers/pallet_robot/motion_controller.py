"""Limit requested body velocities before conversion to wheel commands.

This module does not access Webots, detect cargo or perform navigation.
"""
import math
from cargo_profiles import CARGO_PROFILES


def _clamp(value, limit):
    return max(-limit, min(limit, value))


def _approach(current, target, amount):
    return current + _clamp(target - current, amount)


def _ramp(current, target, acceleration, deceleration, dt):
    # Brake to zero before reversing; use only remaining time to accelerate.
    if current * target < 0:
        stop_time = abs(current) / deceleration
        if dt <= stop_time:
            return _approach(current, 0.0, deceleration * dt)
        return _approach(0.0, target, acceleration * (dt - stop_time))
    rate = deceleration if abs(target) < abs(current) else acceleration
    return _approach(current, target, rate * dt)


class MotionController:
    def __init__(self, profile='empty'):
        self._profile_name = profile
        self._profile = CARGO_PROFILES[profile]
        self._v = 0.0
        self._yaw = 0.0

    @property
    def profile_name(self):
        return self._profile_name

    @property
    def command(self):
        return self._v, self._yaw

    def set_profile(self, name):
        """Only switch at zero commanded motion; caller must verify physical stop."""
        profile = CARGO_PROFILES[name]
        if name == self._profile_name:
            return
        if self.command != (0.0, 0.0):
            raise RuntimeError('Stop before changing cargo profile')
        self._profile_name = name
        self._profile = profile

    def stop(self):
        """Immediate zero command for safety overrides, not normal braking."""
        self._v = self._yaw = 0.0
        return self.command

    def update(self, requested_v, requested_yaw, dt):
        """Return limited (m/s, rad/s); dt is simulation seconds."""
        if not all(math.isfinite(x) for x in (requested_v, requested_yaw, dt)) or dt <= 0:
            raise ValueError('Finite velocity requests and positive dt required')
        p = self._profile
        target_v = _clamp(requested_v, p.max_speed)
        target_yaw = _clamp(requested_yaw, p.max_turn_speed)
        self._v = _ramp(self._v, target_v, p.max_acceleration, p.max_deceleration, dt)
        self._yaw = _ramp(self._yaw, target_yaw, p.max_angular_acceleration,
                          p.max_angular_acceleration, dt)
        return self.command
