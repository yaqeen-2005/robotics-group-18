"""Differential-drive wheel odometry in a local frame at the starting axle pose."""
import math


class WheelOdometry:
    def __init__(self, wheel_radius, wheel_track):
        if not (math.isfinite(wheel_radius) and wheel_radius > 0 and
                math.isfinite(wheel_track) and wheel_track > 0):
            raise ValueError("Wheel radius and track must be positive and finite")
        self.wheel_radius = wheel_radius
        self.wheel_track = wheel_track
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.linear_velocity = 0.0
        self.angular_velocity = 0.0
        self._previous = None

    def update(self, left_angle, right_angle, now):
        """Accept cumulative wheel angles (rad) and simulation time (s).

        Return True when a new motion estimate is integrated. Invalid samples
        are skipped; the next valid update spans the gap. Velocities describe
        the last successfully integrated interval.
        """
        if not all(math.isfinite(v) for v in (left_angle, right_angle, now)):
            return False
        if self._previous is None:
            self._previous = (left_angle, right_angle, now)
            return False

        old_left, old_right, old_time = self._previous
        dt = now - old_time
        if dt <= 0:
            return False

        dl = self.wheel_radius * (left_angle - old_left)
        dr = self.wheel_radius * (right_angle - old_right)
        ds = (dl + dr) / 2.0
        dtheta = (dr - dl) / self.wheel_track

        # Exact constant-curvature integration, stable for straight motion.
        half = dtheta / 2.0
        scale = math.sin(half) / half if abs(half) > 1e-8 else 1.0
        self.x += ds * scale * math.cos(self.theta + half)
        self.y += ds * scale * math.sin(self.theta + half)
        self.theta = math.atan2(math.sin(self.theta + dtheta),
                                math.cos(self.theta + dtheta))
        self.linear_velocity = ds / dt
        self.angular_velocity = dtheta / dt
        self._previous = (left_angle, right_angle, now)
        return True

    def get_pose(self):
        """Return (x, y, heading) in metres and radians, relative to start."""
        return self.x, self.y, self.theta

    def get_velocity(self):
        """Return (linear m/s, angular rad/s) for the last valid interval."""
        return self.linear_velocity, self.angular_velocity
