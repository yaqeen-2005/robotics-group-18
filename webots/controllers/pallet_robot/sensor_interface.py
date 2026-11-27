"""Single-step sensor observations for native Python SLAM (no ground truth)."""
import math
from dataclasses import dataclass
from typing import Tuple, Optional

Point = Tuple[float, float]


@dataclass(frozen=True)
class SensorFrame:
    time_s: float
    ranges_m: Tuple[float, ...]
    angles_rad: Tuple[float, ...]
    points_lidar_m: Tuple[Point, ...]
    odom_pose: Tuple[float, float, float]
    velocity: Tuple[float, float]
    lidar_offset_m: Point
    min_range_m: float
    max_range_m: float

    @property
    def points_base_m(self):
        """Obstacle endpoints relative to the drive axle, +x forward, +y left."""
        ox, oy = self.lidar_offset_m
        return tuple((x + ox, y + oy) for x, y in self.points_lidar_m)


class SensorInterface:
    def __init__(self, resolution, fov, min_range, max_range,
                 lidar_offset=(0.52, 0.0)):
        if resolution <= 0 or not 0 < fov <= 2 * math.pi:
            raise ValueError('Invalid scan geometry')
        if not (0 < min_range < max_range < math.inf):
            raise ValueError('Invalid scan limits')
        self.resolution = resolution
        self.angles = tuple(fov / 2 - (i + 0.5) * fov / resolution
                            for i in range(resolution))
        self.min_range = min_range
        self.max_range = max_range
        self.offset = tuple(lidar_offset)
        self._directions = tuple((math.cos(a), math.sin(a)) for a in self.angles)

    def capture(self, now, scan, odometry, odom_updated) -> Optional[SensorFrame]:
        """Return None if odometry was not updated or the scan is malformed.

        Time is the simulation step at which readings are collected, not a
        per-beam acquisition time. Raw nonfinite readings remain unchanged.
        """
        if not odom_updated or not math.isfinite(now):
            return None
        ranges = tuple(scan)
        if len(ranges) != self.resolution:
            return None
        pose = tuple(odometry.get_pose())
        velocity = tuple(odometry.get_velocity())
        if not all(math.isfinite(v) for v in pose + velocity):
            return None
        points = tuple((r * c, r * s)
                       for r, (c, s) in zip(ranges, self._directions)
                       if math.isfinite(r) and self.min_range <= r <= self.max_range)
        return SensorFrame(now, ranges, self.angles, points, pose, velocity,
                           self.offset, self.min_range, self.max_range)
