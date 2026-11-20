"""Opt-in recording of Webots SLAM inputs."""

import json
import math
import os
from pathlib import Path


def finite_or_none(value):
    return value if math.isfinite(value) else None


class SensorLog:
    def __init__(self, path, lidar, step_ms, radius, track):
        self.lidar = lidar
        self.stream = path.open("x", encoding="utf-8", buffering=1)
        self.write({
            "schema": "g18-slam-sensors-v1",
            "sampling_period_ms": step_ms,
            "wheel_radius_m": radius,
            "wheel_track_m": track,
            "lidar": {
                "name": "front_lidar",
                "horizontal_resolution": lidar.getHorizontalResolution(),
                "horizontal_fov_rad": lidar.getFov(),
                "min_range_m": lidar.getMinRange(),
                "max_range_m": lidar.getMaxRange(),
                "translation_m": [0.52, 0.0, 0.13],  # PalletRobot.proto
                "yaw_rad": 0.0,
            },
        })

    def write(self, row):
        self.stream.write(json.dumps(row, allow_nan=False, separators=(",", ":")) + "\n")

    def record(self, timestamp, angles):
        self.write({
            "time_s": timestamp,
            "wheel_angles_rad": [finite_or_none(angle) for angle in angles],
            "ranges_m": [finite_or_none(value) for value in self.lidar.getRangeImage()],
        })


def open_sensor_log(robot, step_ms, radius, track):
    filename = os.environ.get("G18_SLAM_SENSOR_LOG")
    if not filename:
        return None
    path = Path(filename).resolve()
    path.parent.mkdir(parents=True, exist_ok=True)
    lidar = robot.getDevice("front_lidar")
    lidar.enable(step_ms)
    log = SensorLog(path, lidar, step_ms, radius, track)
    print(f"SLAM sensor log: {path}", flush=True)
    return log
