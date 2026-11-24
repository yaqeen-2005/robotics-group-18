"""Publish wheel odometry in the warehouse frame using a known starting pose."""

import json
import math
import os
from pathlib import Path
import tempfile

from .odometry import DifferentialDriveOdometry


class WarehouseLocalization:
    def __init__(self, radius, track, start):
        self.odometry = DifferentialDriveOdometry(radius, track)
        self.x0, self.y0, self.yaw0 = start
        if not all(math.isfinite(v) for v in start):
            raise ValueError("start pose must be finite")
        self.cos_yaw = math.cos(self.yaw0)
        self.sin_yaw = math.sin(self.yaw0)

    def update(self, time_s, angles):
        if not all(math.isfinite(value) for value in angles):
            return None
        x, y, yaw = self.odometry.update(*angles)
        return {
            "frame": "warehouse",
            "time_s": time_s,
            "x_m": self.x0 + self.cos_yaw * x - self.sin_yaw * y,
            "y_m": self.y0 + self.sin_yaw * x + self.cos_yaw * y,
            "yaw_rad": math.remainder(self.yaw0 + yaw, math.tau),
        }


class PosePublisher:
    def __init__(self, path, estimator, period=0.1):
        self.path = Path(path).resolve()
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self.estimator = estimator
        self.period = period
        self.next_publish = 0.0

    def update(self, time_s, angles):
        pose = self.estimator.update(time_s, angles)
        if pose is None or time_s + 1e-9 < self.next_publish:
            return
        while self.next_publish <= time_s + 1e-9:
            self.next_publish += self.period
        with tempfile.NamedTemporaryFile(mode="w", encoding="utf-8", delete=False,
                                         dir=self.path.parent, suffix=".tmp") as stream:
            staged = Path(stream.name)
            json.dump(pose, stream, allow_nan=False)
        try:
            os.replace(staged, self.path)
        except PermissionError:
            # A Windows reader can briefly block replacement; retry next period.
            pass
        finally:
            staged.unlink(missing_ok=True)


def open_pose_publisher(radius, track):
    path = os.environ.get("G18_LOCALIZATION_POSE")
    if not path:
        return None
    start = json.loads(os.environ["G18_LOCALIZATION_START"])
    return PosePublisher(path, WarehouseLocalization(radius, track, start))
