"""Incremental LiDAR mapping for the final Webots warehouse."""
import json
import math
import os
from pathlib import Path

from .mapping import OccupancyMapper
from .odometry import DifferentialDriveOdometry


class LiveMapper:
    def __init__(self, lidar, radius, track, start, output, stations):
        self.odom = DifferentialDriveOdometry(radius, track)
        self.grid = OccupancyMapper(dict(translation_m=[.52, 0, .13], yaw_rad=0,
            min_range_m=lidar.getMinRange(), max_range_m=lidar.getMaxRange(),
            horizontal_resolution=lidar.getHorizontalResolution(),
            horizontal_fov_rad=lidar.getFov()), .10)
        self.lidar = lidar
        self.pose = tuple(start)
        self.previous = None
        self.output = Path(output)
        self.output.parent.mkdir(parents=True, exist_ok=True)
        self.stations = stations
        self.frames = 0
        self.last_write = -math.inf
        self.trajectory = []

    def align(self, predicted, ranges):
        # Search near the wheel prediction using earlier scan endpoints as landmarks.
        g = self.grid
        points = []
        for a, r in zip(g.bearings[::4], ranges[::4]):
            if g.min_range < r < g.max_range:
                points.append((g.offset_x+r*math.cos(a), g.offset_y+r*math.sin(a)))
        if len(points) < 12 or not g.evidence:
            return predicted
        best = predicted
        for xy_step, angle_step in ((.06, .018), (.025, .007)):
            cx, cy, ca = best
            best_score = -math.inf
            for da in (0, -angle_step, angle_step):
                a = ca + da
                c, s = math.cos(a), math.sin(a)
                rotated = [(c*x-s*y, s*x+c*y) for x,y in points]
                for dx in (0, -xy_step, xy_step):
                    for dy in (0, -xy_step, xy_step):
                        x, y = cx+dx, cy+dy
                        score = sum(max(0, g.evidence.get(g.cell(x+u,y+v),0))
                                    for u,v in rotated) / len(points)
                        score -= 4*math.hypot(x-predicted[0],y-predicted[1])+12*abs(a-predicted[2])
                        if score > best_score:
                            best_score, best = score, (x,y,a)
        return best

    def update(self, timestamp, angles):
        if not all(math.isfinite(v) for v in angles):
            return
        current = self.odom.update(*angles)
        if self.previous is None:
            self.previous = current
        old_x, old_y, old_yaw = self.previous
        dx, dy = current[0]-old_x, current[1]-old_y
        turn = math.remainder(current[2]-old_yaw, math.tau)
        if self.frames and math.hypot(dx,dy)<.08 and abs(turn)<.025:
            return
        delta = self.pose[2]-old_yaw
        c,s = math.cos(delta), math.sin(delta)
        predicted = (self.pose[0]+c*dx-s*dy, self.pose[1]+s*dx+c*dy, self.pose[2]+turn)
        ranges = self.lidar.getRangeImage()
        if not any(self.grid.min_range <= v < self.grid.max_range for v in ranges):
            return
        self.pose = self.align(predicted, ranges)
        self.grid.update(self.pose, ranges)
        self.previous = current
        self.frames += 1
        self.trajectory.append((timestamp,*self.pose))
        if timestamp-self.last_write >= 1:
            self.save()
            self.last_write = timestamp

    def save(self):
        if not self.frames:
            return
        data = self.grid.snapshot(self.trajectory[-1], frame="warehouse")
        data.update(stations=self.stations, scan_count=self.frames, trajectory=self.trajectory)
        tmp = self.output.with_suffix(self.output.suffix+".tmp")
        tmp.write_text(json.dumps(data,separators=(",",":")),encoding="utf-8")
        os.replace(tmp,self.output)


def start_mapper(robot, step, radius, track, root, output):
    config = json.loads((root/"config/final_warehouse.json").read_text())
    dispatch = config["dispatch"]
    stations = {"AMR01":config["parking"], **{f"P{i:02}":[dispatch["x"],y]
                for i,y in enumerate(dispatch["lanes_y"],1)},
                **{d["id"]:d["at"] for d in config["docks"]}}
    lidar = robot.getDevice("front_lidar")
    lidar.enable(step)
    start = json.loads(os.environ.get("G18_LOCALIZATION_START",json.dumps([*config["parking"],-math.pi/2])))
    return LiveMapper(lidar,radius,track,start,output,stations)
