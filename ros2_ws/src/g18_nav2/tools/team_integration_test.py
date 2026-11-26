#!/usr/bin/env python3
"""Team integration test, run against the live simulation (planner:=astar).

1. Yaqeen's LidarSensor.get_points (webots/controllers/pallet_robot/lidar.py) is
   fed the real Webots ranges from /scan_raw and compared with the points ROS
   computes from the same LaserScan (angle_min + i * angle_increment).
2. Exploration: Xu's find_frontiers (webots/exploration/frontier.py) on the live
   slam_toolbox /map picks a frontier; go_to sends it to Nav2, which plans with
   Yi's A* (astar_planner). Repeated for N rounds; known map area should grow.

Start the simulator and nav2.launch.py planner:=astar first, then:

    python3 src/g18_nav2/tools/team_integration_test.py [rounds, default 4]
"""
import math, subprocess, sys, time
from pathlib import Path
import numpy as np
import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from scipy import ndimage
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformListener

REPO = str(Path(__file__).resolve().parents[4])   # repository root
sys.path.insert(0, f'{REPO}/webots/exploration')
sys.path.insert(0, f'{REPO}/webots/controllers/pallet_robot')
from frontier import find_frontiers          # Xu
from lidar import LidarSensor                # Yaqeen

rclpy.init()
node = rclpy.create_node('team_test', parameter_overrides=[rclpy.parameter.Parameter('use_sim_time', value=True)])
tf = Buffer(); TransformListener(tf, node)
latest = {}
node.create_subscription(OccupancyGrid, '/map', lambda m: latest.__setitem__('map', m),
                         QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE))
node.create_subscription(LaserScan, '/scan_raw', lambda m: latest.__setitem__('scan', m), qos_profile_sensor_data)


def wait_for(key, timeout=20):
    latest.pop(key, None)
    end = time.monotonic() + timeout
    while key not in latest and time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.2)
    return latest.get(key)


def robot_xy():
    for _ in range(50):
        rclpy.spin_once(node, timeout_sec=0.1)
        try:
            t = tf.lookup_transform('map', 'base_link', rclpy.time.Time()).transform.translation
            return t.x, t.y
        except Exception:
            pass
    return None


# ---------------------------------------------------------------- 1. LiDAR (Yaqeen)
print('== 1. Yaqeen LidarSensor.get_points vs ROS LaserScan (/scan_raw, same Webots ranges)', flush=True)
scan = wait_for('scan')
class FakeDevice:                 # stands in for the Webots Lidar device with the real scan
    def getHorizontalResolution(self): return len(scan.ranges)
    def getFov(self): return abs(scan.angle_max - scan.angle_min) + abs(scan.angle_increment)
    def enable(self, ms): pass
    def getRangeImage(self): return list(scan.ranges)
class FakeRobot:
    def getDevice(self, name): assert name == 'front_lidar'; return FakeDevice()
ls = LidarSensor(FakeRobot(), 32)
ranges = ls.get_scan()
mine = [(i, r) for i, r in enumerate(ranges) if math.isfinite(r)]
pts = ls.get_points(ranges)
ros = [(r * math.cos(scan.angle_min + i * scan.angle_increment), r * math.sin(scan.angle_min + i * scan.angle_increment)) for i, r in mine]
err = max(math.dist(a, b) for a, b in zip(pts, ros))
print(f'   rays {len(ranges)}, finite {len(pts)}, FOV {math.degrees(ls.fov):.1f} deg, '
      f'ROS angle_min {math.degrees(scan.angle_min):.2f} inc {math.degrees(scan.angle_increment):.3f} deg')
print(f'   max point difference: {err:.3f} m  (half a ray at 10 m = {10 * abs(scan.angle_increment) / 2:.3f} m)')
print(f'   LIDAR: {"PASS" if err < 10 * abs(scan.angle_increment) else "FAIL"}', flush=True)

# ---------------------------------------------------------------- 2. exploration (Xu + Yi + Sirui)
print('== 2. Frontier exploration: Xu find_frontiers -> Yi A* (planner:=astar) -> Sirui Nav2', flush=True)
visited = []
rxy = robot_xy() or (0.0, 0.0)
for rnd in range(1, int(sys.argv[1]) + 1 if len(sys.argv) > 1 else 4):
    m = wait_for('map')
    w, h, res = m.info.width, m.info.height, m.info.resolution
    ox, oy = m.info.origin.position.x, m.info.origin.position.y
    raw = np.array(m.data, dtype=np.int16).reshape(h, w)
    known = int((raw >= 0).sum()) * res * res
    # Xu's function accepts only -1 / 0 / 100: classify SLAM probabilities explicitly.
    occ = raw >= 65
    # keep frontier targets away from walls: a free cell near an obstacle is "occupied" here
    near_wall = ndimage.distance_transform_edt(~occ) * res < 0.8
    grid = np.where(raw < 0, -1, np.where(occ | near_wall, 100, 0))
    t0 = time.monotonic(); cells = find_frontiers(grid.tolist()); dt = time.monotonic() - t0
    mask = np.zeros((h, w), bool)
    for x, y in cells:
        mask[y, x] = True
    labels, n = ndimage.label(mask, structure=np.ones((3, 3)))
    clusters = []
    for k in range(1, n + 1):
        ys, xs = np.nonzero(labels == k)
        if len(xs) * res < 1.0:            # ignore frontiers shorter than 1 m
            continue
        i = np.argmin((xs - xs.mean()) ** 2 + (ys - ys.mean()) ** 2)   # frontier cell nearest the centroid
        gx, gy = ox + (xs[i] + 0.5) * res, oy + (ys[i] + 0.5) * res
        if any(math.dist((gx, gy), v) < 2.0 for v in visited):
            continue
        clusters.append((len(xs) * res, gx, gy))
    print(f'   round {rnd}: map {w}x{h} known {known:.0f} m2, {len(cells)} frontier cells in {dt:.2f} s, '
          f'{len(clusters)} frontiers >= 1 m, robot {tuple(round(v, 1) for v in rxy)}', flush=True)
    if not clusters:
        print('   no frontier left to try'); break
    # nearest big frontier (simple greedy exploration)
    size, gx, gy = min(clusters, key=lambda c: math.dist((c[1], c[2]), rxy) - 0.5 * c[0])
    visited.append((gx, gy))
    t = time.monotonic()
    out = subprocess.run(['ros2', 'run', 'g18_nav2', 'go_to', '--', f'{gx:.2f}', f'{gy:.2f}'],
                         stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, timeout=300).stdout
    res_txt = 'SUCCEEDED' if 'SUCCEEDED' in out else ('ABORTED' if 'ABORTED' in out else 'FAILED')
    at = [l.split('at (')[1].split(')')[0] for l in out.splitlines() if 'at (' in l]
    if at:
        rxy = tuple(float(v) for v in at[-1].split(','))
    rec = [l.split('recoveries')[1].split()[0] for l in out.splitlines() if 'recoveries' in l]
    print(f'     goal frontier ({gx:.1f}, {gy:.1f}) map frame, length {size:.1f} m -> {res_txt}, '
          f'{time.monotonic() - t:.0f} s wall, recoveries {rec[-1] if rec else "?"}', flush=True)
m = wait_for('map')
raw = np.array(m.data, dtype=np.int16)
print(f'   final known area {int((raw >= 0).sum()) * m.info.resolution ** 2:.0f} m2', flush=True)
