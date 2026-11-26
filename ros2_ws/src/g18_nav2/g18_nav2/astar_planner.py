"""Serve Nav2 path requests with Yi's A* (ros2_ws/src/navigation/astar).

Nav2 planner plugins must be C++, but the behaviour tree's ComputePathToPose
node can call any action server by name. This node is that server: it turns
the live global costmap into the grid and cell attributes Yi's
``plan_path(grid, start, goal, attributes, weights)`` expects, plans, and
returns a nav_msgs/Path.

    ros2 launch g18_nav2 nav2.launch.py planner:=astar     # starts this node and the A* behaviour tree

Grid conversion (cell size ``cell_size``, cropped around start and goal):
  * blocked (100): closer to an obstacle than the current footprint's half-width
    plus ``clearance_padding`` -- so a wide pallet cannot use a narrow door;
  * unknown cells are traversable with confidence 0, so Yi's "uncertainty"
    weight makes the robot prefer explored space without refusing to explore;
  * the costmap's inflation becomes the blockage probability, and cells closer
    to an obstacle than the body's corner radius get blockage 1: still allowed
    (a doorway may need it) but expensive, so wide aisles are crossed in the middle;
  * the 4-connected staircase is shortcut wherever a straight line keeps the
    corner radius clear, so the controller gets diagonal segments, not 90 deg steps.
"""
import math
import os
import sys
import time
from pathlib import Path as FsPath

import numpy as np
import rclpy
from geometry_msgs.msg import PolygonStamped, PoseStamped
from nav2_msgs.action import ComputePathToPose
from nav_msgs.msg import OccupancyGrid, Path
from rclpy.action import ActionServer
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from scipy import ndimage
from tf2_ros import Buffer, TransformListener


def find_navigation_dir(override=''):
    """Locate Yi's navigation folder (it is plain Python, not a ROS package)."""
    candidates = [override, os.environ.get('G18_NAVIGATION', '')]
    for parent in FsPath(os.path.realpath(__file__)).parents:   # works with --symlink-install
        candidates.append(str(parent / 'ros2_ws' / 'src' / 'navigation'))
        candidates.append(str(parent / 'navigation'))
    for c in candidates:
        if c and (FsPath(c) / 'astar' / 'a_star.py').is_file():
            return c
    raise RuntimeError('Cannot find ros2_ws/src/navigation (Yi\'s A*); '
                       'set -p navigation_path:=... or G18_NAVIGATION')


class NoPathInWindow(RuntimeError):
    pass


def yaw_quaternion(yaw):
    return math.sin(yaw / 2.0), math.cos(yaw / 2.0)


class AStarPlanner(Node):
    def __init__(self):
        super().__init__('astar_planner')
        self.cell = self.declare_parameter('cell_size', 0.2).value             # m per A* cell
        self.margin = self.declare_parameter('crop_margin', 12.0).value        # m around start/goal
        self.padding = self.declare_parameter('clearance_padding', 0.1).value  # m beyond half-width
        self.goal_snap = self.declare_parameter('goal_snap_radius', 1.0).value
        nav_dir = find_navigation_dir(self.declare_parameter('navigation_path', '').value)
        sys.path.insert(0, nav_dir)
        from astar.a_star import plan_path                     # noqa: E402  (Yi's code)
        from common.cell_attributes import CellAttributeMap    # noqa: E402
        self.plan_path, self.CellAttributeMap = plan_path, CellAttributeMap
        self.get_logger().info(f"Using Yi's A* from {nav_dir}")

        self.costmap = None
        self.half_width = 0.47
        self.corner_radius = 0.73
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(OccupancyGrid, '/global_costmap/costmap', self.on_costmap, latched)
        self.create_subscription(PolygonStamped, '/global_costmap/published_footprint',
                                 self.on_footprint, 10)
        self.tf = Buffer()
        TransformListener(self.tf, self)
        self.path_pub = self.create_publisher(Path, '/astar/plan', 1)
        ActionServer(self, ComputePathToPose, 'astar/compute_path_to_pose', self.execute,
                     callback_group=ReentrantCallbackGroup())

    # ---------------------------------------------------------------- inputs
    def on_costmap(self, msg):
        self.costmap = msg

    def on_footprint(self, msg):
        # published_footprint is the padded outline in the map frame, in the order
        # cargo_manager and the params give it: front-left, front-right, rear-right,
        # rear-left. The first edge is therefore the width across the robot. (Not
        # the shorter side: a wide pallet is wider than it is long.)
        pts = np.array([[p.x, p.y] for p in msg.polygon.points])
        if len(pts) != 4:
            return
        width = float(np.linalg.norm(pts[1] - pts[0]))
        length = float(np.linalg.norm(pts[2] - pts[1]))
        self.half_width = max(width / 2.0 - 0.05, 0.3)   # minus the costmap's footprint_padding
        # Radius swept by the corners when turning (base_link is near the centre).
        self.corner_radius = max(math.hypot(width / 2.0, length / 2.0) - 0.05, self.half_width)

    # ---------------------------------------------------------------- planning
    def robot_pose(self):
        t = self.tf.lookup_transform('map', 'base_link', rclpy.time.Time())
        return t.transform.translation.x, t.transform.translation.y

    def execute(self, handle):
        t0 = time.monotonic()
        req = handle.request
        result = ComputePathToPose.Result()
        try:
            if self.costmap is None:
                raise RuntimeError('no global costmap yet')
            sx, sy = ((req.start.pose.position.x, req.start.pose.position.y)
                      if req.use_start else self.robot_pose())
            gx, gy = req.goal.pose.position.x, req.goal.pose.position.y
            # Plan in a small window first (fast); if the way round is longer than
            # the window (e.g. a wide pallet must use a door 12 m away), widen it.
            for margin in (self.margin, 30.0, 1000.0):
                try:
                    path = self.plan(sx, sy, gx, gy, req.goal, margin)
                    break
                except NoPathInWindow:
                    if margin == 1000.0:
                        raise
        except Exception as e:  # report to the behaviour tree instead of crashing
            self.get_logger().warn(f'A* failed: {e}')
            handle.abort()
            result.error_code = ComputePathToPose.Result.NO_VALID_PATH
            return result
        result.path = path
        result.planning_time.nanosec = int((time.monotonic() - t0) * 1e9)
        self.path_pub.publish(path)
        handle.succeed()
        return result

    def plan(self, sx, sy, gx, gy, goal_pose, margin):
        cm = self.costmap
        res, ox, oy = cm.info.resolution, cm.info.origin.position.x, cm.info.origin.position.y
        full = np.array(cm.data, dtype=np.int16).reshape(cm.info.height, cm.info.width)

        # Crop a window around start and goal, in costmap cells.
        lo_x = int((min(sx, gx) - margin - ox) / res)
        hi_x = int((max(sx, gx) + margin - ox) / res)
        lo_y = int((min(sy, gy) - margin - oy) / res)
        hi_y = int((max(sy, gy) + margin - oy) / res)
        lo_x, lo_y = max(lo_x, 0), max(lo_y, 0)
        hi_x, hi_y = min(hi_x, cm.info.width), min(hi_y, cm.info.height)
        win = full[lo_y:hi_y, lo_x:hi_x]

        # Downsample by k: a coarse cell takes the worst (max) fine cost; it is
        # unknown only if all its fine cells are unknown.
        k = max(1, int(round(self.cell / res)))
        h, w = win.shape[0] // k, win.shape[1] // k
        win = win[:h * k, :w * k]
        blocks = win.reshape(h, k, w, k).swapaxes(1, 2).reshape(h, w, k * k)
        unknown = (blocks < 0).all(axis=2)
        cost = blocks.max(axis=2).clip(min=0)
        lethal = cost >= 100
        cell = res * k

        # Cells closer to an obstacle than the body (or pallet) half-width are blocked.
        clearance = self.half_width + self.padding
        dist = ndimage.distance_transform_edt(~lethal) * cell
        # dist is measured between coarse cell centres, so the real gap can be up to
        # half a cell narrower. Without this margin a 1.95 m doorway (CART) looked
        # wide enough for the 1.8 m pallet, which needs 2.0 m.
        quant = cell / 2.0
        blocked = dist < clearance + quant
        tight = dist < self.corner_radius + self.padding + quant   # corners could touch when turning

        def to_cell(x, y):
            return int((x - ox) / res - lo_x) // k, int((y - oy) / res - lo_y) // k

        def free_near(c, radius):
            """Nearest unblocked cell to c within radius (m), else None."""
            cx, cy = c
            r = int(math.ceil(radius / cell))
            best = None
            for dy in range(-r, r + 1):
                for dx in range(-r, r + 1):
                    x, y = cx + dx, cy + dy
                    if 0 <= x < w and 0 <= y < h and not blocked[y, x]:
                        d = dx * dx + dy * dy
                        if best is None or d < best[0]:
                            best = (d, (x, y))
            return best[1] if best else None

        start = free_near(to_cell(sx, sy), 1.0)        # the robot may sit inside the margin
        goal = free_near(to_cell(gx, gy), self.goal_snap)
        if start is None or goal is None:
            raise RuntimeError('start or goal is inside an obstacle')

        grid = np.where(blocked, 100, 0).astype(int).tolist()
        confidence = np.where(unknown, 0.0, 1.0)
        blockage = np.where(blocked | unknown, 0.0, np.maximum(cost / 100.0, tight * 1.0))
        values = np.stack([confidence, blockage, np.zeros_like(blockage), np.zeros_like(blockage)],
                          axis=2).tolist()
        attributes = self.CellAttributeMap(w, h, values)

        t0 = time.monotonic()
        cells = self.plan_path(grid, start, goal, attributes)
        if not cells:
            raise NoPathInWindow(f'no path within {margin:.0f} m of start and goal')
        self.get_logger().info(f'A*: {len(cells)} cells on a {w}x{h} grid in '
                               f'{time.monotonic() - t0:.2f} s (clearance {clearance:.2f} m, '
                               f'corner radius {self.corner_radius:.2f} m)')
        cells = self.shortcut(cells, ~tight & ~unknown, ~blocked & ~unknown)
        return self.to_path(cells, ox + lo_x * res, oy + lo_y * res, cell, goal_pose)

    @staticmethod
    def shortcut(cells, roomy, passable):
        """Greedy line-of-sight smoothing: from each kept cell jump to the farthest
        later cell whose straight segment stays on `roomy` cells (corners clear).
        Where no roomy segment exists (a corridor narrower than the corner radius),
        fall back to `passable` cells (body clear) so no 0.2 m staircase is left."""
        def clear(a, b, ok):
            n = max(abs(b[0] - a[0]), abs(b[1] - a[1]), 1) * 2
            for i in range(n + 1):
                x = round(a[0] + (b[0] - a[0]) * i / n)
                y = round(a[1] + (b[1] - a[1]) * i / n)
                if not ok[y, x]:
                    return False
            return True

        def farthest(i, ok):
            j = len(cells) - 1
            while j > i + 1 and not clear(cells[i], cells[j], ok):
                j -= 1
            return j

        out, i = [cells[0]], 0
        while i < len(cells) - 1:
            j = farthest(i, roomy)
            if j == i + 1:                 # no roomy segment: accept body clearance
                j = farthest(i, passable)
            out.append(cells[j])
            i = j
        return out

    def to_path(self, cells, x0, y0, cell, goal_pose):
        # Cells are now the waypoints of straight segments; resample every 0.1 m so
        # the controller gets a dense path, and end exactly at the goal.
        corners = [(x0 + (c[0] + 0.5) * cell, y0 + (c[1] + 0.5) * cell) for c in cells]
        corners[-1] = (goal_pose.pose.position.x, goal_pose.pose.position.y)

        path = Path()
        path.header.frame_id = 'map'
        path.header.stamp = self.get_clock().now().to_msg()
        for (ax, ay), (bx, by) in zip(corners, corners[1:]):
            seg = math.hypot(bx - ax, by - ay)
            yaw = math.atan2(by - ay, bx - ax)
            for i in range(max(int(seg / 0.1), 1)):
                ps = PoseStamped()
                ps.header = path.header
                ps.pose.position.x = ax + (bx - ax) * i * 0.1 / max(seg, 1e-9)
                ps.pose.position.y = ay + (by - ay) * i * 0.1 / max(seg, 1e-9)
                ps.pose.orientation.z, ps.pose.orientation.w = yaw_quaternion(yaw)
                path.poses.append(ps)
        end = PoseStamped()
        end.header = path.header
        end.pose = goal_pose.pose
        path.poses.append(end)
        return path


def main():
    rclpy.init()
    node = AStarPlanner()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
