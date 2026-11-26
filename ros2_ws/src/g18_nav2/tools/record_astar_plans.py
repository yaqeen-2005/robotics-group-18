#!/usr/bin/env python3
"""Record every A* plan for later analysis (planner:=astar).

Writes one JSON line per /astar/plan message: sim time, robot position and the
path's corner points, all in warehouse coordinates. Useful to see which divider
crossing A* chose over time and whether the route flips between alternatives.

    python3 src/g18_nav2/tools/record_astar_plans.py plans.jsonl     # Ctrl+C to stop
    python3 src/g18_nav2/tools/record_astar_plans.py --summary plans.jsonl

The map frame is the robot's pose when SLAM started (the parking bay); the
conversion uses the same start pose as go_to.
"""
import argparse
import json
import math

DOORS = {'WEST': -37, 'G01': -22, 'G02': -12, 'SHORT': -4, 'CART': 3,
         'BULK': 8.5, 'G03': 14, 'G04': 30, 'EAST': 42}   # x of each divider crossing (y = 0)
START = (-38.0, -3.3, -90.0)                              # parking bay, as in go_to


def to_warehouse(x, y):
    sx, sy, syaw = START[0], START[1], math.radians(START[2])
    c, s = math.cos(syaw), math.sin(syaw)
    return round(sx + c * x - s * y, 2), round(sy + s * x + c * y, 2)


def crossing(corners):
    """Name of the divider crossing a path uses, or 'none'."""
    for (ax, ay), (bx, by) in zip(corners, corners[1:]):
        if (ay < 0) != (by < 0):
            x = ax + (bx - ax) * ay / (ay - by)
            return min(DOORS, key=lambda k: abs(DOORS[k] - x))
    return 'none'


def record(out_path):
    import rclpy
    from nav_msgs.msg import Path
    from rclpy.executors import ExternalShutdownException
    from rclpy.parameter import Parameter
    from tf2_ros import Buffer, TransformListener

    rclpy.init()
    node = rclpy.create_node('astar_plan_recorder',
                             parameter_overrides=[Parameter('use_sim_time', value=True)])
    tf = Buffer()
    TransformListener(tf, node)
    out = open(out_path, 'w')

    def on_plan(path):
        corners, prev = [], None
        for p in path.poses:                       # a corner is where the heading changes
            yaw = round(2 * math.atan2(p.pose.orientation.z, p.pose.orientation.w), 2)
            if yaw != prev:
                corners.append(to_warehouse(p.pose.position.x, p.pose.position.y))
                prev = yaw
        end = path.poses[-1].pose.position
        corners.append(to_warehouse(end.x, end.y))
        try:
            t = tf.lookup_transform('map', 'base_link', rclpy.time.Time()).transform.translation
            robot = to_warehouse(t.x, t.y)
        except Exception:                          # TF not ready yet
            robot = None
        out.write(json.dumps({'t': node.get_clock().now().nanoseconds / 1e9,
                              'robot': robot, 'corners': corners}) + '\n')
        out.flush()

    node.create_subscription(Path, '/astar/plan', on_plan, 10)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        out.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def summary(path):
    """Print each change of chosen crossing, then totals."""
    plans, changes, last = 0, [], None
    for line in open(path):
        try:
            d = json.loads(line)
        except ValueError:                         # a line cut off when recording stopped
            continue
        plans += 1
        door = crossing(d['corners'])
        if door != last:
            changes.append((d['t'], door, d['robot']))
            last = door
    for t, door, robot in changes:
        print(f't={t:7.1f} s  crossing={door:6s} robot={robot}')
    print(f'{plans} plans, {len(changes) - 1} route changes')


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('file')
    ap.add_argument('--summary', action='store_true', help='analyse a recorded file instead of recording')
    a = ap.parse_args()
    summary(a.file) if a.summary else record(a.file)


if __name__ == '__main__':
    main()
