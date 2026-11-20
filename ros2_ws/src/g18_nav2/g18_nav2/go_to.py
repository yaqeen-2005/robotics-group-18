"""Send one Nav2 goal and report progress.

    ros2 run g18_nav2 go_to 10 -5            # x y in the SLAM map frame (start = 0,0, facing +x)
    ros2 run g18_nav2 go_to -20 -12 --warehouse   # x y in warehouse coordinates

Warehouse coordinates are the facility's ENU frame used in /warehouse/status
and FINAL_WAREHOUSE.md. They are converted with the robot's known start pose
(parking bay, x=-38, y=-3.3, facing south); override with --start X Y YAW_DEG.
"""
import argparse
import math
import sys
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener

START = (-38.0, -3.3, -90.0)


def warehouse_to_map(x, y, yaw, start):
    sx, sy, syaw = start[0], start[1], math.radians(start[2])
    dx, dy = x - sx, y - sy
    c, s = math.cos(-syaw), math.sin(-syaw)
    return c * dx - s * dy, s * dx + c * dy, yaw - syaw


def travel_heading(node, x, y):
    """Heading from the robot's current map pose to (x, y); 0 if TF is unavailable."""
    buf = Buffer()
    TransformListener(buf, node)
    end = time.monotonic() + 5.0          # wall time: sim time may not have started
    while time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.1)
        try:
            t = buf.lookup_transform('map', 'base_link', rclpy.time.Time())
            return math.atan2(y - t.transform.translation.y, x - t.transform.translation.x)
        except Exception:
            continue
    node.get_logger().warn('No map->base_link transform; using heading 0')
    return 0.0


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('x', type=float)
    ap.add_argument('y', type=float)
    ap.add_argument('yaw', type=float, nargs='?', default=None, help='degrees; default: face along the way')
    ap.add_argument('--warehouse', action='store_true', help='x y yaw are warehouse coordinates')
    ap.add_argument('--start', type=float, nargs=3, default=START, metavar=('X', 'Y', 'YAW_DEG'))
    args = ap.parse_args(rclpy.utilities.remove_ros_args(sys.argv)[1:])

    rclpy.init()
    node = Node('go_to', parameter_overrides=[rclpy.parameter.Parameter('use_sim_time', value=True)])
    yaw = math.radians(args.yaw) if args.yaw is not None else None
    if args.warehouse:
        x, y, yaw_m = warehouse_to_map(args.x, args.y, yaw or 0.0, args.start)
        yaw = yaw_m if args.yaw is not None else None
    else:
        x, y = args.x, args.y
    if yaw is None:
        # Arrive facing along the way in: the planner only drives forward with a
        # minimum turning radius, so an arbitrary final heading makes it loop.
        yaw = travel_heading(node, x, y)

    goal = NavigateToPose.Goal()
    goal.pose = PoseStamped()
    goal.pose.header.frame_id = 'map'
    goal.pose.header.stamp = node.get_clock().now().to_msg()
    goal.pose.pose.position.x, goal.pose.pose.position.y = x, y
    goal.pose.pose.orientation.z, goal.pose.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)

    client = ActionClient(node, NavigateToPose, 'navigate_to_pose')
    node.get_logger().info('Waiting for Nav2 (bt_navigator)...')
    client.wait_for_server()
    node.get_logger().info(f'Goal in map frame: x={x:.2f} y={y:.2f} yaw={math.degrees(yaw):.0f} deg')

    last = {'t': -10.0}

    def feedback(msg):
        fb = msg.feedback
        t = node.get_clock().now().nanoseconds * 1e-9
        if t - last['t'] >= 5.0:
            last['t'] = t
            p = fb.current_pose.pose.position
            node.get_logger().info(f'at ({p.x:6.2f}, {p.y:6.2f})  remaining {fb.distance_remaining:5.1f} m  '
                                   f'recoveries {fb.number_of_recoveries}  time {fb.navigation_time.sec} s')

    send = client.send_goal_async(goal, feedback_callback=feedback)
    rclpy.spin_until_future_complete(node, send)
    handle = send.result()
    if not handle.accepted:
        node.get_logger().error('Goal rejected')
        return 1
    result = handle.get_result_async()
    try:
        rclpy.spin_until_future_complete(node, result)
    except KeyboardInterrupt:
        node.get_logger().info('Cancelling goal')
        cancel = handle.cancel_goal_async()
        rclpy.spin_until_future_complete(node, cancel, timeout_sec=3.0)
        return 1
    status = result.result().status
    names = {4: 'SUCCEEDED', 5: 'CANCELED', 6: 'ABORTED'}
    node.get_logger().info(f'Result: {names.get(status, status)} '
                           f'(error code {result.result().result.error_code})')
    rclpy.shutdown()
    return 0 if status == 4 else 1


if __name__ == '__main__':
    sys.exit(main())
