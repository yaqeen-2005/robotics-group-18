#!/usr/bin/env python3
"""Small, bounded manual-command smoke test; run in a clear simulated area only."""
import math
import time

import rclpy
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from rclpy.qos import qos_profile_sensor_data
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import LaserScan


def main():
    rclpy.init()
    node = rclpy.create_node('warehouse_motion_check')
    data = {}
    for key, topic, kind in [
        ('odom', '/r100_0000/platform/odom', Odometry),
        ('scan', '/r100_0000/sensors/lidar2d_0/scan', LaserScan),
        ('clock', '/clock', Clock),
    ]:
        node.create_subscription(kind, topic, lambda msg, k=key: data.update({k: msg}),
                                 qos_profile_sensor_data)
    pub = node.create_publisher(TwistStamped, '/r100_0000/cmd_vel', 10)

    def command(x=0.0, y=0.0, yaw=0.0):
        msg = TwistStamped()
        if 'clock' in data:
            msg.header.stamp = data['clock'].clock
        msg.twist.linear.x, msg.twist.linear.y, msg.twist.angular.z = x, y, yaw
        pub.publish(msg)

    def sim_time():
        stamp = data['clock'].clock
        return stamp.sec + stamp.nanosec * 1e-9

    def phase(seconds, velocity=(0.0, 0.0, 0.0)):
        start = sim_time()
        deadline = time.monotonic() + 60
        while sim_time() - start < seconds:
            if time.monotonic() > deadline:
                raise RuntimeError('Simulation stalled or too slow (60 s wall-time limit).')
            command(*velocity)
            rclpy.spin_once(node, timeout_sec=0.02)
            time.sleep(0.02)

    def pose():
        p = data['odom'].pose.pose
        q = p.orientation
        return (p.position.x, p.position.y,
                math.atan2(2*(q.w*q.z + q.x*q.y), 1-2*(q.y*q.y + q.z*q.z)))

    try:
        deadline = time.monotonic() + 40
        while len(data) != 3 or not pub.get_subscription_count():
            if time.monotonic() > deadline:
                raise RuntimeError(f'Missing live topics or command subscriber: {list(data)}')
            rclpy.spin_once(node, timeout_sec=0.1)
        phase(0.5)
        for name, velocity in [('forward', (0.15, 0.0, 0.0)),
                               ('sideways', (0.0, 0.15, 0.0)),
                               ('turn', (0.0, 0.0, 0.2))]:
            before = pose()
            phase(2.0, velocity)
            phase(0.5)
            after = pose()
            distance = math.hypot(after[0]-before[0], after[1]-before[1])
            angle = abs(math.atan2(math.sin(after[2]-before[2]), math.cos(after[2]-before[2])))
            print(f'{name}: displacement={distance:.3f} m, rotation={angle:.3f} rad', flush=True)
            if (angle if name == 'turn' else distance) < 0.05:
                raise RuntimeError(f'{name}: insufficient odometry response')
        scan = data['scan']
        finite = sum(math.isfinite(x) for x in scan.ranges)
        print(f'LiDAR: {len(scan.ranges)} rays, {finite} finite returns; frame={scan.header.frame_id}')
        if not finite:
            raise RuntimeError('No finite LiDAR returns')
        maps = [name for name, types in node.get_topic_names_and_types()
                if 'nav_msgs/msg/OccupancyGrid' in types]
        print(f'Occupancy-grid topics: {maps}')
        if maps:
            raise RuntimeError('Unexpected occupancy map; stop other mapping/navigation launches')
        print('PASS: command response and live LiDAR; visually confirm physical motion.')
    finally:
        for _ in range(10):
            command()
            time.sleep(0.02)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
