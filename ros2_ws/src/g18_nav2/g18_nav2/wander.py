"""Drive around on the LiDAR alone so slam_toolbox can build a temporary map.

This is a mapping helper, not navigation: it never plans, and it cannot know
whether every room has been visited. It follows the most open direction, turns
away from obstacles in place, and changes its preferred side every so often so
it does not circle one area forever. Stop it with Ctrl+C at any time.

    ros2 run g18_nav2 wander --ros-args -p use_sim_time:=true -p duration:=600
"""
import math
import random

import rclpy
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan


class Wander(Node):
    def __init__(self):
        super().__init__('wander')
        self.speed = self.declare_parameter('speed', 0.4).value          # m/s
        self.turn_speed = self.declare_parameter('turn_speed', 0.6).value  # rad/s
        self.stop_dist = self.declare_parameter('stop_distance', 1.3).value  # front, from the LiDAR
        self.side_dist = self.declare_parameter('side_distance', 0.75).value  # body half-width + margin
        self.duration = self.declare_parameter('duration', 600.0).value  # s of sim time, 0 = forever
        self.switch_period = self.declare_parameter('switch_period', 45.0).value

        self.scan = None
        self.pose = None
        self.turning = 0           # 0 = driving, +1 left, -1 right
        self.prefer = 1            # side to favour when both are similar
        self.start = None
        self.last_switch = None
        self.progress_pose = None
        self.progress_time = None

        self.cmd = self.create_publisher(TwistStamped, '/cmd_vel', 10)
        self.create_subscription(LaserScan, '/scan', self.on_scan, qos_profile_sensor_data)
        self.create_subscription(Odometry, '/odom', self.on_odom, 10)
        self.create_timer(0.1, self.step)
        self.get_logger().info('Wandering for mapping. Ctrl+C stops the robot.')

    def on_scan(self, msg):
        self.scan = msg

    def on_odom(self, msg):
        p = msg.pose.pose.position
        self.pose = (p.x, p.y)

    def sector(self, lo_deg, hi_deg):
        """Smallest range between two bearings (deg, left positive)."""
        s = self.scan
        best = s.range_max
        for i, r in enumerate(s.ranges):
            a = math.degrees(s.angle_min + i * s.angle_increment)
            if lo_deg <= a <= hi_deg:
                if math.isfinite(r) and s.range_min < r < best:
                    best = r
        return best

    def mean_range(self, lo_deg, hi_deg):
        s = self.scan
        vals = []
        for i, r in enumerate(s.ranges):
            a = math.degrees(s.angle_min + i * s.angle_increment)
            if lo_deg <= a <= hi_deg:
                vals.append(r if math.isfinite(r) else s.range_max)
        return sum(vals) / len(vals) if vals else 0.0

    def publish(self, v, w):
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        msg.twist.linear.x = float(v)
        msg.twist.angular.z = float(w)
        self.cmd.publish(msg)

    def step(self):
        now = self.get_clock().now().nanoseconds * 1e-9
        if self.scan is None or now == 0.0:
            return
        if self.start is None:
            self.start = self.last_switch = self.progress_time = now
            self.progress_pose = self.pose
        if self.duration > 0 and now - self.start > self.duration:
            self.publish(0.0, 0.0)
            self.get_logger().info('Time is up; stopping.')
            raise SystemExit

        if now - self.last_switch > self.switch_period:
            self.prefer = random.choice((-1, 1))
            self.last_switch = now

        # Bearings are measured from the LiDAR, 0.52 m ahead of the axle.
        front = self.sector(-25, 25)
        front_left = self.sector(25, 70)
        front_right = self.sector(-70, -25)
        left_open = self.mean_range(20, 90)
        right_open = self.mean_range(-90, -20)

        # Stuck: barely moved for 15 s although driving -> turn around.
        if self.pose and self.progress_pose:
            if math.dist(self.pose, self.progress_pose) > 0.5:
                self.progress_pose, self.progress_time = self.pose, now
            elif now - self.progress_time > 15.0:
                self.turning = self.prefer
                self.progress_pose, self.progress_time = self.pose, now
                self.get_logger().info('No progress, turning away.')

        blocked = front < self.stop_dist or min(front_left, front_right) < self.side_dist
        if self.turning:
            # Keep turning until the way ahead is clearly open.
            if front > self.stop_dist + 0.8 and min(front_left, front_right) > self.side_dist + 0.2:
                self.turning = 0
            else:
                self.publish(0.0, self.turning * self.turn_speed)
                return
        if blocked:
            diff = left_open - right_open
            self.turning = self.prefer if abs(diff) < 0.5 else (1 if diff > 0 else -1)
            self.publish(0.0, self.turning * self.turn_speed)
            return

        # Drive, steering gently toward open space and slowing near obstacles.
        steer = 0.25 * (left_open - right_open) / max(left_open + right_open, 1e-3) * 4.0
        steer += 0.1 * self.prefer
        v = self.speed * min(1.0, (front - self.stop_dist) / 2.0 + 0.3)
        self.publish(v, max(-0.5, min(0.5, steer)))


def main():
    rclpy.init()
    node = Wander()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        if rclpy.ok():
            node.publish(0.0, 0.0)
            node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()
