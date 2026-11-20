"""Republish the Webots LiDAR scan with a positive angle increment.

webots_ros2 publishes the scan left to right (angle_min = +90 deg, negative
angle_increment). Costmaps handle that, but slam_toolbox's Karto backend assumes
an increasing angle and effectively mirrors the scan, which turns SLAM the
wrong way in corners. This node reverses the arrays: /scan_raw -> /scan.
"""
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan


class ScanNormalizer(Node):
    def __init__(self):
        super().__init__('scan_normalizer')
        self.pub = self.create_publisher(LaserScan, '/scan', qos_profile_sensor_data)
        self.create_subscription(LaserScan, '/scan_raw', self.on_scan, qos_profile_sensor_data)

    def on_scan(self, msg):
        if msg.angle_increment < 0:
            msg.angle_min, msg.angle_max = msg.angle_max, msg.angle_min
            msg.angle_increment = -msg.angle_increment
            msg.ranges = list(reversed(msg.ranges))
            msg.intensities = list(reversed(msg.intensities))
        self.pub.publish(msg)


def main():
    rclpy.init()
    node = ScanNormalizer()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass   # Ctrl+C or a SIGTERM from launch/kill: exit quietly
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
