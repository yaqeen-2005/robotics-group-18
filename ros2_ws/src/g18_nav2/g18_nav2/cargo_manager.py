"""Cargo-aware navigation: footprint and speed follow the pallet on board.

When an order is "in_transit" the robot carries its pallet on the lift. The
pallet usually sticks out past the body, so the costmaps must plan with the
combined outline (a wide pallet cannot use a 1.65 m doorway), and heavy or
fragile loads should be driven more slowly.

    /warehouse/status (orders) --> [cargo_manager] --> /local_costmap/footprint
                                                   --> /global_costmap/footprint
                                                   --> /speed_limit

    ros2 run g18_nav2 cargo_manager --ros-args -p use_sim_time:=true
    ros2 run g18_nav2 cargo_manager --ros-args -p use_sim_time:=true -p force_profile:=wide   # test without a pickup
"""
import json

import rclpy
from rclpy.executors import ExternalShutdownException
from geometry_msgs.msg import Point32, Polygon
from nav2_msgs.msg import SpeedLimit
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import String

LATCHED = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)

# Empty robot outline in base_link (drive axle at the origin, x forward).
BODY_FRONT, BODY_REAR, BODY_HALF_WIDTH = 0.56, -0.52, 0.47
# The lift platform (and so the pallet centre) sits over the drive axle.
PALLET_CENTRE_X = 0.0

# The five catalogue profiles (webots/config/integration_scenario.json), used
# for force_profile tests. Real orders carry their own deck_size_m/mass/handling.
PROFILES = {
    'standard': {'deck_size_m': [0.8, 1.2, 0.08], 'mass_kg': 10, 'handling': 'normal'},
    'wide':     {'deck_size_m': [1.2, 1.8, 0.08], 'mass_kg': 18, 'handling': 'normal'},
    'long':     {'deck_size_m': [1.8, 1.2, 0.08], 'mass_kg': 20, 'handling': 'normal'},
    'heavy':    {'deck_size_m': [0.8, 1.2, 0.08], 'mass_kg': 40, 'handling': 'heavy'},
    'fragile':  {'deck_size_m': [0.8, 1.2, 0.08], 'mass_kg': 8, 'handling': 'fragile'},
}


def cargo_footprint(deck_size):
    front, rear, half = BODY_FRONT, BODY_REAR, BODY_HALF_WIDTH
    if deck_size is not None:
        front = max(front, PALLET_CENTRE_X + deck_size[0] / 2)   # pallet sticks out past the front?
        rear = min(rear, PALLET_CENTRE_X - deck_size[0] / 2)     # past the rear? (negative, hence min)
        half = max(half, deck_size[1] / 2)                       # wider than the body?
    return [(round(front, 3), round(half, 3)), (round(front, 3), round(-half, 3)),
            (round(rear, 3), round(-half, 3)), (round(rear, 3), round(half, 3))]



def cargo_speed_limit(cargo):
    if cargo is None:
        return 0.0                                   # empty: no limit
    limit = {'normal': 0.5, 'heavy': 0.35, 'fragile': 0.25}.get(cargo.get('handling'), 0.5)
    if cargo.get('mass_kg', 0) > 30:
        limit = min(limit, 0.35)                     # anything over 30 kg is treated as heavy
    return limit



class CargoManager(Node):
    def __init__(self):
        super().__init__('cargo_manager')
        self.padding_note = 'footprint_padding (0.05 m) is added by the costmaps'
        self.force_profile = self.declare_parameter('force_profile', '').value

        # Costmap2DROS subscribes to <costmap>/footprint as geometry_msgs/Polygon (no header,
        # base frame implied); a PolygonStamped would be silently ignored.
        self.fp_pubs = [self.create_publisher(Polygon, topic, 10)
                        for topic in ('/local_costmap/footprint', '/global_costmap/footprint')]
        self.speed_pub = self.create_publisher(SpeedLimit, '/speed_limit', LATCHED)
        self.create_subscription(String, '/warehouse/status', self.on_status, LATCHED)
        # Costmaps only read the footprint topic on change; repeat it so a costmap
        # (re)started later still gets the current outline.
        self.create_timer(2.0, self.republish)

        self.cargo_key = 'unset'
        self.cargo = None
        if self.force_profile:
            if self.force_profile not in PROFILES:
                raise ValueError(f'force_profile must be one of {sorted(PROFILES)}')
            self.apply(f'forced:{self.force_profile}', PROFILES[self.force_profile])
        else:
            self.apply('empty', None)

    def on_status(self, msg):
        if self.force_profile:
            return
        try:
            status = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        carried = [o for o in status.get('output_pallets', []) if o.get('status') == 'in_transit']
        if carried:
            order = carried[0]
            self.apply(f"{order['id']}:{order.get('profile')}", order)
        else:
            self.apply('empty', None)

    def apply(self, key, cargo):
        if key == self.cargo_key:
            return
        self.cargo_key, self.cargo = key, cargo
        self.republish()
        limit = cargo_speed_limit(cargo)
        speed = SpeedLimit()
        speed.header.frame_id = 'base_link'
        speed.header.stamp = self.get_clock().now().to_msg()
        speed.percentage = False
        speed.speed_limit = float(limit)
        self.speed_pub.publish(speed)
        outline = cargo_footprint(cargo['deck_size_m'] if cargo else None)
        self.get_logger().info(f'cargo {key}: footprint {outline}, '
                               f'speed limit {"none" if limit == 0 else f"{limit} m/s"}')

    def republish(self):
        outline = cargo_footprint(self.cargo['deck_size_m'] if self.cargo else None)
        msg = Polygon()
        msg.points = [Point32(x=float(x), y=float(y), z=0.0) for x, y in outline]
        for pub in self.fp_pubs:
            pub.publish(msg)


def main():
    rclpy.init()
    node = CargoManager()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass   # Ctrl+C or a SIGTERM from launch/kill: exit quietly
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
