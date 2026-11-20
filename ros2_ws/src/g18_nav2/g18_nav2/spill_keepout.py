"""Turn reported spills into a Nav2 keepout mask.

The LiDAR cannot see a spill. The facility reports them on /warehouse/status:

    "reported_spills": [{"id": "spill_1", "center": [-31, -13], "size": [2.1, 1.4]}, ...]

(warehouse frame, metres; size = [extent along warehouse x, extent along y]).

This node draws every reported spill (plus a safety margin) into an
OccupancyGrid in the SLAM `map` frame and publishes it for the costmaps'
KeepoutFilter. Cells with value 100 become lethal: the planner routes around
them, and the controller will not drive into them.

    /warehouse/status (String JSON) --> [spill_keepout] --> /keepout_mask (OccupancyGrid)
                                                        --> /costmap_filter_info (CostmapFilterInfo)

    ros2 run g18_nav2 spill_keepout --ros-args -p use_sim_time:=true
"""
import json
import math

import rclpy
from rclpy.executors import ExternalShutdownException
from nav2_msgs.msg import CostmapFilterInfo
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import String

from g18_nav2.go_to import START, warehouse_to_map

LATCHED = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)

# The hall is 96 x 64 m centred on the warehouse origin; cover it with a margin.
HALL_X = (-50.0, 50.0)
HALL_Y = (-34.0, 34.0)


def spill_rect_in_map(center, size, margin, start=START):
    cx, cy = center
    hx = size[0] / 2 + margin        # half extent along warehouse x + margin
    hy = size[1] / 2 + margin        # half extent along warehouse y + margin
    xs, ys = [], []
    for x in (cx - hx, cx + hx):     # west / east edge
        for y in (cy - hy, cy + hy): # south / north edge -> 4 corners
            mx, my, _ = warehouse_to_map(x, y, 0.0, start)
            xs.append(mx)
            ys.append(my)
    return min(xs), max(xs), min(ys), max(ys)

    


def fill_rect(grid, rect):
    xmin, xmax, ymin, ymax = rect
    info = grid.info
    res = info.resolution
    ox, oy = info.origin.position.x, info.origin.position.y

    # metres -> cell indices (column = x, row = y)
    col0 = int((xmin - ox) / res)
    col1 = int((xmax - ox) / res)
    row0 = int((ymin - oy) / res)
    row1 = int((ymax - oy) / res)

    # clamp to the grid so a spill at the edge cannot index out of range
    col0 = max(0, col0)
    row0 = max(0, row0)
    col1 = min(info.width - 1, col1)
    row1 = min(info.height - 1, row1)

    for row in range(row0, row1 + 1):
        for col in range(col0, col1 + 1):
            grid.data[row * info.width + col] = 100

   


class SpillKeepout(Node):
    def __init__(self):
        super().__init__('spill_keepout')
        self.margin = self.declare_parameter('margin', 0.3).value          # m around each spill
        self.resolution = self.declare_parameter('resolution', 0.1).value  # m per mask cell

        self.mask_pub = self.create_publisher(OccupancyGrid, '/keepout_mask', LATCHED)
        self.info_pub = self.create_publisher(CostmapFilterInfo, '/costmap_filter_info', LATCHED)
        self.create_subscription(String, '/warehouse/status', self.on_status, LATCHED)

        self.empty = self.make_empty_grid()
        self.last_ids = None
        self.publish_info()
        self.publish_mask([])   # start with no keepout areas

    def make_empty_grid(self):
        # Hall corners in the map frame decide the mask's extent.
        corners = [warehouse_to_map(x, y, 0.0, START)[:2] for x in HALL_X for y in HALL_Y]
        xs, ys = [c[0] for c in corners], [c[1] for c in corners]
        grid = OccupancyGrid()
        grid.header.frame_id = 'map'
        grid.info.resolution = self.resolution
        grid.info.width = int(math.ceil((max(xs) - min(xs)) / self.resolution))
        grid.info.height = int(math.ceil((max(ys) - min(ys)) / self.resolution))
        grid.info.origin.position.x = min(xs)
        grid.info.origin.position.y = min(ys)
        grid.info.origin.orientation.w = 1.0
        grid.data = [0] * (grid.info.width * grid.info.height)
        return grid

    def publish_info(self):
        info = CostmapFilterInfo()
        info.header.frame_id = 'map'
        info.header.stamp = self.get_clock().now().to_msg()
        info.type = 0                    # 0 = keepout
        info.filter_mask_topic = '/keepout_mask'
        info.base = 0.0
        info.multiplier = 1.0
        self.info_pub.publish(info)

    def publish_mask(self, spills):
        grid = OccupancyGrid()
        grid.header.frame_id = 'map'
        grid.header.stamp = self.get_clock().now().to_msg()
        grid.info = self.empty.info
        grid.data = list(self.empty.data)
        for spill in spills:
            fill_rect(grid, spill_rect_in_map(spill['center'], spill['size'], self.margin))
        self.mask_pub.publish(grid)

    def on_status(self, msg):
        try:
            spills = json.loads(msg.data).get('reported_spills', [])
        except json.JSONDecodeError:
            return
        ids = sorted(s['id'] for s in spills)
        if ids == self.last_ids:
            return               # status arrives at ~2 Hz; only redraw on change
        self.last_ids = ids
        self.publish_mask(spills)
        self.get_logger().info(f'keepout areas: {ids or "none"}')


def main():
    rclpy.init()
    node = SpillKeepout()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass   # Ctrl+C or a SIGTERM from launch/kill: exit quietly
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
