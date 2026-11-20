"""webots_ros2 plugin: warehouse radio and raw lift access for the pallet robot.

Runs inside the robot's ``<extern>`` controller (webots_ros2_driver), next to
ros2_control, which owns the wheels. This file only forwards data; task logic,
pickup decisions and cargo handling belong to the teammates' nodes.

Topics
  /warehouse/status          std_msgs/String  JSON ``warehouse_status`` packets (channel 18)
  /warehouse/command_result  std_msgs/String  JSON ``command_result`` replies (channel 18)
  /warehouse/command         std_msgs/String  JSON commands sent to the facility (channel 19)
  /lift/command              std_msgs/Float64 target platform height in metres (0.0 - 0.6)
  /lift/height               std_msgs/Float64 measured platform height
  /lift/moving               std_msgs/Bool    true while the lift is travelling; do not drive
"""
import json
import math

import rclpy
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import Bool, Float64, String

LIFT_MAX = 0.60
WHEEL_RADIUS = 0.15
STOPPED_SPEED = 0.01      # m/s at the tyre, as in the manual controller
STOPPED_TIME = 0.4        # s the wheels must be still before the lift may move
LIFT_TOLERANCE = 0.003    # m
LIFT_PUBLISH_PERIOD = 0.1  # s


class WarehouseBridge:
    def init(self, webots_node, properties):
        self.robot = webots_node.robot
        self.step_ms = int(self.robot.getBasicTimeStep())
        self.dt = self.step_ms / 1000.0

        if not rclpy.ok():
            rclpy.init(args=None)
        self.node = rclpy.create_node('warehouse_bridge')
        log = self.node.get_logger()

        # Warehouse radio. Devices exist only in worlds that add them to extensionSlot.
        self.rx = self.robot.getDevice('warehouse_events')
        self.tx = self.robot.getDevice('warehouse_commands')
        if self.rx is not None:
            self.rx.enable(self.step_ms)
        else:
            log.warn('No warehouse_events receiver in this world; /warehouse/* is inactive.')

        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.status_pub = self.node.create_publisher(String, '/warehouse/status', latched)
        self.result_pub = self.node.create_publisher(String, '/warehouse/command_result', 10)
        self.node.create_subscription(String, '/warehouse/command', self._on_command, 10)

        # Lift. The two telescope stages only follow the platform visually.
        self.lift = self.robot.getDevice('lift_motor')
        self.lift_sensor = self.robot.getDevice('lift_sensor')
        self.lift_stages = [self.robot.getDevice('lift_stage1_motor'),
                            self.robot.getDevice('lift_stage2_motor')]
        self.lift_sensor.enable(self.step_ms)
        self.lift.setVelocity(0.25)
        self.lift.setPosition(0.0)
        for stage in self.lift_stages:
            stage.setVelocity(0.5)
            stage.setPosition(0.0)
        self.lift_target = 0.0
        self.pending_lift = None

        # Wheel encoders, read only, for the stationary check before lifting.
        self.encoders = [self.robot.getDevice('left_wheel_sensor'),
                         self.robot.getDevice('right_wheel_sensor')]
        for sensor in self.encoders:
            if sensor.getSamplingPeriod() == 0:
                sensor.enable(self.step_ms)
        self.previous_angles = None
        self.stopped_for = 0.0

        self.height_pub = self.node.create_publisher(Float64, '/lift/height', 10)
        self.moving_pub = self.node.create_publisher(Bool, '/lift/moving', 10)
        self.node.create_subscription(Float64, '/lift/command', self._on_lift_command, 10)
        self.since_lift_publish = 0.0

        log.info('Warehouse bridge ready.')

    # ---- callbacks -------------------------------------------------------

    def _on_command(self, msg):
        if self.tx is None:
            self.node.get_logger().warn('No warehouse_commands emitter; command dropped.')
            return
        try:
            json.loads(msg.data)
        except json.JSONDecodeError as error:
            self.node.get_logger().error(f'/warehouse/command is not JSON: {error}')
            return
        self.tx.send(msg.data.encode('utf-8'))

    def _on_lift_command(self, msg):
        if not math.isfinite(msg.data):
            self.node.get_logger().error('Lift command must be a finite height.')
            return
        self.pending_lift = max(0.0, min(LIFT_MAX, msg.data))

    # ---- per simulation step ---------------------------------------------

    def step(self):
        rclpy.spin_once(self.node, timeout_sec=0)
        self._forward_radio()
        self._update_lift()

    def _forward_radio(self):
        if self.rx is None:
            return
        while self.rx.getQueueLength() > 0:
            text = self.rx.getString()
            self.rx.nextPacket()
            try:
                kind = json.loads(text).get('type')
            except (json.JSONDecodeError, AttributeError):
                self.node.get_logger().warn('Ignored a non-JSON warehouse packet.')
                continue
            msg = String(data=text)
            if kind == 'command_result':
                self.result_pub.publish(msg)
            else:
                self.status_pub.publish(msg)

    def _update_lift(self):
        angles = [sensor.getValue() for sensor in self.encoders]
        speed = math.inf
        if all(math.isfinite(a) for a in angles) and self.previous_angles is not None:
            speed = max(abs(a - b) * WHEEL_RADIUS / self.dt
                        for a, b in zip(angles, self.previous_angles))
        self.previous_angles = angles
        self.stopped_for = self.stopped_for + self.dt if speed < STOPPED_SPEED else 0.0

        height = self.lift_sensor.getValue()
        if not math.isfinite(height):
            return

        if self.pending_lift is not None:
            if self.stopped_for >= STOPPED_TIME:
                self.lift_target = self.pending_lift
                self.lift.setPosition(self.lift_target)
                self.pending_lift = None
            # Otherwise keep the request until the wheels have been still long enough.

        extension = max(0.0, min(LIFT_MAX, height))
        for index, stage in enumerate(self.lift_stages, 1):
            stage.setPosition(extension * index / 3)

        self.since_lift_publish += self.dt
        if self.since_lift_publish >= LIFT_PUBLISH_PERIOD:
            self.since_lift_publish = 0.0
            moving = self.pending_lift is not None or abs(height - self.lift_target) > LIFT_TOLERANCE
            self.height_pub.publish(Float64(data=height))
            self.moving_pub.publish(Bool(data=moving))
