"""Manual bring-up only. No ROS, autonomous navigation, or ground-truth access."""
import math
from controller import Robot, Keyboard
from lidar import LidarSensor
from lidar_visualiser import LidarVisualiser
from odometry import WheelOdometry
from odometry_visualiser import OdometryVisualiser
import matplotlib.pyplot as plt
from sensor_interface import SensorInterface
from motion_controller import MotionController
from cargo_profiles import CARGO_PROFILES


robot = Robot()
step_ms = int(robot.getBasicTimeStep())
dt = step_ms / 1000.0
lidar = LidarSensor(robot, step_ms)
visualiser = LidarVisualiser()

keyboard = robot.getKeyboard()
keyboard.enable(step_ms)
left = robot.getDevice('left_wheel_motor')
right = robot.getDevice('right_wheel_motor')
encoders = [robot.getDevice(name) for name in ('left_wheel_sensor', 'right_wheel_sensor')]
lift = robot.getDevice('lift_motor')
lift_sensor = robot.getDevice('lift_sensor')
for motor in (left, right):
    motor.setPosition(float('inf'))
    motor.setVelocity(0)
for sensor in encoders + [lift_sensor]:
    sensor.enable(step_ms)
lift.setVelocity(0.25)
lift.setPosition(0)
lift_stages = [robot.getDevice('lift_stage1_motor'), robot.getDevice('lift_stage2_motor')]
for stage in lift_stages:
    stage.setPosition(0)
    stage.setVelocity(0.5)

RADIUS = 0.15
TRACK = 0.82
odom_track = 0.911
odometry = WheelOdometry(RADIUS,odom_track)
odometry_visualiser = OdometryVisualiser()
motion = MotionController("empty")

profile_keys = {
    ord("Z"): "empty",
    ord("X"): "standard",
    ord("C"): "wide",
    ord("Q"): "fragile"
}

sensors = SensorInterface(
    lidar.resolution,
    lidar.fov,
    lidar.lidar.getMinRange(),
    lidar.lidar.getMaxRange()
)

lift_target = 0.0
previous = None
stopped_for = 0.0
last_drive_time = -1.0
command = (0.0, 0.0)
held = set()
lift_jogging = False
print('Click the 3D view. Hold W/S: forward/reverse, A/D: turn. Space: stop.')
print('After stopping, hold U/J: raise/lower; release to hold height. Space stops both.')
print('Lift travel blocks driving. R: print lift position.')
print('Print LiDAR Data: L.')
print('Show LiDAR Visualisation: P.')
print("Print Wheel Odometry: O.")
print("Show Odometry Trajectory: T.")
print("Print SLAM Sensor Frame: K.")
print("Cargo profiles: Z - Empty, X - Standard, C - Wide, Q - Fragile.")
print("Print Motion Controller Data: M.")
print('Manual prototype: no attachment, obstacle stop or autonomous controller is active.')

while robot.step(step_ms) != -1:
    now = robot.getTime()
    scan = lidar.get_scan()
    angles = [s.getValue() for s in encoders]
    odom_updated = odometry.update(angles[0], angles[1], now)

    sensor_frame = sensors.capture(
        now, scan, odometry, odom_updated
    )
    odometry_visualiser.update(odometry.get_pose())
    valid = all(math.isfinite(x) for x in angles)
    wheel_speed = float('inf')
    if valid and previous is not None:
        wheel_speed = max(abs(a-b)*RADIUS/dt for a,b in zip(angles, previous))
    previous = angles if valid else None
    stopped_for = stopped_for + dt if wheel_speed < 0.01 else 0.0
    keys = set()
    key = keyboard.getKey()
    while key != -1:
        keys.add(key & 0xFFFF)
        key = keyboard.getKey()
    pressed = keys - held
    held = keys
    if ord(' ') in keys:
        command = (0.0, 0.0)
        last_drive_time = -1.0
    else:
        v = 0.60 * ((ord('W') in keys) - (ord('S') in keys))
        yaw = 0.50 * ((ord('A') in keys) - (ord('D') in keys))
        if v or yaw:
            command = (v, yaw)
            last_drive_time = now
        if now - last_drive_time > 0.12:
            command = (0.0, 0.0)
    position = lift_sensor.getValue()
    if math.isfinite(position):
        extension = max(0.0, min(0.60, position))
        for index, stage in enumerate(lift_stages, 1):
            stage.setPosition(extension * index / 3)
    lift_direction = (ord('U') in keys) - (ord('J') in keys)
    jog_allowed = (lift_direction != 0 and ord(' ') not in keys
                   and stopped_for >= 0.4 and command == (0.0, 0.0)
                   and math.isfinite(position)
                   and motion.command == (0.0,0.0)
                   )
    if jog_allowed:
        lift_target = 0.60 if lift_direction > 0 else 0.0
        lift.setPosition(lift_target)
    elif lift_jogging and math.isfinite(position):
        # Position control holds the measured height against gravity on release.
        lift_target = max(0.0, min(0.60, position))
        lift.setPosition(lift_target)
    if lift_direction and not jog_allowed and ord(' ') not in keys:
        if ord('U') in pressed or ord('J') in pressed:
            print('Lift request rejected: release driving controls and stop for 0.4 s.')
    lift_jogging = jog_allowed
    moving_lift = (lift_jogging or not math.isfinite(position)
                   or abs(position-lift_target) > 0.003)
    if moving_lift:
        command = (0.0, 0.0)
        last_drive_time = -1.0

    for key, profile in profile_keys.items():
        if key in pressed:
            if (
                stopped_for >= 0.4
                and command == (0.0, 0.0)
                and motion.command == (0.0, 0.0)
                and not moving_lift
                and not any(ord(k) in keys for k in "WSADUJ")
            ):
                motion.set_profile(profile)
                print(f"Cargo profile: {profile.upper()}")
            else:
                print("Stop the robot before changing cargo profile.")

    if ord(' ') in keys or moving_lift or not valid:
        v, yaw = motion.stop()
        command = (0.0, 0.0)
        last_drive_time = -1.0
    else:
        v, yaw = motion.update(command[0], command[1], dt)
    
    left.setVelocity((v-yaw*TRACK/2)/RADIUS)
    right.setVelocity((v+yaw*TRACK/2)/RADIUS)
    if ord('R') in pressed:
        print(f'Lift: {position:.3f} m, target {lift_target:.3f} m; stationary {stopped_for:.2f} s')
    if ord('L') in pressed:
        valid_ranges = [d for d in scan if math.isfinite(d)]
        print(f"LiDAR readings: {len(scan)}")
        print(f"Resolution: {lidar.resolution}")
        print(f"Field of view: {math.degrees(lidar.fov):.2f} degrees")
        print(f"Angle increment: {math.degrees(lidar.angle_increment):.2f} degrees")
        print(f"Far left: {scan[0]:.2f} m")
        print(f"Front left: {scan[90]:.2f} m")
        print(f"Centre: {scan[180]:.2f} m")
        print(f"Front right: {scan[270]:.2f} m")
        print(f"Far right: {scan[359]:.2f} m")
        if valid_ranges:
            print(f"Closest obstacle: {min(valid_ranges):.2f} m")
            print(f"Centre reading: {scan[len(scan) // 2]:.2f} m")
        else:
            print("No valid distance readings")
        points = lidar.get_points(scan)

        print(f"Detected points: {len(points)}")

        for i in [0, 90, 180, 270, 359]:
            if i < len(scan) and math.isfinite(scan[i]):
                angle = lidar.fov / 2 - (i + 0.5) * lidar.angle_increment

                x = scan[i] * math.cos(angle)
                y = scan[i] * math.sin(angle)

                print(f"Index {i}: X: {x:.3f} m, Y: {y:.3f} m")
    if ord('P') in pressed:
        points = lidar.get_points(scan)
        visualiser.show_scan(points)
    if ord('O') in pressed:
        x, y, theta = odometry.get_pose()
        v, w = odometry.get_velocity()
        print("----- Wheel Odometry -----")
        print(f"Left encoder: {angles[0]:.3f} rad")
        print(f"Right encoder: {angles[1]:.3f} rad")
        print(f"X: {x:.3f} m")
        print(f"Y: {y:.3f} m")
        print(f"Heading: {math.degrees(theta):.2f} degrees")
        print(f"Linear velocity: {v:.3f} m/s")
        print(f"Angular velocity: {w:.3f} rad/s")
    if ord('T') in pressed:
        odometry_visualiser.show_trajectory()
    if plt.get_fignums():
        plt.pause(0.001)
    if ord('K') in pressed and sensor_frame is not None:
        print("----- SLAM Sensor Frame -----")
        print(f"Time: {sensor_frame.time_s:.2f} s")
        print(f"Scan readings: {len(sensor_frame.ranges_m)}")
        print(f"Detected points: {len(sensor_frame.points_base_m)}")
        print(f"Pose: {sensor_frame.odom_pose}")
        print(f"Velocity: {sensor_frame.velocity}")
        print(f"LiDAR offset: {sensor_frame.lidar_offset_m}")
    if ord('M') in pressed:
        profile = CARGO_PROFILES[motion.profile_name]
        cmd_v, cmd_w = motion.command
        actual_v, actual_w = odometry.get_velocity()

        print("----- Cargo Motion Control -----")
        print(f"Profile: {motion.profile_name.upper()}")
        print(f"Speed limit: {profile.max_speed:.3f} m/s")
        print(f"Acceleration limit: {profile.max_acceleration:.3f} m/s²")
        print(f"Deceleration limit: {profile.max_deceleration:.3f} m/s²")
        print(f"Turning limit: {profile.max_turn_speed:.3f} rad/s")
        print(f"Angular acceleration limit: {profile.max_angular_acceleration:.3f} rad/s²")
        print(f"Commanded speed: {cmd_v:.3f} m/s")
        print(f"Commanded turning: {cmd_w:.3f} rad/s")
        print(f"Measured speed: {actual_v:.3f} m/s")
        print(f"Measured turning: {actual_w:.3f} rad/s")
