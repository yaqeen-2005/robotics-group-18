"""Manual bring-up only. No ROS, autonomous navigation, or ground-truth access."""
import math
import os
from pathlib import Path
import sys
from controller import Robot, Keyboard
from sensor_log import open_sensor_log
from docking import dock_command
from lidar import LidarSensor

robot = Robot()
step_ms = int(robot.getBasicTimeStep())
dt = step_ms / 1000.0
lidar = LidarSensor(robot, step_ms)

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
sensor_log = open_sensor_log(robot, step_ms, RADIUS, TRACK)
pose_publisher = None
mapper = None
if os.environ.get("G18_WAREHOUSE_MAP"):
    root = Path(__file__).resolve().parents[2]
    sys.path.insert(0, str(root.parent))
    from webots.localization.live_mapping import start_mapper
    mapper = start_mapper(robot, step_ms, RADIUS, TRACK, root, os.environ["G18_WAREHOUSE_MAP"])
if os.environ.get("G18_LOCALIZATION_POSE"):
    sys.path.insert(0, str(Path(__file__).resolve().parents[3]))
    from webots.localization.live_pose import open_pose_publisher

    pose_publisher = open_pose_publisher(RADIUS, TRACK)
lift_target = 0.0
previous = None
stopped_for = 0.0
last_drive_time = -1.0
command = (0.0, 0.0)
dock_target = None
dock_camera = None
dock_deadline = 0.
print('Final warehouse: from a clear approach stop, K docks at P03; L docks at D01. Space cancels.')
held = set()
lift_jogging = False
print('Click the 3D view. Hold W/S: forward/reverse, A/D: turn. Space: stop.')
print('After stopping, hold U/J: raise/lower; release to hold height. Space stops both.')
print('Lift travel blocks driving. R: print lift position.')
print('P: print LiDAR data.')
print('Manual prototype: no attachment, obstacle stop or autonomous controller is active.')

while robot.step(step_ms) != -1:
    now = robot.getTime()
    scan = lidar.get_scan()
    angles = [s.getValue() for s in encoders]
    if mapper is not None:
        mapper.update(now, angles)
    if pose_publisher is not None:
        pose_publisher.update(now, angles)
    if sensor_log is not None:
        sensor_log.record(now, angles)
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
    manual_keys = set(map(ord, ' WASDUJ'))
    if keys & manual_keys or now >= dock_deadline:
        if dock_target:
            command = (0.,0.)
        dock_target = None
    if stopped_for >= .4 and not keys & manual_keys:
        for key, station in [('K','P03'),('L','D01')]:
            if ord(key) in pressed:
                dock_target = station
                dock_deadline = now + 60.
                if dock_camera is None:
                    dock_camera = robot.getDevice('dock_camera')
                    dock_camera.enable(step_ms)
                    dock_camera.recognitionEnable(step_ms)
                print('Visual docking:',station,flush=True)
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
                   and math.isfinite(position))
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
    if dock_target and not moving_lift:
        v,w,aligned = dock_command(dock_camera,dock_target)
        command = (v,w)
        if aligned:
            print('Dock aligned:',dock_target,flush=True)
            dock_target = None
    if moving_lift:
        dock_target = None
        command = (0.0, 0.0)
        last_drive_time = -1.0
    v, yaw = command
    left.setVelocity((v-yaw*TRACK/2)/RADIUS)
    right.setVelocity((v+yaw*TRACK/2)/RADIUS)
    if ord('R') in pressed:
        print(f'Lift: {position:.3f} m, target {lift_target:.3f} m; stationary {stopped_for:.2f} s')
    if ord('P') in pressed:   # L is taken by docking at D01
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

if mapper is not None:
    mapper.save()
