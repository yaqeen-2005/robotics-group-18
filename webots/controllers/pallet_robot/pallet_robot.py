"""Manual bring-up only. No ROS, autonomous navigation, or ground-truth access."""
import math
from controller import Robot, Keyboard

robot = Robot()
step_ms = int(robot.getBasicTimeStep())
dt = step_ms / 1000.0
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
print('Manual prototype: no attachment, obstacle stop or autonomous controller is active.')

while robot.step(step_ms) != -1:
    now = robot.getTime()
    angles = [s.getValue() for s in encoders]
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
    if moving_lift:
        command = (0.0, 0.0)
        last_drive_time = -1.0
    v, yaw = command
    left.setVelocity((v-yaw*TRACK/2)/RADIUS)
    right.setVelocity((v+yaw*TRACK/2)/RADIUS)
    if ord('R') in pressed:
        print(f'Lift: {position:.3f} m, target {lift_target:.3f} m; stationary {stopped_for:.2f} s')
