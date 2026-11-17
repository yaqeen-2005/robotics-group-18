# Webots warehouse and pallet robot

The additional **integration_test.wbt** world is described in
[INTEGRATION_TEST.md](INTEGRATION_TEST.md), including spill/traffic controls,
load profiles, team interfaces and validation. The original warehouse below
remains the small component test world.

## Open on Windows

1. Start your installed Webots from its existing shortcut. The supplied
   `C:\ProgramData\Microsoft\Windows\Start Menu\Programs\Cyberbotics` path holds
   shortcuts, rather than the application executable; no executable path is
   needed for this workflow.
2. In Webots select **File → Open World** and open `worlds/warehouse.wbt`.
   The source currently resides in WSL at
   `\\wsl.localhost\Ubuntu\home\tmozz\robotics-group-18\webots`.
   For a reliable native Windows working directory, copy the **whole `webots`
   folder** to a writable Windows project folder, or use a Windows checkout of
   this repository after committing the changes. Copying only the world breaks
   relative PROTO/controller references. Keep track of which copy you edit.
3. Files target the R2025a format. Confirm the complete version under Help/About
   if your installation reports only “R2025”. Review any compatibility warning.
4. The controller uses Python 3 and only the standard library plus Webots' own
   `controller` module. If it does not start, configure Webots' Python command
   in its preferences for an installed Windows Python 3 interpreter. Do not
   install a similarly named `controller` package with pip.
5. Press Play at real-time speed and click the 3D view to give it keyboard focus.

No ROS installation, colcon build, or WSL terminal command is required to run it.
All model assets are local primitives; no online robot models are referenced.

The saved camera now starts directly overhead at (0, 0, 20). If the view is blank,
reload the updated world without saving the old camera over it. If using a Windows
copy, copy the updated world there first. Check the Scene Tree for the floor,
walls and `TRANSPORT_ROBOT`, and check console errors if objects are missing.

WSL storage is optional. For native Windows use, a writable folder such as
`C:\Users\<you>\Projects\robotics-group-18` keeps Webots, Python and files on
the same operating system. Copy the whole repository including hidden `.git`
and uncommitted files before retiring the original; a fresh Git clone alone
does not include uncommitted work. Avoid working in both copies concurrently.
Moving files does not automatically reopen this assistant's workspace: reopen
the Windows folder in the editor/Codex before requesting further edits there.

## Controls

| Key | Action |
| --- | --- |
| Hold W / S | Forward / reverse at 0.60 m/s command |
| Hold A / D | Left / right turn at 0.50 rad/s command |
| Space | Stop wheels and hold current lift height |
| Hold U | Raise platform; release to hold height (maximum 0.60 m) |
| Hold J | Lower platform; release to hold height (minimum zero) |
| R | Print measured lift extension and target in Webots console |

Wheel commands expire after 0.12 simulated seconds without driving-key events.
Lift motion requires U/J keyboard events each step; release (or both keys together)
holds the measured height. There is no lift-key timeout. The window needs focus
and keyboard repeat behaviour must be checked; missing events can interrupt travel.
The platform accepts commands only after wheel feedback has been stationary
for 0.4 s and driving commands are zero. Driving is blocked during lift travel.
Stationarity uses wheel encoders, not chassis velocity: sliding is not detected.
These are prototype controls, not a verified safety system. There is no automatic
obstacle stop, cargo profile switching or pallet attachment.

## Model and warehouse design

Coordinates are metres, +x forward, +y left, +z up. Robot origin is the wheel axle.
The robot starts at (-4.8, -3), facing +x toward the normal pallet at (-3, -3).
The wide pallet is at (3, -3); the empty delivery stand is at (-3, 3).

- Interior: 12 × 10 m. Shortcut centred at (-3, 0), width 1.60 m; wide opening
  centred at (3, 0), width 2.40 m. Passage widths are measured between wall faces.
- Chassis: 0.99 × 0.68 × 0.18 m, 40 kg. Wheel radius 0.15 m, track 0.82 m,
  wheel width 0.10 m and mass 2 kg each. Two spherical low-friction supports
  approximate caster support; they are not detailed swivelling casters.
- Empty conservative rectangular envelope: 1.08 × 0.92 m, centred on the axle.
- Platform: 0.80 × 0.72 × 0.04 m, 5 kg. Nominal top height is 0.40–1.00 m
  above the floor after settling. Lift speed 0.25 m/s, max force 1500 N.
  A central telescopic sleeve, two intermediate stages and a piston connect
  the chassis and platform. Each intermediate stage adds 0.05 kg.
  These remain inside the robot envelope; the slider supplies the physical
  constraint; the intermediate stages have cylindrical collision shapes.
- Normal pallet: 0.80 × 1.20 × 0.08 m, 10 kg. Wide: 1.20 × 1.40 × 0.08 m,
  15 kg. Full boxes provide visual and collision geometry, including overhang.
- Stand tops: 0.48 m. Each has two rails leaving 1.04 m clear between them.
  Enter along the x axis. The lowered deck should fit below the pallet.
- Nominal LiDAR height 0.28 m, below the lowered platform and pallet. Device
  `front_lidar` has a 180-degree forward field, 360 samples and 12 m range.
  It is supplied as a mounting/device placeholder; the manual controller does
  not enable it or implement Yaqeen's sensor processing.

Physics nodes specify mass; Webots derives inertia from collision geometry.
These values are initial simulation design assumptions, not measured hardware
parameters. `config/geometry.json` records the dimensions for team integration;
changing it alone does not change PROTO/world dimensions or the controller.

With a 0.05 m margin, the empty enclosing circle diameter is about 1.52 m,
normal loaded about 1.71 m, and wide loaded about 1.94 m. Under that conservative
planner, the empty robot should fit the shortcut and both loaded profiles should
use the wide passage. These are design calculations; grid resolution, offsets,
and actual collision behaviour still need validation. No planner uses them yet.

## Lift startup fix (16 November 2026)

An isolated Webots run reproduced repeated lift jumps of roughly 7 cm at the
lower commanded limit. Mechanical stops now sit 5 mm outside the unchanged
0–0.22 m commanded travel, avoiding motor/stop chatter. The LiDAR housing now
has a 0.1 kg Physics node so its collision box participates in physics.

With the revised model, a probe controller checked 10 seconds of settling and
three empty raise/lower cycles. Final lift errors were below 0.06 mm. The LiDAR
NULL-physics warning was absent. Keyboard driving, loaded handling and the full
manual acceptance sequence still need validation. Reload the world from disk
to pick up the PROTO changes; restarting just the controller is insufficient.

## Manual acceptance checklist

The initial migration was not simulation-tested; the limited checks above were
performed subsequently. For the remaining manual checks,
Record your Webots version, pass/fail, console errors and observations:

1. **Load:** open the world. Check for unknown fields, missing PROTOs, Python
   errors or controller failures. Confirm one robot, two pallets, three stands,
   walls and shelves. Inspect the camera view and adjust it with the mouse.
2. **Settle:** run for 10 seconds without commands. Robot wheels/supports should
   rest on the floor; pallets should rest on rails. Check for sinking, jumping,
   chassis scraping and unwanted creep.
3. **Drive:** in clear space, W moves toward +x, S reverses, A turns left and D
   right. Verify releasing keys and Space stop motion. Check straight tracking,
   turning traction and behaviour when the window loses focus.
4. **Lift empty:** stop, wait, then hold U. Release halfway and press R to check
   that height holds. Hold U to reach 0.60 m; hold J to return to zero. Check
   release in both directions and Space during travel. Try driving during travel (should be blocked)
   and requesting lift while driving (should be rejected).
5. **Collision:** approach a wall slowly. The chassis should collide rather than
   pass through it. Inspect wheels, supports, slab and shelf collision shapes
   using Webots' optional rendering of bounding objects.
6. **Pickup geometry:** approach the normal stand along +x with the lift lowered.
   Align the platform under the pallet centre, stop and raise. Check rail/wheel
   clearance and that the pallet is supported above the rails. Reverse slowly
   and observe stability. **It is supported by contact only and can slide/fall**;
   this does not prove attachment or reliable transport. Reload after a failed attempt.
7. **Wide load:** repeat at the wide stand. Check overhang collisions and pallet
   stability. On unloading, centre on the delivery rails and lower until they
   support the pallet before withdrawing.
8. **Sensors:** visually confirm the sensor is clear of the platform and both
   cargo types. Its owner must enable it and inspect scans before SLAM integration.
9. **Repeatability/performance:** reload the saved world to restore initial poses.
   Repeat driving and lifting, record lag/errors, and repeat on another teammate's PC.

Do not save the world in a displaced or dropped-pallet state over the baseline.
Runtime tests are necessary before this becomes the team's accepted simulation.

## Editing and migration

`protos/PalletRobot.proto` defines robot physics and devices; `Pallet.proto`
defines reusable loads. `controllers/pallet_robot/pallet_robot.py` is only the
manual bring-up controller. The robot has no Supervisor privileges and uses no
world coordinates for navigation. There is no saved occupancy map.

`tools/build_world.py` generates the primitive layout and station locations.
The generated world is checked in and opens directly. If you later regenerate,
manual edits to `warehouse.wbt` will be overwritten; make layout changes in the
builder or adopt GUI editing and stop using the builder.

The existing planner/map/event work under `ros2_ws/src/navigation` and all other
Yi/Yaqeen components are unchanged. Future integration should agree device/API
interfaces with their owners. Wheel position devices and the LiDAR mount are
available; no odometry estimator, SLAM replacement, fragile controller, or Nav2
substitute is claimed here. Rigid attachment and confirmed load-state exchange
remain a separate implementation milestone.

Legacy Gazebo files are retained as reference. Share this project by committing
source files and having teammates pull the same revision. Each runs a local
simulation. Do not share generated caches or assume the Windows copy is synced
with the WSL checkout.

## Manual feedback and jog controls (16 November 2026)

Thomas reported that settling, W/A/S/D and collision/contact pickup worked after
the startup fix. This is a manual observation, not a repeated loaded validation.
Subsequent changes add hold-to-move U/J, Space lift hold, 0.50 rad/s turning, and
a visual telescopic actuator. A mocked controller check passed key release in
both directions, drive lockout while lifting, rejection with drive commands,
turn speed and Space hold. Real keyboard timing and loaded stability with these
changes still need checking in Webots.

The lift now has 0.60 m travel at 0.25 m/s (previously 0.22 m at
0.04 m/s). Historical validation notes above describe the earlier settings.
The original central actuator appearance is restored with intermediate stages
to keep the piston connected over the extended travel. Forward/reverse manual
speed is now 0.60 m/s (three times the original speed).

Replacement controllers should drive `lift_stage1_motor` and `lift_stage2_motor`
to one third and two thirds of the measured `lift_sensor` extension each step.
The main `lift_motor` and `lift_sensor` retain their existing interfaces.
