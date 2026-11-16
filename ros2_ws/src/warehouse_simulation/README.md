# Ridgeback warehouse milestone

Run these commands in Ubuntu/WSL Bash, from the repository root. Dependencies:
ROS 2 Jazzy, Gazebo Harmonic, and `ros-jazzy-clearpath-simulator`.
These are source configuration and scripts, not a colcon package yet; no build
is needed. They reuse Clearpath's installed warehouse and simulation launch.

For an introduction to Gazebo, the proposed lift/cargo design and team sharing,
read the [robot design guide](../robot_description/README.md).

## Compact project warehouse

An optional primitive-only starter world is included in `worlds/cargo_warehouse.sdf`.
It has a 12 × 10 m interior, a dividing wall with a 1.60 m shortcut and a 2.40 m
alternative, two simple shelves and coloured pickup/delivery markers. The markers
are not physical pickup stands. Pallets and lift/attachment behaviour are still
future work. Xu can edit this file as the warehouse owner.

Close any existing simulation, then run from the repository root:

```bash
WAREHOUSE_LAYOUT=compact bash ros2_ws/src/warehouse_simulation/scripts/launch_warehouse.sh platform
```

The robot starts at (0, -3). Green marker: (-3, -3); blue marker: (-3, 3).
The shortcut is centred at (-3, 0); the wide opening at (3, 0). All coordinates
are world coordinates for scenario construction, not navigation goals before
SLAM frame alignment. No occupancy map or navigation algorithm is loaded.

This layout avoids detailed meshes and Fuel downloads, reducing rendering work;
it does not fix the WSL GPU-driver problem. The default launcher still selects
Clearpath's original warehouse. Both layouts use the same robot configuration.

## Teammate setup

After these source changes are pushed and checked out on another Ubuntu 24.04
machine with ROS 2 Jazzy installed:

```bash
sudo apt update
sudo apt install ros-jazzy-clearpath-simulator
source /opt/ros/jazzy/setup.bash
WAREHOUSE_LAYOUT=compact bash ros2_ws/src/warehouse_simulation/scripts/launch_warehouse.sh platform
```

On Windows, run inside the Ubuntu WSL distro with GUI support. No local file
paths from Thomas's home directory are needed. Team members run separate local
simulations; cloning the repository does not connect to a shared live Gazebo.
The repository remote is `https://github.com/yaqeen-2005/robotics-group-18.git`.

## Launch and drive

```bash
bash ros2_ws/src/warehouse_simulation/scripts/launch_warehouse.sh stock
```

Wait for `Configured and activated platform_velocity_controller`. The first
launch downloads Gazebo Fuel assets and can take considerably longer.
In Gazebo, select the Teleop panel, using topic `/r100_0000/cmd_vel`.
Use small forward, sideways and turning commands in the clear spawn area.
Release the controls to stop. No physical gamepad is required; the warning
about missing `/dev/input/ps4` is expected without one.

Stop the launch with Ctrl+C and wait for Gazebo to close before switching:

```bash
bash ros2_ws/src/warehouse_simulation/scripts/launch_warehouse.sh platform
```

Run only one instance at a time. If Gazebo survives launch shutdown, close its
window before restarting. The launcher requests shutdown of ROS when Gazebo
exits, but upstream shell-launched Gazebo processes can survive a forced stop.

The launcher copies the selected YAML from `../robot_description/config/`
to `.runtime/clearpath/{stock,platform}/robot.yaml`. Clearpath writes its
generated descriptions, launch files and parameters there. `.runtime/`, including
ROS logs, is Git-ignored. Edit the source YAML, then relaunch; do not edit generated files.

The underlying command is:

```bash
source /opt/ros/jazzy/setup.bash
ros2 launch clearpath_gz simulation.launch.py world:=warehouse \
  setup_path:="$PWD/.runtime/clearpath/stock" on_exit_shutdown:=true
```

The setup directory must already contain `robot.yaml` (the wrapper handles this).
Extra launch arguments can follow the wrapper's variant, for example `rviz:=true`.

## Verify

With the simulation running and teleop released, in a second terminal:

```bash
source /opt/ros/jazzy/setup.bash
python3 ros2_ws/src/warehouse_simulation/scripts/check_motion.py
```

This commands 0.15 m/s forward and sideways and 0.2 rad/s turning, each for
two simulated seconds, checks odometry response and LiDAR returns, and sends
zero velocity on exit. Only run it in a clear simulated area. It uses
`geometry_msgs/msg/TwistStamped` on `/r100_0000/cmd_vel`, as required by the
installed Clearpath version. Wheel odometry alone does not prove collision-free
physical movement, so also watch the robot move in Gazebo.

Visual acceptance checks for both variants:

- Warehouse geometry and a single Ridgeback appear, with wheels resting on the floor.
- Forward, sideways and turn commands move the chassis, and release stops it.
- The platform variant has a centred rectangular slab on the deck, with no
  intersection with the front LiDAR, wheels or body and no unstable bouncing.
- The sensor remains exposed below the slab; inspect from the side/front.

No map server, AMCL, Nav2 or SLAM is launched. Warehouse geometry is the physical
world, not a saved occupancy map. Clearpath's odometry EKF is active in `odom`;
the team can add SLAM later using `/r100_0000/sensors/lidar2d_0/scan`, odometry
and the namespaced TF topics. The motion check reports any occupancy-grid topics.

## Platform geometry

`ridgeback_stock.yaml` copies the installed Clearpath `r100_default.yaml`: R100,
namespace `r100_0000`, one front Hokuyo UST and the standard drive configuration.
`ridgeback_platform.yaml` adds only a fixed box, 0.70 × 0.60 × 0.04 m.

The installed R100 Xacro defines `default_mount` at z=0.295 m relative to
`base_link` (0.220 m riser origin + 0.075 m mount offset). The box centre is
0.020 m above that frame, giving a bottom at 0.295 m and top at 0.335 m.
Do not substitute the Husky example's `top_plate_link`.

The front LiDAR scan plane is z=0.1856+0.0474=0.2330 m, giving 62 mm vertical
clearance below the slab. Its collision box reaches z=0.2656 m and starts at
x=0.3622 m; the platform ends at x=0.3500 m. Thus the added geometry clears
both the scan plane and sensor collision body. The IMU is internal and unchanged.

Clearpath's `links.box` supplies visual and collision geometry, but **no added
mass/inertia**. This milestone validates fixed geometry and unloaded driving;
it does not validate payload dynamics, lifting, pallets or pickup stands.
See [Clearpath links configuration](https://docs.clearpathrobotics.com/docs/ros/config/yaml/links/).

## WSL performance

Check `glxinfo -B`: `llvmpipe` and `Accelerated: no` mean CPU rendering.
This machine exhibited that fallback, and a process-local test with
`GALLIUM_DRIVER=d3d12 glxinfo -B` failed to create a GLX context. Merely having
`/dev/dxg` and Mesa's D3D12 library installed did not enable acceleration.
No persistent graphics settings have been changed.

Keep only one simulator open; a small window can reduce GUI rendering cost.
Resolving the Windows/WSLg graphics driver path is still needed for smooth use.
Microsoft describes the GPU path and driver requirements in the
[WSLg documentation](https://github.com/microsoft/wslg) and
[GPU selection guide](https://github.com/microsoft/wslg/wiki/GPU-selection-in-WSLg).

## Recorded verification (2026-11-16)

Ubuntu 24.04.2 in WSL distro `Ubuntu`, ROS Jazzy, Gazebo Sim 8.15.0;
Clearpath simulator 2.9.4, config 2.9.8, platform description 2.9.17.
The repository's original Ubuntu 24.04.5 requirement differs from this tested host.

Stock: both controllers activated after a cached restart. Motion check measured
0.298 m forward, 0.306 m sideways, 0.398 rad turn; LiDAR had 720 rays and
690 finite returns. No occupancy-grid topics. Thomas confirmed physical driving
visually, but reported substantial lag. Sampled real-time factors were 0.52 and
0.78; these are snapshots, not a benchmark. The first cold launch had a controller
activation timeout. Logs are retained locally in `.runtime/`.

Platform: both controllers activated on a clean launch. Motion check measured
0.298 m forward, 0.299 m sideways and 0.398 rad turn; LiDAR again had 720 rays
and 690 finite returns, with no occupancy-grid topics. Generated SDF retained
both platform visual and collision boxes at z=0.315 m. Final visual inspection
of the platform remains for Thomas; the assistant cannot see the Gazebo window.
Gazebo's final physical pose was x=0.295 m, y=0.295 m, yaw=0.393 rad, confirming
chassis motion as well as wheel odometry. Performance remained poor: a later
platform sample reported real-time factor 0.061 (13.9 simulated seconds after
50.2 wall seconds). The milestone's motion checks pass, but smooth interactive
performance is unresolved.

Compact warehouse: SDF validation passed, passage collision edges were checked
at 1.60 and 2.40 m, and both controllers activated. The platform robot moved
0.298 m forward, 0.306 m sideways and 0.408 rad in the smoke test. Gazebo pose
was (0.295, -2.698) m with yaw 0.403 rad, consistent with the (0, -3) spawn.
LiDAR returned 720 rays, 690 finite, and no occupancy-grid topics were present.
A speed snapshot reported real-time factor 0.25; the simpler world still does
not resolve CPU-rendering lag. Visual inspection and autonomous route/load
experiments remain outstanding.
