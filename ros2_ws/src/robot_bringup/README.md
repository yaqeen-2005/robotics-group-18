# robot_bringup — Webots ↔ ROS 2 bridge (Sirui)

Drives the team's Webots pallet robot from ROS 2 Jazzy, as the base for Nav2.
It does not change anything under `webots/`: the launch file copies the
`webots/` folder to a runtime folder (default `.runtime/ros_bridge/`, ignored by
Git) and, in the copy only, sets the `TRANSPORT_ROBOT` controller to `<extern>`.
The facility supervisor (people, forklifts, orders, gates, spills) runs as usual.

## Setup (Windows + WSL2)

- Webots R2025a installed on Windows; Python on the Windows `PATH` for the facility controller.
- Ubuntu 24.04 in WSL2 with `ros-jazzy-desktop`, `ros-jazzy-webots-ros2`,
  `ros-jazzy-ros2-control`, `ros-jazzy-ros2-controllers`, `ros-jazzy-navigation2`,
  `ros-jazzy-nav2-bringup`, `ros-jazzy-slam-toolbox`, `ros-jazzy-teleop-twist-keyboard`.
- In WSL: `export WEBOTS_HOME=/mnt/c/<path to>/Webots`.
- `%USERPROFILE%\.wslconfig` needs `[wsl2]` / `dnsTunneling=false`. Otherwise
  `/etc/resolv.conf` holds 10.255.255.254 instead of the Windows host, and
  webots_ros2 cannot reach Webots ("Cannot connect to Webots instance").

## Build and run

```bash
cd ros2_ws
colcon build --symlink-install --packages-select robot_bringup
source install/setup.bash
ros2 launch robot_bringup webots_bridge.launch.py                      # final_warehouse.wbt
ros2 launch robot_bringup webots_bridge.launch.py world:=integration_test.wbt
```

Arguments: `world` (file in `webots/worlds/`), `mode` (`realtime`/`fast`/`pause`),
`repo` (auto-detected), `runtime` (copy location). Keep the runtime path short:
the facility controller writes `results/final/<run>/ground_truth.csv`, and
Windows rejects paths over 260 characters (seen as `FileNotFoundError`).

Drive by keyboard (Jazzy's diff-drive controller expects `TwistStamped`):

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -p stamped:=true
```

## Interface

| Topic | Type | Notes |
| --- | --- | --- |
| `/cmd_vel` | `geometry_msgs/TwistStamped` | diff_drive_controller; 0.5 s timeout stops the wheels |
| `/odom`, TF `odom→base_link` | `nav_msgs/Odometry` | wheel odometry only (slips on spills) |
| `/scan` | `sensor_msgs/LaserScan` | frame `front_lidar`, 180°, 12 m, 10 Hz; angle_min = −π/2, positive increment (`scan_normalizer` reverses Webots' `/scan_raw`, which slam_toolbox otherwise mirrors) |
| `/joint_states`, `/tf_static` | | robot_state_publisher uses the URDF Webots generates from the PROTO; `base_link→base_footprint` comes from a static publisher in the launch file |
| `/clock` | | simulation time; use `use_sim_time:=true` everywhere |
| `/warehouse/status` | `std_msgs/String` | JSON `warehouse_status` (latched, ~2 Hz) |
| `/warehouse/command` | `std_msgs/String` | JSON facility command, e.g. `{"op":"report","id":"spill_1","value":true}` |
| `/warehouse/command_result` | `std_msgs/String` | JSON reply with echoed `request_id` |
| `/lift/command` | `std_msgs/Float64` | target height 0–0.6 m; applied once wheels are still for 0.4 s |
| `/lift/height`, `/lift/moving` | `Float64`, `Bool` | do not send `/cmd_vel` while `moving` is true |

Frames: `base_link` on the drive axle (x forward), `base_footprint` 0.15 m below,
`front_lidar` at (0.52, 0, 0.13). Wheel radius 0.15 m, separation 0.82 m.
Facility coordinates in `/warehouse/status` are in the `warehouse` frame; a
transform to the SLAM `map` frame is still needed.

Lift and pickup *decisions* belong to the pallet-transport owner; this package
only exposes the motor. Verified on 2026-11-19 in `final_warehouse.wbt`: controllers
active, 0.4 m/s command moved odom ≈1.16 m, lift reached 0.20 m, status packets
and a `traffic` command round-trip received. `slam_toolbox online_async_launch.py`
built and saved a map with default parameters (scan orientation checked against the west wall).

## Files

- `resource/pallet_robot.urdf` — frames, ros2_control joints, LiDAR and plugins
- `config/ros2_control.yaml` — diff-drive geometry, turn calibration (`wheel_separation_multiplier` 1.115, measured against ground truth) and velocity/acceleration limits
- `robot_bringup/scan_normalizer.py` — `/scan_raw` → `/scan` with increasing angles
- `robot_bringup/warehouse_bridge.py` — warehouse radio and lift plugin
- `launch/webots_bridge.launch.py` — world copy, Webots, Ros2Supervisor (clock), driver, controllers
