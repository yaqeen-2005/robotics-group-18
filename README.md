# Cargo Aware Autonomous Pallet Transport Robot

Group 18: Peifeng Xu, Sirui Meng, Thomas Morris, Yaqeen Bharoochi, Yi Gu.

The active simulation is now **Webots R2025a with a native Python controller**.
ROS 2 is not needed to open the warehouse or manually drive and lift.

Open [`webots/worlds/warehouse.wbt`](webots/worlds/warehouse.wbt) in Webots.
Read the [setup and manual acceptance checklist](webots/README.md) first.

For the final project environment, open [Meridian distribution hall](webots/worlds/final_warehouse.wbt).
The [final-world operator guide](webots/FINAL_WAREHOUSE.md) includes a floor plan, pickup instructions, team interfaces and the validated delivery run.

For team feature integration, open
[`webots/worlds/integration_test.wbt`](webots/worlds/integration_test.wbt).
This larger test warehouse adds varied cargo, moving people/forklift, spill
reporting, controlled blockages and a pallet output bay. See the
[integration run guide and interfaces](webots/INTEGRATION_TEST.md).

## Current implementation

- Compact warehouse with a narrow shortcut, wide alternative, shelves and stands.
- Custom differential-drive robot with physical wheels, low-friction supports,
  a motorised platform, collision geometry and mass.
- Reusable normal/wide pallet models and a front LiDAR device.
- Keyboard driving and lift control, with a stationary check before lift commands.

**Validation is partial.** Subsequent Webots checks reproduced and corrected
lift startup oscillation, then checked settling and three empty lift cycles.
Driving and loaded handling still require the manual acceptance checklist;
see `webots/README.md` for the test scope.

The robot can attempt to raise a pallet through physical contact; it does not
have an attachment mechanism yet. Mapping, localisation, autonomous navigation,
confirmed cargo-state coordination and comparative trials are not integrated.
The robot controller has no privileged world-pose access or saved occupancy map.

## Repository layout and ownership

- `webots/`: active world, robot, pallets and manual bring-up controller.
- `ros2_ws/src/navigation/`: existing planner, map attributes and event-store work,
  retained in place and unchanged. It is not connected to the Webots controller.
- Other existing ROS directories: retained for their owners and historical reference.
- `ros2_ws/src/robot_description/` and `warehouse_simulation/`: legacy Gazebo prototype.
- `docs/archive/README-gazebo.md`: previous project README, preserved verbatim.

Thomas owns physical robot/lift/load geometry. Xu owns warehouse/exploration/tasks;
Sirui owns integration and cargo-aware planning. Yi and Yaqeen retain their own
components; their implementation was not migrated or modified in this change.

The project objective remains mapping an initially unknown warehouse, completing
pallet deliveries and adapting clearance/motion to cargo. Team planning should
now account for native Webots interfaces; ROS/Nav2 integration is no longer an
assumption of this prototype. Reused and AI-assisted work must be recorded in
contribution/report declarations.
