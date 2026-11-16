# Robot design and team handover

> Legacy Ridgeback/Gazebo design. The active custom differential-drive robot is
> documented in the [Webots guide](../../../webots/README.md).

The latest cargo-aware proposal supersedes the older responsibilities in the
original project README. Thomas owns robot/lift/physical attachment; Xu owns
the warehouse and task sequence; Yi owns SLAM/localisation; Sirui owns combined
launch and cargo-aware planning; Yaqeen owns sensing, odometry and motion limits.

## How the simulation is built

Gazebo runs the physical simulation: contact, gravity, joints and simulated
sensors. Its GUI displays that simulation and provides inspection and manual
controls. ROS nodes implement sensing interfaces, mapping, planning and tasks.

- SDF (`.sdf`) describes the world, models, collision shapes and simulation plugins.
- URDF/Xacro describes robot links and joints. Xacro adds reusable macros and
  parameters; Clearpath generates it from our `robot.yaml` configuration.
- YAML holds configuration. Python launch files start Gazebo and the ROS nodes.
- Python/C++ nodes implement behaviour; a visible model alone cannot navigate.

Use version-controlled files as the source of truth. The GUI is useful for
viewing, selecting objects, inspecting poses and trying layouts. An interactive
change is not automatically a change to this repository. Reproduce useful
changes in the source world/model and relaunch. CAD is unnecessary for simple
boxes, wheels, platforms and fixtures.

## Implemented foundation

We reuse the Clearpath Ridgeback R100 chassis, mecanum wheels, controller,
odometry, IMU and front Hokuyo LiDAR. `config/ridgeback_stock.yaml` is the baseline;
`config/ridgeback_platform.yaml` adds a fixed 0.70 × 0.60 × 0.04 m platform.
See the [run guide](../warehouse_simulation/README.md) for tested geometry and results.

This is an adapted commercial base, not a robot designed entirely from scratch.
The plan's phrase “custom wheeled robot” should be interpreted explicitly with
the team: a customised Ridgeback saves substantial wheel/controller work. If
the assessment requires an original chassis, confirm that before replacing it.
The proposal itself does not require inventing a new drive base.

## Next robot milestones (not implemented yet)

1. Replace the fixed slab with a controlled vertical prismatic joint and platform
   in a custom Xacro extension. Include mass, inertia and hard travel limits.
   Keep the existing wheel controller; integrate a separate lift controller using
   the installed Gazebo/ros2_control interfaces. Choose stroke and carrying height
   together with the pickup fixtures, rather than arbitrarily lifting the robot.
2. Create simple dynamic pallet SDF models with collision shapes matching their
   full overhang, mass and inertia. Add fixed stands that support the pallets
   while leaving room for the chassis and lowered platform underneath.
3. Implement bounded pickup/release operations. Check station alignment, distance,
   low measured speed and lift state; confirm attachment only after the simulator
   actually establishes it. A rigid attachment is acceptable for this proposal.
   Test release without dropping the pallet through the floor or supporting stand.
4. Publish confirmed load state and geometry. Xu sequences operations; Sirui and
   Yaqeen acknowledge geometry and motion-limit changes before movement resumes.
   Failure or timeout leaves the robot stopped. A command request is not confirmation.
5. Verify repeated pickup/release, incorrect alignment, moving pickup rejection,
   sensor clearance with each cargo type, and stable loaded turns. Record limitations.

The current `links.box` slab has collision geometry but no added inertia. Do not
use the current motion check as evidence for payload dynamics or lifting.

## Cargo and passage design

Use metres, radians and seconds, with +x forward, +y left and +z up in `base_link`.
Keep `base_link` fixed to the chassis across loading. Define load poses relative
to it, including any offset and overhang. Normal/wide are geometry profiles;
fragile is a handling profile that can apply to either geometry.

For a centred rectangular envelope of length L and width W, a conservative
planning radius is `sqrt((L/2)^2 + (W/2)^2) + margin`. For offset loads or multiple
shapes, take the maximum distance from `base_link` to every footprint vertex,
then add the margin. Use the union of robot and load; the platform size alone
is not the empty robot footprint.

Illustrative design values, **not measured or active navigation configuration**:

| Item | Length × width | Circle diameter with 0.05 m margin |
| --- | --- | --- |
| Provisional empty envelope | 1.10 × 0.90 m | 1.52 m |
| Normal pallet | 0.80 × 0.60 m | Empty robot still governs if centred |
| Wide pallet | 1.20 × 1.40 m | 1.94 m |

The compact world has a 1.60 m shortcut and a 2.40 m alternative. These illustrate
empty-versus-loaded feasibility under a circular planner. Measure the actual
complete collision envelope (including wheels and sensors), confirm the safety
margin with Sirui and verify discretisation effects before freezing corridor
sizes. A rectangular robot can physically fit where the conservative circular
planner refuses a route; explain this in evaluation.

Only one node should own each TF transform. Clearpath currently supplies local
odometry; Yi adds `map -> odom`. Agree namespaced `/r100_0000/tf` and `tf_static`
handling with Sirui. Keep Gazebo ground truth out of navigation inputs. Existing
topics are listed in the run guide; future lift/load command names are not frozen.

## Sharing with teammates

These changes are currently local working-tree files. They are not on GitHub
until committed and pushed. Review `git diff` and untracked files, create a
feature branch, commit the source files and push it for team review. Generated
`.runtime/` files stay local. Each teammate checks out that branch/merged commit,
installs the compatible dependencies and runs the same launcher on their own PC.

This gives everyone the same reproducible simulation, not automatic access to
Thomas's running Gazebo window. A live shared simulator would require deliberate
network/ROS discovery configuration; it is unnecessary for normal collaboration.
Each developer should use their own ROS domain on a shared network, consistently
across their terminals, to avoid accidentally controlling another simulation.

Keep the tested package versions in the run guide and record version changes.
Different package versions can change generated controllers or models. The compact
world is stored in Git and needs no Fuel assets; Clearpath's warehouse downloads
assets on first use. Robot assets still come from the installed Clearpath packages.
