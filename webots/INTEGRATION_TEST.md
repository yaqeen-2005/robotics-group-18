# Integration test warehouse

Open **`webots/worlds/integration_test.wbt`** in Webots, press Play at real-time
speed, and click the 3D view. The original `warehouse.wbt` remains available for
small component tests. Both worlds use the same robot PROTO and manual controller.

This is a 24 × 18 m intermediate warehouse for team development. It supplies
physical obstacles, loads, scenario controls and interfaces. Teammates connect
their sensing, SLAM, planning, motion control and delivery software to it.
The supplied robot still uses manual control and contact-only pallet support.

## Controls

| Key | Action |
| --- | --- |
| W / S, A / D | Existing forward/reverse and turning controls |
| Hold U / J | Lift up/down; release to hold height |
| Space / R | Stop robot and hold lift / print lift measurement |
| 1 / 2 / 3 | Toggle spill 1 / 2 / 3 between reported and not reported |
| 4 / 5 / 6 | Toggle physical presence of spill 1 / 2 / 3 |
| T | Freeze/resume people and forklift at their current positions |
| N | Enable/disable seeded random spill presence changes |
| G | Close/open the narrow passage gate |
| H | Close/open the wide passage gate |
| B | Close the wide passage for 20 simulated seconds, then reopen |
| F | Select the next storage-output load profile (shown on screen) |
| O | Request the selected pallet at the storage-output station |

Tap scenario keys; movement/lift keys are held. The on-screen status and console
show the scenario state. An absent spill cannot be reported. Spawning a pallet
or lowering a gate into an occupied area is rejected; move the obstruction
clear before retrying. A manual H command cancels the temporary gate timer.
Reload the world from disk for a fresh run with the same seed and initial poses.
Do not save moved pallets over the baseline world.

## Layout and loads

Coordinates are metres, +x right/east, +y up/north in the overhead view, +z up.
The robot starts at (-11, -6), facing the standard pallet. Enter pickup and
delivery stands along the x axis, with the platform lowered and centred.
Rails leave 0.96 m internal width and have 0.48 m tops.

The divider at y=0 has a 1.60 m shortcut centred at x=-6 and a 3.20 m wide
passage at x=6. Shelves create detours and scan landmarks. A crate, column and
southern dead-end bay add obstacle and recovery cases. A marked waiting bay is
at (-10.8, -8). Three empty delivery stands are at (-4, 6.5), (2, 6.5), (8, 6.5).
The storage output is at (-9, 3); approach it from the east facing -x.
Its facade/conveyor represents the upstream storage machinery. New pallets
appear stationary on the rails; it does not simulate conveyor transport.

| Profile | Deck length × width | Cargo height above deck | Total mass | Initial station | Purpose |
| --- | --- | --- | --- | --- | --- |
| Standard | 0.80 × 1.20 m | 0.32 m | 10 kg | (-9, -6) | Reference load |
| Wide | 1.20 × 1.80 m | 0.40 m | 18 kg | (-4.5, -6) | Overhang and clearance rerouting |
| Long | 1.80 × 1.20 m | 0.32 m | 20 kg | (0, -6) | Swept turning clearance |
| Heavy | 0.80 × 1.20 m | 0.32 m | 40 kg | (4.5, -6) | Same geometry as standard, different mass |
| Fragile | 0.80 × 1.20 m | 0.70 m | 8 kg | (9, -6) | Tall load and handling metadata |

Every deck is 0.08 m thick. Deck and cargo are one rigid body with collision
geometry and the listed combined mass. Webots derives inertia from that geometry.
Fragility is a catalogue property; no breakage or cargo-damage model is included.
Teammates must implement and justify speed/acceleration/turn limits from metadata.
Mass changes physics but does not automatically change the controller settings.

`config/load_catalog.json` contains geometry, mass, handling and conservative
clearance estimates. Centred enclosing-circle diameters with a 0.05 m margin:
empty ≈1.39 m, standard/heavy/fragile ≈1.65 m, wide/long ≈2.26 m. Thus the
circle-based planner should use the shortcut only while empty. The wide pallet
also physically exceeds the shortcut's width in its initial orientation.
Recompute combined bounds for real attachment offsets; never mark a robot loaded
solely because a pallet was requested or spawned.

## People, forklift and spills

Two visible people shuttle across walkways; a forklift shuttles through the
wide passage and reverses on its return leg. Their collision shapes are visible
to sensors. The manager moves them on repeatable paths with endpoint dwell.
They pause before a closed gate. They are prescribed kinematic obstacles:
they do not yield to the robot, simulate human decisions or realistic impact
dynamics. Robot avoidance and stopping are features for teammates to test.

Spills are flat visual hazard patches: amber = present/unreported, red =
present/reported, absent = hidden. They do not form solid barriers or change
friction. A horizontal LiDAR will not automatically see these patches. Reported
spill rectangles must be inserted into the planner's forbidden-area layer and
expanded by the appropriate robot/load clearance. Changing a report does not
currently make the manual robot avoid the patch.

Initial state: spill 1 (-6,1.4) is present/unreported; spill 2 (6,-1.5) is
present/reported; spill 3 (0,5) is absent. For a clean geometry comparison,
turn off all reports and freeze traffic first. Existing unreported patches then
remain visible to the operator but are absent from the report interface.

Random spill mode is OFF by default. While enabled, each spill independently
has a 0.15 chance to toggle presence every 10 simulated seconds. New spills
start unreported; removal clears the report. Reporting itself is manual or an
explicit external command. Seed 18 reproduces the sequence when commands and
timing are matched. T freezes traffic only; N controls the random-spill clock.
These parameters are in `config/integration_scenario.json`.

## Team interfaces

The transport robot has optional devices in its `extensionSlot`:

- `warehouse_events`: Receiver on channel **18**, UTF-8 JSON.
- `warehouse_commands`: Emitter on channel **19**, UTF-8 JSON.

The original robot controller does not enable/consume these devices. Integrate
the following pattern into a teammate controller, using its existing Robot
instance and step loop (do not instantiate a second Robot):

```python
import json
rx = robot.getDevice('warehouse_events')
rx.enable(step_ms)
tx = robot.getDevice('warehouse_commands')

# Inside the controller's step loop:
while rx.getQueueLength():
    message = json.loads(rx.getString())
    if message['type'] == 'warehouse_status':
        reports = message['reported_spills']  # Full replacement snapshot.
        # Transform to map frame and update your forbidden-area layer here.
    elif message['type'] == 'command_result':
        print(message)  # ok, optional reason, and echoed request_id
    rx.nextPacket()

# Example request (send once, not on every step):
tx.send(json.dumps({'op': 'report', 'id': 'spill_1',
                    'value': True, 'request_id': 'report-001'}).encode())
```

Status snapshots arrive about twice per simulated second and after commands:
`schema_version`, `type`, `seq`, `simulation_time_s`, `frame: "warehouse"`,
`reported_spills: [{id, center:[x,y], size:[length,width]}]`, `output_profile`,
and `output_pallets: [{id,profile}]`. The output list identifies pallets spawned
this run; it does not certify that they remain at the station, are attached,
or have been delivered. These require the team's task/handling interfaces.
Replacing the report list removes cleared/unreported hazards, including when
the new list is empty. Do not append reports indefinitely.

| Command `op` | Other fields |
| --- | --- |
| `report` | `id`: spill_1/2/3, `value`: true/false |
| `presence` | `id`: spill_1/2/3, `value`: true/false (test orchestration) |
| `traffic`, `random_spills` | `value`: true/false |
| `gate` | `id`: narrow/wide, `value`: true closes, false opens |
| `temporary_blockage` | none |
| `select_load` | `id`: standard/wide/long/heavy/fragile |
| `spawn` | none; requests selected profile |

Commands may include `request_id`. The response echoes it with `ok` and a
rejection reason when relevant. Channels are a local simulation interface, not
a ROS bridge. The scene manager is the only Supervisor. It handles the environment
and evaluation; the robot retains ordinary Robot privileges.

Infrastructure reports use the warehouse frame. Agree a legitimate transform
to Yi's SLAM map frame before using them. This may be a known initial alignment
or calibrated station reference; do not silently copy simulator robot pose into
localisation. Station coordinates in this guide are setup/evaluation information;
station discovery and the mapping-to-delivery transition remain team features.
Unreported spills, actor poses and the ground-truth layout are not broadcast.

## Suggested test sequence

1. **Model and sensing:** drive, jog lift, inspect scans around shelves, people,
   forklift and each load. Check self-observation/occlusion with the taller cargo.
2. **Mapping:** freeze traffic with T for initial SLAM integration, then repeat
   with moving traffic. Save/load the SLAM-generated map and revisit a loop.
3. **Geometry:** unreport spill 2 with 2; compare empty and wide/long loaded
   routes between the southern and northern halves. Verify actual planner
   clearance and local footprints, including turns. Use the same map/goals.
4. **Motion:** compare standard, heavy and fragile handling on matched routes.
   Record measured motion and failures; use attachment once implemented.
5. **Obstacles:** enable traffic, exercise stopping/replanning, trigger B while
   the gate area is clear, and test a longer closure using H. Close G and H
   together for a no-route case. Check bounded waits and failure reporting.
6. **Hazards:** toggle 1/2/3 while a route is active; verify report addition and
   removal changes the forbidden-area layer. Then enable N for seeded runs.
7. **Delivery:** F/O supply different requested profiles, collect, transport,
   unload at a destination and return. Attempt O with an occupied output bay.

Teammates first demonstrate their own component and supply expected outcomes.
Thomas runs the integration checks and records failures against the code version.
Keep failed trials. Avoid calling contact-only pickup a confirmed attachment.

## Logs, editing and validation

Each run creates `webots/results/integration/<UTC timestamp>/` containing:

- `scenario.json`: configuration used for the run.
- `events.jsonl`: timestamped commands, rejections, spawn/gate/random events.
- `reported_state.json`: latest public status.
- `evaluation_state.json`: full spill/gate state for evaluation only.
- `ground_truth.csv`: robot, actor and pallet positions for evaluation only.

`results/integration/latest.json` identifies the latest run. Results are ignored
by Git. These are scenario/evaluation logs, not finished delivery, collision,
acceleration or localisation-error metrics; the team supplies those producers.

Edit `config/integration_scenario.json` for profiles, spawn stations, actor paths
and spill settings. The fixed rack/wall layout is in
`tools/build_integration_world.py`. Regenerate after changes:

```powershell
python webots/tools/build_integration_world.py
python -m unittest discover -s webots/tests -v
python webots/tools/smoke_integration.py
```

The generator writes only the new world, its evaluation layout, load catalogue
and local label textures. Generating missing label textures requires Pillow;
the checked-in world and controllers need only Python and Webots. Never import
the old `build_world.py` as a helper: it writes the original world on import.
The smoke runner launches a separate Webots instance against a timestamped copy
under `.runtime/`, runs 25 simulated seconds and verifies file-based results.
Use `--webots PATH` if Webots is installed elsewhere. It substitutes a test
controller only in that copy and leaves the current interactive world alone.

Validation on 16 November 2026: four automated tests passed (report filtering,
seeded ticks across step sizes, continuous traffic/pause state, route clearance
and no-route geometry). A 25-second Webots smoke run passed load settling,
moving traffic, reporting/presence changes, gate changes/timed reopening and
output spawning/occupied-bay rejection. This does not establish autonomous
navigation, loaded transport stability or planner response to reported hazards.
An additional controller verified radio status reception, report filtering,
command acknowledgement and finite LiDAR obstacle returns in Webots.
