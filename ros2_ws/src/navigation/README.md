# Navigation prototype

Run the default map with a plot from this directory:

```bash
python -m astar.run
```

The source is organised into:

- `astar`: path planning on an existing map
- `exploration`: future exploration of an unknown map
- `common`: shared map, cell-attribute, and event data for navigation and ML

Run the deterministic dynamic demonstration from this directory:

```bash
python -m tests.dynamic_navigation_demo --show --tasks 3 --seed 18
```

The Matplotlib window shows a 24 x 16 warehouse with black walls and racks, randomized pickup A and delivery B locations, a blue robot, three red people, and the current route. The planner considers only the nearest person on the remaining route and ignores that person while farther than the default six-cell attention distance. The gold-outlined person is the nearest one on the route; a dark-red person is close enough to affect planning. Change the future trainable threshold with `--attention-distance 4`. During a human wait the robot becomes a larger orange square and the window displays `WAITING FOR PERSON`. Save the same animation with `--save-gif .runtime/dynamic_navigation.gif`.

The scenario assumes SLAM supplies a correct pose. A person moves onto the initial route, the robot waits while the linear score is below its threshold, then reroutes through the second passage and reaches the goal. The script uses the real `EventStore`, attribute updater, `PathManager`, and A* components; it does not simulate sensors or wheel motion.

Print the path without a plot:

```bash
python -m astar.run --no-visual
```

Pass a different map to the planner:

```bash
python -m astar.run my_map.txt
```

Load the matching attributes and an editable trained profile during local testing:

```bash
python -m astar.run tests/maps/demo_map.txt --attributes tests/maps/demo_attributes.txt --weights ml/weights/human_safe.json --no-visual
```

The map header is `MAP v1 <width> <height>`. All map cells are flattened onto the next line in row-major order. Map symbols are `S` (start), `G` (goal), `.` (free), `#` (obstacle), and `?` (unknown/blocked).

The separate attribute file accepts legacy `ATTRIBUTES v2 <width> <height> 4` and new `ATTRIBUTES v3 <width> <height> 10`. The original four fields keep their order. The six appended fields are observation count, block count, block rate, switch count, last state (-1 unknown, 0 free, 1 blocked), and door belief. Old four-field maps supply temporal defaults on demand. Counts are non-negative integers; probabilities are in [0, 1].

`TemporalTraversabilityLayer.update(timestamp, observations)` consumes only currently valid sensor observations (`{(x, y): True/False}`), at most once per configured interval (0.5 seconds by default). It never interprets a cached SLAM map as a new observation. `DoorPredictor` loads the trained CNN from `ml/weights/door_belief.pt`; `layer.predict(predictor, cells)` writes its probabilities to the feature map. PyTorch and NumPy are needed for this optional prediction module. Reproduce the 96×96 synthetic scenes (0.1 m per cell), 11×11 patches, compressed data archive and trained model with `python -m ml.door_belief --samples 6000 --epochs 12`; train the irregular-map door routing profile with `python -m ml.train_special_navigation`. The held-out synthetic classification accuracy is 99.9%, which does not establish real-world detection accuracy. The set includes horizontal, vertical, and several slanted multi-cell obstacles with irregular shelf edges.

Show the combined person and door demonstration with `python -m tests.door_human_demo`, or export it with `python -m tests.door_human_demo --save-gif door_human_demo.gif`. The gray dashed line comes from `PathManager.raw_path()` and shows the unsmoothed A* route; the green line contains only positions the simulated robot has actually reached. The blue circle is its sensor range. People move between cells; those outside range are gray, those inside range but behind walls or closed doors are hidden, and only directly observed people are red. The occupancy display is grayed out outside line of sight, including behind walls. The outbound trip waits for a person and reroutes through lower doors. The middle trip executes a 30° staircase shortcut. The return trip also executes smoothed waypoints through both upper doors rather than stepping along the raw staircase. A final single-corner trip executes a collision-checked 45° hypotenuse. The door counter records four actual gate crossings.

The A* cost combines distance, the original risk attributes, and, when present, door belief multiplied by observed block rate. A repeated cell can also incur a separate cost. Default weights are defined at the top of `astar/a_star.py`; optional profiles can override them. The synthetic door experiment is in `ml/weights/special_navigation.json`.

Display the map and all four attribute layers:

```bash
python tests/show_map.py tests/maps/demo_map.txt tests/maps/demo_attributes.txt
```

The planner interface is `astar.a_star.plan_path(grid, start, goal)`. Coordinates use `(x, y)` and map cells use `grid[y][x]`.

## Clearance and route state

Create an inflated test map with a clearance radius measured in grid cells:

```bash
python -m common.inflate_map tests/maps/demo_map.txt tests/maps/demo_attributes.txt --cells 2 --output-map inflated_map.txt --output-attributes inflated_attributes.txt
```

The inflation component is separate from A*. Do not apply it again when the input is already an inflated Nav2 costmap. Expanded cells copy the nearest obstacle's attributes to keep the files aligned, although blocked-cell attributes do not affect A*.

For a fresh, uninflated occupancy grid, include currently observed person cells in the same pass as walls: `inflate_map(grid, inflation_cells=1, dynamic_obstacles=person_cells, protected_points={robot_position})`. A radius of one cell is the default example safety margin; choose the actual radius from map resolution and robot clearance. Pass the resulting grid to `PathManager.plan()` along with `human_cells=person_cells`. Do not inflate an already inflated static map again. The output is temporary for that sensor update; the stored static map is unchanged.

A* keeps the arrival direction internally and adds the configurable `turn` weight when a route changes direction. `FrontierExplorer.next_move()` also returns `heading_rad` and, when the current yaw is supplied to `update_position`, `turn_rad`. `common.path_smoothing.smooth_path(grid, path)` scans for a left-to-right or right-to-left turn reversal. From that point it tests 30°/60° rays for an exact intersection with a later route segment and checks the whole shortcut against the occupancy grid. A route with just one right-angle turn instead tries a 45° hypotenuse with equal legs along its incoming and outgoing straight runs. Unsafe shortcuts fall back to the original grid route. `PathManager.plan()` returns both the original grid `path` for replanning and collision-checked `waypoints` for execution; `raw_path()` provides a separate copy of the unsmoothed route, and `WAIT` returns no executable waypoints. `sample_waypoints(waypoints, step)` generates continuous positions for the simulated follower. Run smoothing against an inflated map when robot width matters; wheel dynamics and real controller integration remain separate.

`common.path_manager.PathManager` calls A* on every update but retains the active route unless a non-human map change is explicitly reported. A human obstruction is handled separately by a configurable linear score using human distance, waiting time, and reroute count. It returns `WAIT` below the threshold, or tries a route that avoids the remaining old path above the threshold.

`common.special_obstacles.SpecialObstacles` registers the human and door-like rules. A door is considered only when its belief and proximity pass thresholds; its observed open rate determines whether waiting is reasonable. Both rules use the same rerouting branch. `PathManager.backtrack_counts` tracks actual returns to previously visited cells, while `repeat_counts` additionally penalises cells on abandoned routes; these are different measures. If the old route can be avoided, it is excluded; otherwise A* may reuse it with a penalty. The trained thresholds are only calibrated on synthetic examples. No controller loop or automatic live sensor ingestion is included here.

When a person and a door are detected together, each rule votes independently. Any `REROUTE` vote produces one route decision using the union of all active special-obstacle cells (including obstacles whose rule voted `WAIT`). If neither votes to reroute but one votes `WAIT`, the robot waits. The processed occupancy grid also blocks the inflated person footprint.

## Event history

`EventStore` records `CELL_VISITED`, `HUMAN_DETECTED`, `PATH_BLOCKED`, `REROUTE`, `BACKTRACK`, and `ROBOT_ENCOUNTERED` events in SQLite. Every event includes a `robot_id`, `run_id`, `map_id`, timestamp, and position. Optional fields are `task_id` and JSON `details`.

For the MVP, a visited route is stored as consecutive straight line segments. Record the completed segment whenever the robot turns or stops:

```python
from common.event_store import EventStore

events = EventStore()
events.record_visited_segment("robot-1", "run-1", "map-1", (1, 2), (5, 2))
```

The default development database is `tests/events.db`.

The sensor side may record a person without knowing the navigation decision:

```python
event_id = events.record_human_detection(
    "robot-1", "run-1", "map-1", (4, 7), distance=2.1
)
```

After `PathManager` returns, the future runtime loop can attach the decision without creating a duplicate detection:

```python
events.annotate_event(event_id, decision="WAIT", reroute_score=0.42)
```

Apply every new database event to the attribute map exactly once:

```python
from common.event_attributes import apply_new_events

result = apply_new_events(events, attributes, "map-1")
if result["human_detected"]:
    print("A new human event was received")
```

The caller saves the modified attribute map after the batch. No robot control loop is included in this component.

The same batch operation is available from the command line:

```bash
python -m common.event_attributes tests/events.db tests/maps/demo_map.txt tests/maps/demo_attributes.txt map-1 --output updated_attributes.txt
```

Its JSON output includes `human_detected`, which is true when the processed batch contains a new `HUMAN_DETECTED` event.

Produce a JSON report of cells where people are frequently detected and cause waiting or rerouting:

```bash
python -m common.human_activity_report tests/events.db --map-id map-1 --output human_activity.json
```
