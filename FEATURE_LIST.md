# Feature and Contribution List

This file records the features implemented by each group member. It can be used as a reference when preparing the final presentation, report, or viva.

## Peifeng Xu:

## Sirui Meng:

## Thomas Morris:

## Yaqeen Bharoochi:

## Yi Gu:

- Implemented a basic A* path planner for finding the shortest available route on a known two-dimensional grid map.
- Added obstacle detection during path planning so that blocked and unknown cells are excluded from the generated path.
- Defined a simple planner interface that accepts a grid map, a known start position, and a goal position, and returns an ordered list of path coordinates.
- Added a versioned `MAP v1` text format and a map reader that validates the map dimensions, symbols, start point, and goal point.
- Added an independent `ATTRIBUTES v1` component that stores map confidence, historical blockage probability, and human encounter probability for every cell.
- Added validation and an update interface for the three per-cell attributes. These values are not yet used by the A* algorithm.
- Added Matplotlib visualisation for the map, start point, goal point, obstacles, and calculated route.
- Added command-line execution with optional visual output, together with a shell wrapper for Linux.
- Added unit tests and example map data for checking successful pathfinding, unreachable goals, and matching confidence-grid dimensions.
- Added a SQLite event store for recording robot, task, run, map, position, and event-specific data.
- Added five initial navigation event types and straight-line visited-path segments for later visit, congestion, human-presence, rerouting, and bottleneck analysis.

### Usage

From `ros2_ws/src/navigation`, show the default map and calculated path:

```bash
python run.py
```

Print the path without opening the visualisation:

```bash
python run.py --no-visual
```

Use a custom map:

```bash
python run.py my_map.txt
```

Run the automated tests:

```bash
python -m unittest discover -s tests -v
```

Display a map without running A*:

```bash
python tests/show_map.py tests/maps/demo_map.txt tests/maps/demo_attributes.txt
```

### Current limitations

- The planner currently requires an existing map and does not create a map while exploring an unknown environment.
- Movement is currently limited to four directions: up, down, left, and right.
- Per-cell attributes are validated, loaded, and visualised, but they do not currently affect route selection.
- ROS 2 message conversion and integration with the robot, localisation, and Nav2 have not yet been implemented.
