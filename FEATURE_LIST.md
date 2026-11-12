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
- Added a matching `CONFIDENCE v1` grid format for storing a confidence estimate between `0.0` and `1.0` for every map cell.
- Added validation to ensure that the map and confidence grid contain the same number of corresponding cells. The confidence values are not yet used by the A* algorithm.
- Added Matplotlib visualisation for the map, start point, goal point, obstacles, and calculated route.
- Added command-line execution with optional visual output, together with a shell wrapper for Linux.
- Added unit tests and example map data for checking successful pathfinding, unreachable goals, and matching confidence-grid dimensions.

### Usage

From `ros2_ws/src/navigation`, show the default map and calculated path:

```bash
python run.py
```

Print the path without opening the visualisation:

```bash
python run.py --no-visual
```

Use a custom map and matching confidence file:

```bash
python run.py my_map.txt my_confidence.txt
```

Run the automated tests:

```bash
python -m unittest discover -s tests -v
```

Display a map without running A*:

```bash
python tests/show_map.py tests/maps/demo_map.txt
```

### Current limitations

- The planner currently requires an existing map and does not create a map while exploring an unknown environment.
- Movement is currently limited to four directions: up, down, left, and right.
- Confidence values are validated and loaded, but they do not currently affect route selection.
- ROS 2 message conversion and integration with the robot, localisation, and Nav2 have not yet been implemented.
