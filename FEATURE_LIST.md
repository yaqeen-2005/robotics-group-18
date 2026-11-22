# Feature and Contribution List

This file records the features implemented by each group member. It can be used as a reference when preparing the final presentation, report, or viva.

## Peifeng Xu:

## Sirui Meng:

## Thomas Morris:

- Started the project's move to Webots and set up the directory structure for worlds, reusable models, textures and Python controllers. This gave the team a common simulator setup for testing individual components and integrating them later.
- Modelled and textured the first 12 m by 10 m test warehouse. Added normal and wide pallets, pickup and drop-off stands, and narrow and wide passages so the team could check basic driving, lifting, pallet clearance and collisions in a small, repeatable layout.
- Built the robot model with independently driven wheels, physical collision shapes, wheel encoders, a front LiDAR mount and a central telescopic lift. Wrote the Python keyboard controller for driving, turning and raising or lowering the platform. The controller requires the wheels to stop before the lift moves and holds the platform at its current height when the lift key is released.
- Built and textured a 24 m by 18 m integration warehouse for testing team features together. Added storage racks, a dead-end bay, narrow and wide routes, a selectable pallet outfeed and three drop-off stands. Defined five load profiles with different dimensions, masses or handling requirements for clearance and motion-control trials.
- Modelled the moving people, forklift, gates and spills and wrote the controller that runs those scenarios. It lets testers pause traffic, close either gate, introduce a timed blockage and control whether a spill is present or reported. The robot's radio interface sends reported spill locations to the navigation controller so the team can test treating them as forbidden areas.
- Created the final 96 m by 64 m warehouse with separate picking, packing and stock areas, open loading floors and walled corridors. The robot can take a narrow shortcut when empty or use the wider perimeter roads when carrying a large pallet.
- Added three roller pickup lanes to the storage wall. Each lane supplies a random pallet from five size, weight and handling profiles and assigns it to one of six drop off locations.
- Added matching lights to the pickup and drop off locations, a display showing which parcel the robot is carrying, and pickup and delivery messages. Delivered pallets fade from the receiving stand.
- Added seven people who walk within their work areas, give forklifts priority and return to a standing pose when they stop. Two forklifts move their own pallets between shelves and sampled floor locations, then return to collect them. Their routes are reserved to avoid conflicting trips.
- Added timed gates and gates that open for approaching traffic. Spills can appear and be reported randomly or reported manually, and reported spills can be cleaned up while keeping between two and five spills active.
- Tested a complete pickup and delivery in Webots, checked all five pallets on all six drop off stands, and checked that a loaded route remains available with all gates closed and all possible spills blocked.
- Added slippery tyre contact for active spills and tested wheel encoder distance against actual robot travel on dry and wet floor.
- Added recessed exterior walls, three marked entrance widths for cargo clearance trials, pallet stacks in the north stores, and wall-side workstations with chairs.

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
- Added a lightweight YOLO26n camera component that returns person bounding boxes and confidence scores while keeping Webots and navigation integration separate.
- Added basic frontier exploration that falls back from an unreachable A* goal to the nearest reachable unknown or low-confidence map boundary.
- Added risk-aware A* costs for historical blockage, human presence, map uncertainty, and encounters with other robots, with replaceable weights for future ML tuning.
- Added reproducible factory-map templates with synthetic human and robot trajectories for future path-weight experiments.
- Added a separate obstacle-inflation tool that accepts a clearance radius in grid cells and writes matching processed map and attribute files.
- Added a configurable A* turn penalty so later training can prefer routes with fewer direction changes, while keeping the public path format unchanged.
- Added next-heading and relative-turn outputs for handing grid routes to a downstream motion controller.
- Added route-state management that regenerates A* candidates on every update, immediately adopts routes after non-human map changes, and handles human obstructions through a configurable linear wait/reroute decision.

### Usage

From `ros2_ws/src/navigation`, show the default map and calculated path:

```bash
python -m astar.run
```

Print the path without opening the visualisation:

```bash
python -m astar.run --no-visual
```

Use a custom map:

```bash
python -m astar.run my_map.txt
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
