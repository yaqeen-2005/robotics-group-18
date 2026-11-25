# Navigation prototype

Run the default map with a plot from this directory:

```bash
python -m astar.run
```

The source is organised into:

- `astar`: path planning on an existing map
- `exploration`: future exploration of an unknown map
- `common`: shared map, cell-attribute, and event data for navigation and ML

Print the path without a plot:

```bash
python -m astar.run --no-visual
```

Pass a different map to the planner:

```bash
python -m astar.run my_map.txt
```

The map header is `MAP v1 <width> <height>`. All map cells are flattened onto the next line in row-major order. Map symbols are `S` (start), `G` (goal), `.` (free), `#` (obstacle), and `?` (unknown/blocked).

The separate attribute file uses `ATTRIBUTES v2 <width> <height> 4`. Each cell stores confidence, historical blockage probability, human encounter probability, and other-robot encounter probability. All values are between `0.0` and `1.0`.

The A* cost combines distance and all four risk attributes. Default weights are defined at the top of `astar/a_star.py`; callers may pass a replacement weight dictionary produced by a future training process.

Display the map and all three attribute layers:

```bash
python tests/show_map.py tests/maps/demo_map.txt tests/maps/demo_attributes.txt
```

The planner interface is `astar.a_star.plan_path(grid, start, goal)`. Coordinates use `(x, y)` and map cells use `grid[y][x]`.

## Event history

`EventStore` records `CELL_VISITED`, `HUMAN_DETECTED`, `PATH_BLOCKED`, `REROUTE`, `BACKTRACK`, and `ROBOT_ENCOUNTERED` events in SQLite. Every event includes a `robot_id`, `run_id`, `map_id`, timestamp, and position. Optional fields are `task_id` and JSON `details`.

For the MVP, a visited route is stored as consecutive straight line segments. Record the completed segment whenever the robot turns or stops:

```python
from common.event_store import EventStore

events = EventStore()
events.record_visited_segment("robot-1", "run-1", "map-1", (1, 2), (5, 2))
```

The default development database is `tests/events.db`.
