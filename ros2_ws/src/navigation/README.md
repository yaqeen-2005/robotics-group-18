# Navigation prototype

Run the default map with a plot:

```bash
./run.sh
```

Print the path without a plot:

```bash
./run.sh --no-visual
```

Pass a different map to the planner:

```bash
./run.sh my_map.txt
```

The map header is `MAP v1 <width> <height>`. All map cells are flattened onto the next line in row-major order. Map symbols are `S` (start), `G` (goal), `.` (free), `#` (obstacle), and `?` (unknown/blocked).

The separate attribute file uses `ATTRIBUTES v1 <width> <height> 3`. Each cell stores three values in this order: confidence, historical blockage probability, and human encounter probability. All values are between `0.0` and `1.0`. This component is not yet connected to the planner.

Display the map and all three attribute layers:

```bash
python tests/show_map.py tests/maps/demo_map.txt tests/maps/demo_attributes.txt
```

The planner interface is `plan_path(grid, start, goal)`. Coordinates use `(x, y)` and map cells use `grid[y][x]`.

## Event history

`EventStore` records `CELL_VISITED`, `HUMAN_DETECTED`, `PATH_BLOCKED`, `REROUTE`, and `BACKTRACK` events in SQLite. Every event includes a `robot_id`, `run_id`, `map_id`, timestamp, and position. Optional fields are `task_id` and JSON `details`.

For the MVP, a visited route is stored as consecutive straight line segments. Record the completed segment whenever the robot turns or stops:

```python
from event_store import EventStore

events = EventStore()
events.record_visited_segment("robot-1", "run-1", "map-1", (1, 2), (5, 2))
```

The default development database is `tests/events.db`.
