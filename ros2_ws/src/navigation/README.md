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
