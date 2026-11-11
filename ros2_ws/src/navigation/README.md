# Navigation prototype

Run the default map with a plot:

```bash
./run.sh
```

Print the path without a plot:

```bash
./run.sh --no-visual
```

Pass a different map and matching confidence file as the first two arguments:

```bash
./run.sh my_map.txt my_confidence.txt
```

The map header is `MAP v1 <width> <height>`. All map cells are flattened onto the next line in row-major order. Map symbols are `S` (start), `G` (goal), `.` (free), `#` (obstacle), and `?` (unknown/blocked).

The confidence header is `CONFIDENCE v1 <width> <height>`. All values are flattened onto the next line in the same row-major order as the map. Each value must be between `0.0` (no confidence) and `1.0` (fully confident). The loader currently validates this data but the planner does not use it.

The planner interface is `plan_path(grid, start, goal)`. Coordinates use `(x, y)` and map cells use `grid[y][x]`.
