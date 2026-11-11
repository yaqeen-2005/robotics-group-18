# Navigation prototype

Run the default map with a plot:

```bash
./run.sh
```

Print the path without a plot:

```bash
./run.sh --no-visual
```

Pass a different text map as the first argument. Map symbols are `S` (start), `G` (goal), `.` (free), `#` (obstacle), and `?` (unknown/blocked).

The planner interface is `plan_path(grid, start, goal)`. Coordinates use `(x, y)` and map cells use `grid[y][x]`.
