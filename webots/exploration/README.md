# Exploration frontier detection

Initial standalone component for Xu's autonomous exploration work. It needs
Python 3 only; Webots, ROS and a running SLAM system are not required.

## Interface

`find_frontiers(grid)` returns a list of `(x, y)` cell coordinates, ordered by
row then column. Input is a rectangular sequence of rows, indexed `grid[y][x]`:

| Value | Meaning |
| --- | --- |
| -1 | Unknown |
| 0 | Free |
| 100 | Occupied |

These values match the existing navigation text-map loader. Intermediate
occupancy probabilities are not accepted; future SLAM integration must define
explicit classification thresholds before calling this function.

A frontier is a free cell with at least one unknown neighbour above, below,
left or right. Diagonals and positions outside the map do not count. Each cell
appears at most once. The input is not modified. Empty grids return `[]`;
ragged grids or unsupported cell values raise `ValueError`.

Example (when importing from this directory):

```python
from frontier import find_frontiers

grid = [
    [100, 100, 100, 100],
    [100,   0,   0,  -1],
    [100, 100, 100, 100],
]
assert find_frontiers(grid) == [(2, 1)]
```

Coordinates are grid indices, not metres or Webots world coordinates. Runtime
is O(height * width), with output storage proportional to frontier count.

## Run tests

From the repository root:

```sh
python -m unittest discover -s webots/exploration -p "test_*.py" -v
```

Tests cover uniform maps, obstacles, cardinal versus diagonal adjacency, map
edges, coordinate ordering, duplicate prevention, input preservation and invalid
inputs. They use synthetic maps, not SLAM results or simulation measurements.

## Current limits and next steps

This detects frontier cells only. It does not build a map, measure coverage,
check reachability or robot clearance, select goals, or command the robot.
A frontier cell is not automatically a safe navigation target. No detected
frontiers does not by itself prove exploration is complete (an entirely unknown
map also has none).

Next: group adjacent frontier cells, select safe reachable targets with the
planner, then connect map/pose inputs and navigation feedback in Webots. Coverage
evaluation will require a fixed evaluation region and agreed coordinate alignment.

## Contribution note

This initial module, tests and documentation were prepared with Codex assistance.
Record this assistance in the team's contribution/report declaration as required
by the project README.
