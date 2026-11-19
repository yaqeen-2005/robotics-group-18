# Unknown-map exploration

This module chooses where the robot should explore when parts of the map are unknown or have confidence below a configured threshold. It reuses `common` for map data and `astar` for paths to selected frontier targets.

`FrontierExplorer.next_move(goal)` first attempts the requested goal. If A* cannot reach it while uncertain map cells remain, it moves toward the nearest reachable frontier. If the map is already confirmed and the goal is unreachable, it returns `None`.
