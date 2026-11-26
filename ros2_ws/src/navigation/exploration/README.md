# Unknown-map exploration

This module chooses where the robot should explore when parts of the map are unknown or have confidence below a configured threshold. It reuses `common` for map data and `astar` for paths to selected frontier targets.

Frontier detection is adapted from Xu's `origin/codex/exploration-frontiers` implementation (commit `398f480`). Its original four-neighbour rule is retained, with support added for the navigation attribute map and confidence threshold.

`FrontierExplorer.next_move(goal)` first attempts the requested goal. If A* cannot reach it while uncertain map cells remain, it moves toward the nearest reachable frontier. If the map is already confirmed and the goal is unreachable, it returns `None`.
