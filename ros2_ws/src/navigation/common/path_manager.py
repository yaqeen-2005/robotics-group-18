"""Keep route state and handle human blocking separately from map changes."""

from astar.a_star import plan_path

DEFAULT_HUMAN_DECISION_WEIGHTS = {
    "bias": 0.0,
    "distance": 0.2,
    "waiting_time": 0.8,
    "reroute_count": -0.4,
}
DEFAULT_REROUTE_THRESHOLD = 0.6
DEFAULT_NORMALISATION_LIMITS = {
    "human_distance": 10.0,
    "waiting_time": 10.0,
    "reroute_count": 5,
}


class PathManager:
    def __init__(
        self,
        reroute_threshold=DEFAULT_REROUTE_THRESHOLD,
        decision_weights=None,
        max_human_distance=DEFAULT_NORMALISATION_LIMITS["human_distance"],
        max_waiting_time=DEFAULT_NORMALISATION_LIMITS["waiting_time"],
        max_reroutes=DEFAULT_NORMALISATION_LIMITS["reroute_count"],
    ):
        self.reroute_threshold = reroute_threshold
        self.decision_weights = DEFAULT_HUMAN_DECISION_WEIGHTS.copy()
        if decision_weights is not None:
            unknown = set(decision_weights) - set(self.decision_weights)
            if unknown:
                raise ValueError(f"unknown decision weights: {sorted(unknown)}")
            self.decision_weights.update(decision_weights)
        if max_human_distance <= 0 or max_waiting_time <= 0 or max_reroutes <= 0:
            raise ValueError("normalisation limits must be greater than zero")
        self.max_human_distance = max_human_distance
        self.max_waiting_time = max_waiting_time
        self.max_reroutes = max_reroutes
        self.current_path = []
        self.previous_paths = []
        self.reroute_count = 0

    def human_reroute_score(self, human_distance, waiting_time):
        """Return the configurable linear score used for wait/reroute decisions."""
        if human_distance < 0 or waiting_time < 0:
            raise ValueError("distance and waiting time must not be negative")
        distance = min(human_distance / self.max_human_distance, 1.0)
        waiting = min(waiting_time / self.max_waiting_time, 1.0)
        reroutes = min(self.reroute_count / self.max_reroutes, 1.0)
        weights = self.decision_weights
        return (
            weights["bias"]
            + weights["distance"] * distance
            + weights["waiting_time"] * waiting
            + weights["reroute_count"] * reroutes
        )

    def plan(
        self,
        grid,
        start,
        goal,
        attributes=None,
        weights=None,
        start_direction=None,
        map_changed=False,
        human_blocking=False,
        human_distance=0.0,
        waiting_time=0.0,
        human_cells=None,
    ):
        """Generate an A* path every call, then decide whether to adopt it."""
        generated = plan_path(
            grid,
            start,
            goal,
            attributes,
            weights,
            start_direction,
        )
        remaining = self._remaining_path(start)

        if not self.current_path:
            self.current_path = generated
            if len(generated) > 1:
                action = "MOVE"
            elif generated:
                action = "ARRIVED"
            else:
                action = "NO_PATH"
            return self._result(action, generated)

        if human_blocking:
            score = self.human_reroute_score(human_distance, waiting_time)
            if score < self.reroute_threshold:
                return self._result("WAIT", remaining, score)

            old_path = remaining or self.current_path
            avoided = set(old_path[1:-1])
            occupied_by_people = set(human_cells or [])
            occupied_by_people.discard(start)
            if goal in occupied_by_people:
                return self._result("WAIT", old_path, score)
            alternative = plan_path(
                grid,
                start,
                goal,
                attributes,
                weights,
                start_direction,
                blocked_cells=avoided | occupied_by_people,
            )
            if alternative and alternative != old_path:
                self._replace_path(alternative, old_path)
                return self._result("REROUTE", alternative, score)
            return self._result("WAIT", old_path, score)

        if map_changed:
            old_path = remaining or self.current_path
            if generated != old_path:
                self._replace_path(generated, old_path)
                action = "REROUTE" if generated else "NO_PATH"
                return self._result(action, generated)
            self.current_path = generated
            return self._result("MOVE" if len(generated) > 1 else "NO_PATH", generated)

        if remaining:
            return self._result("MOVE" if len(remaining) > 1 else "ARRIVED", remaining)
        self.current_path = generated
        return self._result("MOVE" if len(generated) > 1 else "NO_PATH", generated)

    def _remaining_path(self, start):
        if start not in self.current_path:
            return []
        return self.current_path[self.current_path.index(start) :]

    def _replace_path(self, new_path, old_path):
        if old_path:
            self.previous_paths.append(list(old_path))
        self.current_path = new_path
        self.reroute_count += 1

    def _result(self, action, path, score=None):
        return {
            "action": action,
            "path": path,
            "reroute_score": score,
            "reroute_count": self.reroute_count,
        }
