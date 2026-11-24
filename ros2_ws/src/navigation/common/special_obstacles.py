"""Small registry of wait/reroute rules for detected people and door-like cells."""


class SpecialObstacles:
    def __init__(self, human_distance=float("inf"), door_distance=3, door_belief=0.6,
                 door_open_rate=0.2, door_wait_limit=6, backtrack_wait_weight=0.1):
        self.rules = {"human": self.human, "door": self.door}
        self.human_distance = human_distance
        self.door_distance = door_distance
        self.door_belief = door_belief
        self.door_open_rate = door_open_rate
        self.door_wait_limit = door_wait_limit
        self.backtrack_wait_weight = backtrack_wait_weight

    def decide(self, kind, *, distance, waiting_time=0, belief=1, block_rate=1,
               human_reroute_score=0, human_threshold=0.6, backtracks=0):
        if kind not in self.rules:
            raise ValueError(f"unknown special obstacle: {kind}")
        return self.rules[kind](distance, waiting_time, belief, block_rate,
                                human_reroute_score, human_threshold, backtracks)

    def human(self, distance, waiting_time, belief, block_rate, score, threshold, backtracks):
        if distance > self.human_distance:
            return "IGNORE"
        return "WAIT" if score - self.backtrack_wait_weight * min(backtracks, 3) < threshold else "REROUTE"

    def door(self, distance, waiting_time, belief, block_rate, score, threshold, backtracks):
        if distance > self.door_distance or belief < self.door_belief:
            return "IGNORE"
        open_threshold = max(0.1, self.door_open_rate - self.backtrack_wait_weight * min(backtracks, 3))
        wait_limit = self.door_wait_limit + min(backtracks, 3)
        return "WAIT" if 1 - block_rate >= open_threshold and waiting_time < wait_limit else "REROUTE"
