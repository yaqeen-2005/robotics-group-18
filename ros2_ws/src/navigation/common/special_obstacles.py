"""Small registry of wait/reroute rules for detected people and door-like cells."""


class SpecialObstacles:
    def __init__(self, human_distance=float("inf"), door_distance=3, door_belief=0.6,
                 door_open_rate=0.2, door_wait_limit=6):
        self.rules = {"human": self.human, "door": self.door}
        self.human_distance = human_distance
        self.door_distance = door_distance
        self.door_belief = door_belief
        self.door_open_rate = door_open_rate
        self.door_wait_limit = door_wait_limit

    def decide(self, kind, *, distance, waiting_time=0, belief=1, block_rate=1,
               human_reroute_score=0, human_threshold=0.6):
        if kind not in self.rules:
            raise ValueError(f"unknown special obstacle: {kind}")
        return self.rules[kind](distance, waiting_time, belief, block_rate,
                                human_reroute_score, human_threshold)

    def human(self, distance, waiting_time, belief, block_rate, score, threshold):
        if distance > self.human_distance:
            return "IGNORE"
        return "WAIT" if score < threshold else "REROUTE"

    def door(self, distance, waiting_time, belief, block_rate, score, threshold):
        if distance > self.door_distance or belief < self.door_belief:
            return "IGNORE"
        return "WAIT" if 1 - block_rate >= self.door_open_rate and waiting_time < self.door_wait_limit else "REROUTE"
