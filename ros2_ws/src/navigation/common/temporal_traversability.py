"""Update door-like obstacle statistics from fresh, valid sensor observations."""

from common.cell_attributes import (
    BLOCK_COUNT, BLOCK_RATE, DOOR_BELIEF, LAST_STATE, OBSERVATION_COUNT,
    SWITCH_COUNT, TEMPORAL_DEFAULTS,
)


class TemporalTraversabilityLayer:
    def __init__(self, attributes, sample_interval=0.5):
        if sample_interval <= 0:
            raise ValueError("sample interval must be positive")
        self.attributes = attributes
        self.sample_interval = sample_interval
        self.last_sample_time = None

    def update(self, timestamp, observations):
        """Observations maps (x,y) to True=blocked or False=free.

        The caller must supply only current valid sensor observations, never a
        cached SLAM occupancy map. Returns the number of updated cells.
        """
        if self.last_sample_time is not None and timestamp < self.last_sample_time:
            raise ValueError("timestamps must be ordered")
        if self.last_sample_time is not None and timestamp - self.last_sample_time < self.sample_interval:
            return 0
        for (x, y), blocked in observations.items():
            if not (0 <= x < self.attributes.width and 0 <= y < self.attributes.height):
                raise IndexError("observed cell is outside the feature map")
            if type(blocked) is not bool:
                raise ValueError("observation must be True (blocked) or False (free)")
        for (x, y), blocked in observations.items():
            cell = self.attributes.values[y][x]
            if len(cell) == 4:
                cell.extend(TEMPORAL_DEFAULTS)
            state = int(blocked)
            if cell[LAST_STATE] != -1 and cell[LAST_STATE] != state:
                cell[SWITCH_COUNT] += 1
            cell[OBSERVATION_COUNT] += 1
            cell[BLOCK_COUNT] += state
            cell[BLOCK_RATE] = cell[BLOCK_COUNT] / cell[OBSERVATION_COUNT]
            cell[LAST_STATE] = state
        if observations:
            self.last_sample_time = timestamp
        return len(observations)

    def predict(self, predictor, cells=None):
        """Write trained-model probabilities for selected cells only."""
        for x, y in cells if cells is not None else ((x, y) for y in range(self.attributes.height) for x in range(self.attributes.width)):
            self.attributes.update_cell(x, y, DOOR_BELIEF, predictor.predict(self.attributes, x, y))
