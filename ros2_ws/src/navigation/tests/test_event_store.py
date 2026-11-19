import tempfile
import unittest
from pathlib import Path

from common.event_store import CELL_VISITED, EventStore


class EventStoreTest(unittest.TestCase):
    def test_records_a_visited_line_segment(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            store = EventStore(Path(directory) / "events.db")
            event_id = store.record_visited_segment(
                "robot-1", "run-1", "map-1", (1, 2), (5, 2), task_id="task-1"
            )

            event = store.read_events()[0]
            self.assertEqual(event["event_id"], event_id)
            self.assertEqual(event["type"], CELL_VISITED)
            self.assertEqual(event["position"]["kind"], "line_segment")
            self.assertEqual(event["position"]["end"], [5.0, 2.0])


if __name__ == "__main__":
    unittest.main()
