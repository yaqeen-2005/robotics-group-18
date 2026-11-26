import tempfile
import unittest
from pathlib import Path

from common.event_store import CELL_VISITED, HUMAN_DETECTED, EventStore


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

    def test_sensor_records_and_navigation_annotates_a_human_event(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            store = EventStore(Path(directory) / "events.db")
            event_id = store.record_human_detection(
                "robot-1", "run-1", "map-1", (2, 3), distance=1.5
            )
            store.annotate_event(event_id, decision="WAIT", reroute_score=0.3)

            event = store.read_events()[0]
            self.assertEqual(event["type"], HUMAN_DETECTED)
            self.assertEqual(event["details"]["distance"], 1.5)
            self.assertEqual(event["details"]["decision"], "WAIT")

    def test_each_consumer_reads_an_event_only_once(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            store = EventStore(Path(directory) / "events.db")
            event_id = store.record_human_detection(
                "robot-1", "run-1", "map-1", (2, 3)
            )
            first = store.read_unprocessed_events("attributes", "map-1")
            store.mark_events_processed("attributes", [event_id])
            second = store.read_unprocessed_events("attributes", "map-1")

            self.assertEqual(len(first), 1)
            self.assertEqual(second, [])


if __name__ == "__main__":
    unittest.main()
