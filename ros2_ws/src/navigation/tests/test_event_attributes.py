import tempfile
import unittest
from pathlib import Path

from common.cell_attributes import (
    BLOCKAGE_PROBABILITY,
    HUMAN_PROBABILITY,
    CellAttributeMap,
)
from common.event_attributes import apply_new_events
from common.event_store import PATH_BLOCKED, EventStore, point
from common.human_activity_report import build_human_activity_report


def empty_attributes(width=4, height=4):
    values = [[[0.5, 0.0, 0.0, 0.0] for _ in range(width)] for _ in range(height)]
    return CellAttributeMap(width, height, values)


class EventAttributeTest(unittest.TestCase):
    def test_new_events_update_attributes_once_and_set_human_flag(self):
        with tempfile.TemporaryDirectory() as directory:
            store = EventStore(Path(directory) / "events.db")
            attributes = empty_attributes()
            store.record_human_detection("r1", "run1", "map1", (2, 1), distance=2.0)
            store.record_event(
                PATH_BLOCKED, "r1", "run1", "map1", point(1, 1)
            )

            first = apply_new_events(store, attributes, "map1")
            human_value = attributes.values[1][2][HUMAN_PROBABILITY]
            blockage_value = attributes.values[1][1][BLOCKAGE_PROBABILITY]
            second = apply_new_events(store, attributes, "map1")

            self.assertTrue(first["human_detected"])
            self.assertEqual(first["processed_events"], 2)
            self.assertGreater(human_value, 0.0)
            self.assertGreater(blockage_value, 0.0)
            self.assertEqual(second["processed_events"], 0)
            self.assertEqual(attributes.values[1][2][HUMAN_PROBABILITY], human_value)

    def test_report_ranks_cells_that_caused_waiting_and_rerouting(self):
        with tempfile.TemporaryDirectory() as directory:
            store = EventStore(Path(directory) / "events.db")
            waiting = store.record_human_detection(
                "r1", "run1", "map1", (3, 2), distance=1.0
            )
            store.annotate_event(waiting, decision="WAIT")
            rerouting = store.record_human_detection(
                "r1", "run1", "map1", (3, 2), distance=2.0
            )
            store.annotate_event(rerouting, decision="REROUTE")

            report = build_human_activity_report(store.read_events(), "map1")

            self.assertEqual(report[0]["position"], [3, 2])
            self.assertEqual(report[0]["waits"], 1)
            self.assertEqual(report[0]["reroutes"], 1)
            self.assertEqual(report[0]["average_distance"], 1.5)


if __name__ == "__main__":
    unittest.main()
