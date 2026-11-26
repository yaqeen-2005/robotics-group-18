"""Apply new navigation events to the persistent per-cell attribute map."""

import argparse
import json

from common.cell_attributes import load_cell_attributes, save_cell_attributes
from common.event_store import EventStore
from common.map_io import load_text_map

from common.cell_attributes import (
    BLOCKAGE_PROBABILITY,
    CONFIDENCE,
    HUMAN_PROBABILITY,
    ROBOT_ENCOUNTER_PROBABILITY,
)
from common.event_store import (
    BACKTRACK,
    CELL_VISITED,
    HUMAN_DETECTED,
    PATH_BLOCKED,
    REROUTE,
    ROBOT_ENCOUNTERED,
)

DEFAULT_EVENT_UPDATE_RATES = {
    "confidence_gain": 0.10,
    "successful_visit_blockage_decay": 0.05,
    "blockage_gain": 0.20,
    "human_gain": 0.25,
    "robot_gain": 0.20,
}


def apply_new_events(
    store,
    attributes,
    map_id,
    consumer="attribute_map_v1",
    rates=None,
):
    """Update attributes once per event and return current-batch flags."""
    active_rates = DEFAULT_EVENT_UPDATE_RATES.copy()
    if rates is not None:
        unknown = set(rates) - set(active_rates)
        if unknown:
            raise ValueError(f"unknown event update rates: {sorted(unknown)}")
        active_rates.update(rates)
    if any(not 0.0 <= value <= 1.0 for value in active_rates.values()):
        raise ValueError("event update rates must be between 0.0 and 1.0")

    events = store.read_unprocessed_events(consumer, map_id)
    updated_cells = set()
    human_detected = False
    for event in events:
        event_type = event["type"]
        points = _event_points(event["position"])
        if event_type == HUMAN_DETECTED:
            human_detected = True
        for x, y in points:
            if not (0 <= x < attributes.width and 0 <= y < attributes.height):
                continue
            cell = attributes.values[y][x]
            if event_type == CELL_VISITED:
                cell[CONFIDENCE] = _increase(
                    cell[CONFIDENCE], active_rates["confidence_gain"]
                )
                cell[BLOCKAGE_PROBABILITY] *= (
                    1.0 - active_rates["successful_visit_blockage_decay"]
                )
            elif event_type in {PATH_BLOCKED, BACKTRACK}:
                cell[BLOCKAGE_PROBABILITY] = _increase(
                    cell[BLOCKAGE_PROBABILITY], active_rates["blockage_gain"]
                )
            elif event_type == HUMAN_DETECTED:
                cell[HUMAN_PROBABILITY] = _increase(
                    cell[HUMAN_PROBABILITY], active_rates["human_gain"]
                )
            elif event_type == ROBOT_ENCOUNTERED:
                cell[ROBOT_ENCOUNTER_PROBABILITY] = _increase(
                    cell[ROBOT_ENCOUNTER_PROBABILITY], active_rates["robot_gain"]
                )
            elif event_type == REROUTE:
                details = event["details"] or {}
                dimension = (
                    HUMAN_PROBABILITY
                    if details.get("cause") == "human"
                    else BLOCKAGE_PROBABILITY
                )
                rate = (
                    active_rates["human_gain"]
                    if dimension == HUMAN_PROBABILITY
                    else active_rates["blockage_gain"]
                )
                cell[dimension] = _increase(cell[dimension], rate)
            updated_cells.add((x, y))

    store.mark_events_processed(consumer, [event["event_id"] for event in events])
    return {
        "processed_events": len(events),
        "updated_cells": sorted(updated_cells),
        "human_detected": human_detected,
    }


def _increase(value, rate):
    return min(1.0, value + rate * (1.0 - value))


def _event_points(position):
    if position["kind"] == "point":
        return [(round(position["x"]), round(position["y"]))]
    start_x, start_y = (round(value) for value in position["start"])
    end_x, end_y = (round(value) for value in position["end"])
    if start_x != end_x and start_y != end_y:
        return []
    step_x = (end_x > start_x) - (end_x < start_x)
    step_y = (end_y > start_y) - (end_y < start_y)
    length = max(abs(end_x - start_x), abs(end_y - start_y))
    return [(start_x + step_x * index, start_y + step_y * index) for index in range(length + 1)]


def main():
    parser = argparse.ArgumentParser(description="Apply new events to an attribute map")
    parser.add_argument("database", help="EventStore SQLite database")
    parser.add_argument("map", help="matching MAP v1 file")
    parser.add_argument("attributes", help="input ATTRIBUTES v2 file")
    parser.add_argument("map_id", help="event map identifier")
    parser.add_argument("--output", required=True, help="updated ATTRIBUTES v2 file")
    parser.add_argument("--consumer", default="attribute_map_v1")
    args = parser.parse_args()

    grid, _, _ = load_text_map(args.map)
    attributes = load_cell_attributes(args.attributes, len(grid[0]), len(grid))
    result = apply_new_events(
        EventStore(args.database), attributes, args.map_id, args.consumer
    )
    save_cell_attributes(args.output, attributes)
    print(json.dumps(result))


if __name__ == "__main__":
    main()
