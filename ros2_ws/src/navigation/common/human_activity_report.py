"""Summarise where human detections cause waiting or rerouting."""

import argparse
import json
from collections import defaultdict
from pathlib import Path

from common.event_store import EventStore, HUMAN_DETECTED, REROUTE


def build_human_activity_report(events, map_id=None):
    cells = defaultdict(
        lambda: {
            "detections": 0,
            "waits": 0,
            "reroutes": 0,
            "distance_total": 0.0,
            "distance_samples": 0,
        }
    )
    for event in events:
        if map_id is not None and event["map_id"] != map_id:
            continue
        details = event["details"] or {}
        position = event["position"]
        if position["kind"] != "point":
            continue
        key = (round(position["x"]), round(position["y"]))
        if event["type"] == HUMAN_DETECTED:
            cells[key]["detections"] += 1
            decision = details.get("decision") or details.get("action")
            if decision == "WAIT":
                cells[key]["waits"] += 1
            elif decision == "REROUTE":
                cells[key]["reroutes"] += 1
            if details.get("distance") is not None:
                cells[key]["distance_total"] += float(details["distance"])
                cells[key]["distance_samples"] += 1
        elif event["type"] == REROUTE and details.get("cause") == "human":
            cells[key]["reroutes"] += 1

    rows = []
    for (x, y), values in cells.items():
        samples = values.pop("distance_samples")
        total = values.pop("distance_total")
        rows.append(
            {
                "position": [x, y],
                **values,
                "average_distance": round(total / samples, 3) if samples else None,
                "impact": values["waits"] + values["reroutes"],
            }
        )
    rows.sort(key=lambda row: (-row["impact"], -row["detections"], row["position"]))
    return rows


def main():
    parser = argparse.ArgumentParser(description="Report human activity by map cell")
    parser.add_argument("database", help="EventStore SQLite database")
    parser.add_argument("--map-id", help="only include one map")
    parser.add_argument("--output", help="write JSON instead of printing it")
    args = parser.parse_args()
    report = build_human_activity_report(
        EventStore(args.database).read_events(), args.map_id
    )
    text = json.dumps(report, indent=2)
    if args.output:
        Path(args.output).write_text(text + "\n")
    else:
        print(text)


if __name__ == "__main__":
    main()
