"""Persist robot navigation events in a small SQLite database.

EventStore records what happened, where it happened, and which run and map it
belongs to. It deliberately stores observations only; later components can use
the event history to calculate visit counts, congestion, or route reliability.
"""

import json
import sqlite3
from contextlib import closing
from datetime import datetime, timezone
from pathlib import Path
from uuid import uuid4

CELL_VISITED = "CELL_VISITED"  # Build visit counts from completed path segments.
HUMAN_DETECTED = "HUMAN_DETECTED"  # Estimate human presence by map area.
PATH_BLOCKED = "PATH_BLOCKED"  # Estimate congestion and unreliable cells.
REROUTE = "REROUTE"  # Measure route reliability after environmental changes.
BACKTRACK = "BACKTRACK"  # Identify traffic conflicts and bottlenecks.
ROBOT_ENCOUNTERED = "ROBOT_ENCOUNTERED"  # Estimate multi-robot traffic risk.
EVENT_TYPES = {
    CELL_VISITED,
    HUMAN_DETECTED,
    PATH_BLOCKED,
    REROUTE,
    BACKTRACK,
    ROBOT_ENCOUNTERED,
}

DEFAULT_EVENT_DATABASE = Path(__file__).parent.parent / "tests" / "events.db"


def point(x: float, y: float) -> dict:
    return {"kind": "point", "x": float(x), "y": float(y)}


def line_segment(start: tuple[float, float], end: tuple[float, float]) -> dict:
    """Represent one straight visited segment, normally recorded at a turn."""
    return {
        "kind": "line_segment",
        "start": [float(start[0]), float(start[1])],
        "end": [float(end[0]), float(end[1])],
    }


class EventStore:
    def __init__(self, database: str | Path = DEFAULT_EVENT_DATABASE):
        self.database = Path(database)
        self.database.parent.mkdir(parents=True, exist_ok=True)
        with closing(sqlite3.connect(self.database)) as connection:
            with connection:
                connection.execute(
                    """
                    CREATE TABLE IF NOT EXISTS events (
                        event_id TEXT PRIMARY KEY,
                        timestamp TEXT NOT NULL,
                        robot_id TEXT NOT NULL,
                        task_id TEXT,
                        type TEXT NOT NULL,
                        position TEXT NOT NULL,
                        details TEXT,
                        run_id TEXT NOT NULL,
                        map_id TEXT NOT NULL
                    )
                    """
                )
                connection.execute(
                    """
                    CREATE TABLE IF NOT EXISTS event_consumers (
                        consumer TEXT NOT NULL,
                        event_id TEXT NOT NULL,
                        processed_at TEXT NOT NULL,
                        PRIMARY KEY (consumer, event_id),
                        FOREIGN KEY (event_id) REFERENCES events(event_id)
                    )
                    """
                )

    def record_event(
        self,
        event_type: str,
        robot_id: str,
        run_id: str,
        map_id: str,
        position: dict,
        *,
        task_id: str | None = None,
        details: dict | None = None,
        timestamp: str | None = None,
        event_id: str | None = None,
    ) -> str:
        if event_type not in EVENT_TYPES:
            raise ValueError(f"unsupported event type: {event_type}")
        if not robot_id or not run_id or not map_id:
            raise ValueError("robot_id, run_id, and map_id are required")
        if position.get("kind") not in {"point", "line_segment"}:
            raise ValueError("position must be a point or line_segment")

        event_id = event_id or str(uuid4())
        timestamp = timestamp or datetime.now(timezone.utc).isoformat()
        with closing(sqlite3.connect(self.database)) as connection:
            with connection:
                connection.execute(
                    """
                    INSERT INTO events (
                        event_id, timestamp, robot_id, task_id, type,
                        position, details, run_id, map_id
                    ) VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?)
                    """,
                    (
                        event_id,
                        timestamp,
                        robot_id,
                        task_id,
                        event_type,
                        json.dumps(position, separators=(",", ":")),
                        json.dumps(details, separators=(",", ":")) if details else None,
                        run_id,
                        map_id,
                    ),
                )
        return event_id

    def record_visited_segment(
        self,
        robot_id: str,
        run_id: str,
        map_id: str,
        start: tuple[float, float],
        end: tuple[float, float],
        *,
        task_id: str | None = None,
    ) -> str:
        """Record the straight path completed before a turn or stop."""
        return self.record_event(
            CELL_VISITED,
            robot_id,
            run_id,
            map_id,
            line_segment(start, end),
            task_id=task_id,
        )

    def record_human_detection(
        self,
        robot_id,
        run_id,
        map_id,
        position_xy,
        *,
        distance=None,
        task_id=None,
    ):
        """Sensor-facing helper for recording a person in map coordinates."""
        details = {"distance": float(distance)} if distance is not None else None
        return self.record_event(
            HUMAN_DETECTED,
            robot_id,
            run_id,
            map_id,
            point(*position_xy),
            task_id=task_id,
            details=details,
        )

    def annotate_event(self, event_id, **details):
        """Add a navigation decision such as WAIT or REROUTE to an event."""
        with closing(sqlite3.connect(self.database)) as connection:
            row = connection.execute(
                "SELECT details FROM events WHERE event_id = ?", (event_id,)
            ).fetchone()
            if row is None:
                raise KeyError(f"unknown event_id: {event_id}")
            merged = json.loads(row[0]) if row[0] else {}
            merged.update(details)
            with connection:
                connection.execute(
                    "UPDATE events SET details = ? WHERE event_id = ?",
                    (json.dumps(merged, separators=(",", ":")), event_id),
                )

    def read_events(self) -> list[dict]:
        with closing(sqlite3.connect(self.database)) as connection:
            connection.row_factory = sqlite3.Row
            rows = connection.execute("SELECT * FROM events ORDER BY timestamp").fetchall()
        events = [dict(row) for row in rows]
        for event in events:
            event["position"] = json.loads(event["position"])
            event["details"] = json.loads(event["details"]) if event["details"] else None
        return events

    def read_unprocessed_events(self, consumer, map_id=None):
        """Read events not yet consumed by one named downstream component."""
        query = """
            SELECT events.* FROM events
            LEFT JOIN event_consumers
              ON event_consumers.event_id = events.event_id
             AND event_consumers.consumer = ?
            WHERE event_consumers.event_id IS NULL
        """
        parameters = [consumer]
        if map_id is not None:
            query += " AND events.map_id = ?"
            parameters.append(map_id)
        query += " ORDER BY events.timestamp"
        with closing(sqlite3.connect(self.database)) as connection:
            connection.row_factory = sqlite3.Row
            rows = connection.execute(query, parameters).fetchall()
        events = [dict(row) for row in rows]
        for event in events:
            event["position"] = json.loads(event["position"])
            event["details"] = json.loads(event["details"]) if event["details"] else None
        return events

    def mark_events_processed(self, consumer, event_ids):
        if not event_ids:
            return
        processed_at = datetime.now(timezone.utc).isoformat()
        with closing(sqlite3.connect(self.database)) as connection:
            with connection:
                connection.executemany(
                    """
                    INSERT OR IGNORE INTO event_consumers
                        (consumer, event_id, processed_at)
                    VALUES (?, ?, ?)
                    """,
                    [(consumer, event_id, processed_at) for event_id in event_ids],
                )
