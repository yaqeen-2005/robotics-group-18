"""Replay Webots wheel encoder logs as planar differential-drive odometry."""

import argparse
import csv
import json
import math
from pathlib import Path


class DifferentialDriveOdometry:
    """Integrate cumulative left and right wheel angles in radians."""

    def __init__(self, wheel_radius_m, wheel_track_m):
        if any(not math.isfinite(v) or v <= 0 for v in (wheel_radius_m, wheel_track_m)):
            raise ValueError("wheel radius and track must be finite and positive")
        self.radius = wheel_radius_m
        self.track = wheel_track_m
        self.pose = (0.0, 0.0, 0.0)
        self.previous = None

    def update(self, left_rad, right_rad):
        angles = (left_rad, right_rad)
        if any(not math.isfinite(angle) for angle in angles):
            raise ValueError("wheel angles must be finite")
        previous, self.previous = self.previous, angles
        if previous is None:
            return self.pose

        left = self.radius * (left_rad - previous[0])
        right = self.radius * (right_rad - previous[1])
        distance = (left + right) / 2
        turn = (right - left) / self.track
        half_turn = turn / 2
        chord = distance * (math.sin(half_turn) / half_turn if half_turn else 1.0)
        x, y, yaw = self.pose
        self.pose = (
            x + chord * math.cos(yaw + half_turn),
            y + chord * math.sin(yaw + half_turn),
            math.remainder(yaw + turn, math.tau),
        )
        return self.pose


def replay(stream):
    """Yield (time_s, x_m, y_m, yaw_rad) from a g18-slam-sensors-v1 stream."""
    metadata = json.loads(next(stream, "null"))
    if not isinstance(metadata, dict) or metadata.get("schema") != "g18-slam-sensors-v1":
        raise ValueError("expected a g18-slam-sensors-v1 header")
    odometry = DifferentialDriveOdometry(metadata["wheel_radius_m"], metadata["wheel_track_m"])
    previous_time = -math.inf
    for line_number, line in enumerate(stream, start=2):
        try:
            row = json.loads(line)
            timestamp = row["time_s"]
            left, right = row["wheel_angles_rad"]
            if not math.isfinite(timestamp) or timestamp <= previous_time:
                raise ValueError("timestamps must be finite and strictly increasing")
            previous_time = timestamp
            if left is None or right is None:
                continue
            pose = odometry.update(left, right)
        except (KeyError, TypeError, ValueError) as error:
            raise ValueError(f"line {line_number}: {error}") from error
        yield (timestamp, *pose)


def plot_trajectory(rows, path):
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    times, x, y, yaw = zip(*rows)
    fig, (xy, heading) = plt.subplots(1, 2, figsize=(9, 4), layout="constrained")
    xy.plot(x, y, color="tab:blue")
    xy.scatter(x[0], y[0], marker="o", color="tab:green", label="Start", zorder=3)
    xy.scatter(x[-1], y[-1], marker="x", color="tab:red", label="End", zorder=3)
    heading.plot(times, yaw, color="tab:blue")
    xy.set(xlabel="x (m)", ylabel="y (m)", title="Estimated trajectory")
    xy.set_aspect("equal", adjustable="datalim")
    xy.legend()
    heading.set(xlabel="Simulation time (s)", ylabel="Yaw (rad)", title="Estimated heading")
    for axis in (xy, heading):
        axis.grid(alpha=0.25)
    fig.savefig(path, dpi=160)
    plt.close(fig)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("log", type=Path)
    parser.add_argument("--csv", required=True, type=Path, help="output pose CSV")
    parser.add_argument("--plot", type=Path, help="optional plot (requires matplotlib)")
    args = parser.parse_args()
    paths = [p.resolve() for p in (args.log, args.csv, args.plot) if p is not None]
    if len(set(paths)) != len(paths):
        parser.error("log, CSV and plot must use distinct paths")
    with args.log.open(encoding="utf-8") as stream:
        rows = list(replay(stream))
    if not rows:
        parser.error("the log contains no valid encoder pairs")
    args.csv.parent.mkdir(parents=True, exist_ok=True)
    with args.csv.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.writer(stream)
        writer.writerow(("time_s", "x_m", "y_m", "yaw_rad"))
        writer.writerows(rows)
    if args.plot:
        args.plot.parent.mkdir(parents=True, exist_ok=True)
        plot_trajectory(rows, args.plot)
    print(f"Replayed {len(rows)} poses to {args.csv}")


if __name__ == "__main__":
    main()
