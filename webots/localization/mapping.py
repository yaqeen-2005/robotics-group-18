"""Build a 2D occupancy grid from a recorded Webots drive."""

import argparse
from collections import defaultdict
import json
import math
from pathlib import Path

from .odometry import read_header, replay_frames


def ray_cells(start, end):
    """Yield Bresenham grid cells, including both endpoints."""
    x, y = start
    end_x, end_y = end
    dx, dy = abs(end_x - x), -abs(end_y - y)
    sx, sy = (1 if x < end_x else -1), (1 if y < end_y else -1)
    error = dx + dy
    while (x, y) != end:
        yield x, y
        doubled_error = 2 * error
        if doubled_error >= dy:
            error += dy
            x += sx
        if doubled_error <= dx:
            error += dx
            y += sy
    yield end


class OccupancyMapper:
    def __init__(self, lidar, resolution):
        if not math.isfinite(resolution) or resolution <= 0:
            raise ValueError("resolution must be finite and positive")
        self.resolution = resolution
        self.offset_x, self.offset_y, _ = lidar["translation_m"]
        self.sensor_yaw = lidar["yaw_rad"]
        self.min_range = lidar["min_range_m"]
        self.max_range = lidar["max_range_m"]
        count = lidar["horizontal_resolution"]
        fov = lidar["horizontal_fov_rad"]
        if count < 1 or not 0 < fov <= math.tau:
            raise ValueError("invalid LiDAR resolution or field of view")
        self.bearings = [fov * (0.5 - (i + 0.5) / count) for i in range(count)]
        self.evidence = defaultdict(int)

    def cell(self, x, y):
        return math.floor(x / self.resolution), math.floor(y / self.resolution)

    def update(self, pose, ranges):
        if len(ranges) != len(self.bearings):
            raise ValueError("scan length differs from horizontal resolution")
        x, y, yaw = pose
        c, s = math.cos(yaw), math.sin(yaw)
        sx = x + c * self.offset_x - s * self.offset_y
        sy = y + s * self.offset_x + c * self.offset_y
        free, occupied = set(), set()
        for angle, distance in zip(self.bearings, ranges):
            if distance is None or not self.min_range <= distance < self.max_range:
                continue
            heading = yaw + self.sensor_yaw + angle
            c, s = math.cos(heading), math.sin(heading)
            start = self.cell(sx + self.min_range * c, sy + self.min_range * s)
            end = self.cell(sx + distance * c, sy + distance * s)
            free.update(ray_cells(start, end))
            occupied.add(end)
        for cell in free - occupied:
            self.evidence[cell] = max(-8, self.evidence[cell] - 1)
        for cell in occupied:
            self.evidence[cell] = min(8, self.evidence[cell] + 3)

    def snapshot(self, timed_pose, frame="startup_odometry"):
        if not self.evidence:
            raise ValueError("no valid LiDAR returns to map")
        xs, ys = zip(*self.evidence)
        px, py = self.cell(*timed_pose[1:3])
        x0, y0 = min(min(xs), px), min(min(ys), py)
        width, height = max(max(xs), px) - x0 + 1, max(max(ys), py) - y0 + 1
        grid = [[-1] * width for _ in range(height)]
        for (x, y), score in self.evidence.items():
            grid[y - y0][x - x0] = 100 if score > 0 else 0 if score < 0 else -1
        timestamp, x, y, yaw = timed_pose
        return {
            "frame": frame,
            "resolution_m": self.resolution,
            "origin_m": [x0 * self.resolution, y0 * self.resolution],
            "origin_yaw_rad": 0,
            "width": width, "height": height,
            "time_s": timestamp,
            "pose": {"x_m": x, "y_m": y, "yaw_rad": yaw},
            "grid": grid,
        }


def plot_map(snapshot, trajectory, path):
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.colors import BoundaryNorm, ListedColormap
    from matplotlib.patches import Patch

    ox, oy = snapshot["origin_m"]
    resolution = snapshot["resolution_m"]
    extent = (ox, ox + snapshot["width"] * resolution,
              oy, oy + snapshot["height"] * resolution)
    colors = ["#b4bdc8", "#ffffff", "#182332"]
    fig, ax = plt.subplots(figsize=(8, 6), layout="constrained")
    ax.imshow(snapshot["grid"], origin="lower", extent=extent, interpolation="nearest",
              cmap=ListedColormap(colors), norm=BoundaryNorm([-2, -0.5, 0.5, 101], 3))
    _, x, y, _ = zip(*trajectory)
    path_line, = ax.plot(x, y, color="#d74323", linewidth=2, label="Estimated path")
    ax.scatter(x[0], y[0], color="#d74323", s=20)
    for name, point in snapshot.get("stations", {}).items():
        if extent[0] <= point[0] < extent[1] and extent[2] <= point[1] < extent[3]:
            ax.scatter(*point, marker="s", color="#67468f", s=24)
            ax.annotate(name, point, xytext=(4, 4), textcoords="offset points")
    ax.set(title="Webots occupancy grid", xlabel="x (m)", ylabel="y (m)")
    patches = [Patch(facecolor=c, edgecolor="#546477", label=l)
               for c, l in zip(colors, ("Unknown", "Free", "Occupied"))]
    for point in snapshot.get("approaches", {}).values():
        ax.scatter(*point, marker="x", color="#187643", s=35)
    route = snapshot.get("checked_route")
    if route:
        rx, ry = zip(*route)
        route_line, = ax.plot(rx, ry, "--", color="#187643", label="Clearance-checked path")
        patches.append(route_line)
    ax.legend(handles=[*patches, path_line], loc="upper center",
              bbox_to_anchor=(0.5, -0.12), ncols=4)
    fig.savefig(path, dpi=160)
    plt.close(fig)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("log", type=Path)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--resolution", type=float, default=0.10)
    parser.add_argument("--plot", type=Path)
    args = parser.parse_args()
    paths = [p.resolve() for p in (args.log, args.output, args.plot) if p is not None]
    if len(paths) != len(set(paths)):
        parser.error("log, output and plot must have distinct paths")
    trajectory = []
    with args.log.open(encoding="utf-8") as stream:
        metadata = read_header(stream)
        mapper = OccupancyMapper(metadata["lidar"], args.resolution)
        for pose, frame in replay_frames(stream, metadata):
            mapper.update(pose[1:], frame["ranges_m"])
            trajectory.append(pose)
    if not trajectory:
        parser.error("log contains no valid encoder pairs")
    snapshot = mapper.snapshot(trajectory[-1])
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(snapshot, separators=(",", ":")) + "\n", encoding="utf-8")
    if args.plot:
        args.plot.parent.mkdir(parents=True, exist_ok=True)
        plot_map(snapshot, trajectory, args.plot)
    print(f"Mapped {len(trajectory)} frames to {snapshot['width']} x {snapshot['height']} cells.")


if __name__ == "__main__":
    main()
