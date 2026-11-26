"""Generate reproducible factory maps and synthetic traffic trajectories."""

import argparse
import json
import random
from collections import deque
from pathlib import Path


TEMPLATES = {
    "small_open_quiet": (12, 8, "open", "low", "low", "high"),
    "large_open_busy": (24, 16, "open", "high", "high", "high"),
    "small_aisles_quiet": (16, 10, "aisles", "low", "low", "high"),
    "large_aisles_human_busy": (24, 16, "aisles", "high", "low", "high"),
    "large_aisles_robot_busy": (24, 16, "aisles", "low", "high", "high"),
    "large_aisles_mixed_busy": (24, 16, "aisles", "high", "high", "high"),
    "small_bottleneck_busy": (16, 10, "bottleneck", "high", "high", "high"),
    "large_bottleneck_uncertain": (24, 16, "bottleneck", "high", "high", "low"),
    "training_factory_busy": (40, 30, "aisles", "very_high", "high", "high"),
}

TRAJECTORY_COUNTS = {"low": 2, "high": 8, "very_high": 20}


def make_layout(width: int, height: int, layout: str, rng: random.Random):
    grid = [[0 for _ in range(width)] for _ in range(height)]
    if layout == "open":
        area_count = rng.randint(4, 8)
        for _ in range(area_count):
            area_width = rng.randint(2, max(2, width // 5))
            area_height = rng.randint(2, max(2, height // 4))
            left = rng.randint(2, width - area_width - 2)
            top = rng.randint(2, height - area_height - 2)
            for y in range(top, top + area_height):
                for x in range(left, left + area_width):
                    grid[y][x] = 100
    elif layout == "aisles":
        spacing = rng.randint(3, 5)
        if rng.choice(("vertical", "horizontal")) == "vertical":
            crossing_rows = {1, rng.randint(3, height - 4), height - 2}
            for x in range(rng.randint(2, 4), width - 2, spacing):
                for y in range(1, height - 1):
                    if y not in crossing_rows:
                        grid[y][x] = 100
        else:
            crossing_columns = {1, rng.randint(3, width - 4), width - 2}
            for y in range(rng.randint(2, 4), height - 2, spacing):
                for x in range(1, width - 1):
                    if x not in crossing_columns:
                        grid[y][x] = 100
    elif layout == "bottleneck":
        if rng.choice(("vertical", "horizontal")) == "vertical":
            wall_x = rng.randint(width // 3, width * 2 // 3)
            opening_y = rng.randint(2, height - 3)
            for y in range(height):
                if y != opening_y:
                    grid[y][wall_x] = 100
        else:
            wall_y = rng.randint(height // 3, height * 2 // 3)
            opening_x = rng.randint(2, width - 3)
            for x in range(width):
                if x != opening_x:
                    grid[wall_y][x] = 100
    else:
        raise ValueError(f"unknown layout: {layout}")

    grid[0][0] = 0
    grid[height - 1][width - 1] = 0
    return grid


def shortest_path(grid, start, goal):
    width = len(grid[0])
    height = len(grid)
    queue = deque([start])
    previous = {start: None}
    while queue:
        current = queue.popleft()
        if current == goal:
            path = []
            while current is not None:
                path.append(current)
                current = previous[current]
            return path[::-1]
        x, y = current
        for neighbour in ((x + 1, y), (x - 1, y), (x, y + 1), (x, y - 1)):
            nx, ny = neighbour
            if (
                0 <= nx < width
                and 0 <= ny < height
                and grid[ny][nx] == 0
                and neighbour not in previous
            ):
                previous[neighbour] = current
                queue.append(neighbour)
    return []


def make_trajectories(grid, count: int, rng: random.Random):
    free_cells = [
        (x, y)
        for y, row in enumerate(grid)
        for x, value in enumerate(row)
        if value == 0
    ]
    base_trajectories = []
    route_count = 1 if count <= 2 else 3
    attempts = 0
    while len(base_trajectories) < route_count and attempts < route_count * 20:
        attempts += 1
        start, goal = rng.sample(free_cells, 2)
        path = shortest_path(grid, start, goal)
        if len(path) >= 3:
            base_trajectories.append(path)
    if not base_trajectories:
        return []
    return [base_trajectories[index % len(base_trajectories)] for index in range(count)]


def probability_layer(width, height, trajectories):
    counts = [[0 for _ in range(width)] for _ in range(height)]
    for path in trajectories:
        for x, y in path:
            counts[y][x] += 1
    highest = max(max(row) for row in counts) or 1
    return [[value / highest for value in row] for row in counts]


def blockage_layer(grid):
    width = len(grid[0])
    height = len(grid)
    values = []
    for y, row in enumerate(grid):
        output_row = []
        for x, cell in enumerate(row):
            if cell >= 50:
                output_row.append(1.0)
                continue
            blocked_neighbours = 0
            for nx, ny in ((x + 1, y), (x - 1, y), (x, y + 1), (x, y - 1)):
                if 0 <= nx < width and 0 <= ny < height and grid[ny][nx] >= 50:
                    blocked_neighbours += 1
            output_row.append(min(0.1 + blocked_neighbours * 0.15, 0.85))
        values.append(output_row)
    return values


def generate_template(name: str, output_root: str | Path, seed: int = 1):
    if name not in TEMPLATES:
        raise ValueError(f"unknown template: {name}")
    width, height, layout, human_level, robot_level, map_quality = TEMPLATES[name]
    rng = random.Random(f"{seed}:{name}")
    grid = make_layout(width, height, layout, rng)
    human_paths = make_trajectories(grid, TRAJECTORY_COUNTS[human_level], rng)
    robot_paths = make_trajectories(grid, TRAJECTORY_COUNTS[robot_level], rng)
    human_probability = probability_layer(width, height, human_paths)
    robot_probability = probability_layer(width, height, robot_paths)
    blockage_probability = blockage_layer(grid)

    confidence = []
    for _ in range(height):
        if map_quality == "high":
            confidence.append([rng.uniform(0.85, 1.0) for _ in range(width)])
        else:
            confidence.append([rng.uniform(0.35, 0.85) for _ in range(width)])

    output = Path(output_root) / name
    output.mkdir(parents=True, exist_ok=True)
    symbols = []
    attributes = []
    for y, row in enumerate(grid):
        for x, cell in enumerate(row):
            symbol = "#" if cell >= 50 else "."
            if (x, y) == (0, 0):
                symbol = "S"
            elif (x, y) == (width - 1, height - 1):
                symbol = "G"
            symbols.append(symbol)
            attributes.extend(
                (
                    confidence[y][x],
                    blockage_probability[y][x],
                    human_probability[y][x],
                    robot_probability[y][x],
                )
            )

    (output / "map.txt").write_text(
        f"MAP v1 {width} {height}\n{''.join(symbols)}\n"
    )
    attribute_text = " ".join(f"{value:.3f}" for value in attributes)
    (output / "attributes.txt").write_text(
        f"ATTRIBUTES v2 {width} {height} 4\n{attribute_text}\n"
    )
    metadata = {
        "template": name,
        "seed": seed,
        "layout": layout,
        "size": [width, height],
        "human_traffic": human_level,
        "robot_traffic": robot_level,
        "map_quality": map_quality,
        "human_trajectories": human_paths,
        "robot_trajectories": robot_paths,
    }
    (output / "metadata.json").write_text(json.dumps(metadata, indent=2))
    return output


def main():
    parser = argparse.ArgumentParser(description="Generate synthetic factory maps")
    parser.add_argument("--template", choices=list(TEMPLATES) + ["all"], default="all")
    parser.add_argument("--seed", type=int, default=1)
    parser.add_argument("--output", default=Path(__file__).parent / "maps")
    args = parser.parse_args()

    names = TEMPLATES if args.template == "all" else [args.template]
    for name in names:
        print(generate_template(name, args.output, args.seed))


if __name__ == "__main__":
    main()
