"""Evaluate a final-warehouse mapping trial and render the measured route."""
import argparse
from collections import Counter, deque
import csv
import json
import math
from pathlib import Path

from .mapping import plot_map


def accessible_route(data, start, goal, radius):
    """Four-connected search with a circular clearance and cell-corner margin."""
    grid, r = data["grid"], data["resolution_m"]
    ox, oy = data["origin_m"]
    def cell(p):
        return math.floor((p[0]-ox)/r), math.floor((p[1]-oy)/r)
    extent = math.ceil(radius/r + .71)
    offsets = [(i,j) for i in range(-extent,extent+1) for j in range(-extent,extent+1)
               if math.hypot(i,j)*r <= radius+r/math.sqrt(2)]
    def clear(p):
        return all(0 <= p[1]+j < len(grid) and 0 <= p[0]+i < len(grid[0]) and
                   grid[p[1]+j][p[0]+i] == 0 for i,j in offsets)
    first, last = cell(start), cell(goal)
    if not clear(first) or not clear(last):
        return []
    queue, parent = deque([first]), {first: None}
    visited = {first}
    while queue:
        point = queue.popleft()
        if point == last:
            path = []
            while point is not None:
                x,y = point
                path.append([ox+(x+.5)*r,oy+(y+.5)*r])
                point = parent[point]
            return path[::-1]
        x,y = point
        for neighbour in ((x-1,y),(x+1,y),(x,y-1),(x,y+1)):
            if neighbour in visited:
                continue
            visited.add(neighbour)
            if clear(neighbour):
                parent[neighbour] = point
                queue.append(neighbour)
    return []


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("trial",type=Path)
    parser.add_argument("output",type=Path)
    args = parser.parse_args()
    data = json.loads((args.trial/"warehouse-map.json").read_text())
    track = data["trajectory"]
    with (args.trial/"paired.csv").open() as f:
        truth = {round(float(row["time_s"]),6): row for row in csv.DictReader(f)}
    errors = []
    for t,x,y,yaw in track:
        sample = truth[round(t,6)]
        errors.append(math.hypot(x-float(sample["truth_x_m"]), y-float(sample["truth_y_m"])))
    # Same approach waypoints as the QA driver, in the warehouse frame.
    start, goal = [-36.7,-8], [-36.7,16]
    data["approaches"] = {"P03":start,"D01":goal}
    data["checked_route"] = accessible_route(data,start,goal,.825)
    args.output.mkdir(parents=True,exist_ok=True)
    (args.output/"map.json").write_text(json.dumps(data,separators=(",",":")))
    plot_map(data,track,args.output/"map.png")
    result = dict(scans=len(track),position_rmse_m=math.sqrt(sum(e*e for e in errors)/len(errors)),
                  last_scan_error_m=errors[-1], clearance_radius_m=.825,
                  approach_path_found=bool(data["checked_route"]),
                  cells=dict(Counter(v for row in data["grid"] for v in row)))
    (args.output/"validation.json").write_text(json.dumps(result,indent=2))
    print(result)


if __name__=="__main__":
    main()
