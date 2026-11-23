"""Measure warehouse localization error from timestamp-paired poses."""

import argparse
import csv
import json
import math
from pathlib import Path


def evaluate(rows):
    errors = []
    phase_ends = {}
    for row in rows:
        position = math.hypot(float(row["x_m"]) - float(row["truth_x_m"]),
                              float(row["y_m"]) - float(row["truth_y_m"]))
        yaw = abs(math.remainder(float(row["yaw_rad"]) - float(row["truth_yaw_rad"]), math.tau))
        error = {"time_s": float(row["time_s"]), "position_m": position,
                 "heading_deg": math.degrees(yaw)}
        errors.append(error)
        phase_ends[row["phase"]] = error
    if not errors:
        raise ValueError("no paired poses")
    return {
        "samples": len(errors),
        "position_rmse_m": math.sqrt(math.fsum(e["position_m"] ** 2 for e in errors) / len(errors)),
        "heading_rmse_deg": math.sqrt(math.fsum(e["heading_deg"] ** 2 for e in errors) / len(errors)),
        "max_position_error_m": max(e["position_m"] for e in errors),
        "max_heading_error_deg": max(e["heading_deg"] for e in errors),
        "phase_end_errors": phase_ends,
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("paired_csv", type=Path)
    args = parser.parse_args()
    with args.paired_csv.open(newline="", encoding="utf-8") as stream:
        print(json.dumps(evaluate(csv.DictReader(stream)), indent=2, allow_nan=False))


if __name__ == "__main__":
    main()
