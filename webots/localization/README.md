# Differential-drive odometry

Replay a Webots sensor log from the repository root:

```powershell
py webots/localization/odometry.py webots/results/slam/drive-validation.jsonl --csv webots/results/slam/odometry.csv
```

To generate a figure, add `--plot webots/results/slam/odometry.png` with
Matplotlib installed. CSV output and the odometry class use only Python's
standard library. See the [sensor recording guide](../README.md) to capture
another run.

## Pose interface

- `DifferentialDriveOdometry.update(left_rad, right_rad)` accepts cumulative
  encoder angles and returns `(x_m, y_m, yaw_rad)`.
- The first valid encoder pair establishes pose `(0, 0, 0)` at the drive axle
  midpoint. The startup frame points forward along +x, left along +y, and
  has positive yaw toward the left. Yaw is wrapped to ±π.
- The CSV has `time_s,x_m,y_m,yaw_rad`. Time comes from the simulation clock.
  Wheel radius and track come from the log header. LiDAR fields are unused
  in this estimator.
- A record with a missing encoder value is skipped. The next valid pair
  integrates the accumulated angle change from the previous valid pair as
  one arc. Accuracy may decrease if the robot changes its curvature in the gap.
- Mapping consumers need an explicit transform from this startup frame to
  their map frame. A map-relative pose requires alignment or correction.

The model converts wheel rotation into wheel travel using the wheel radius.
The mean wheel travel gives centre displacement and the right-minus-left
wheel travel divided by track gives heading change. Each interval is
integrated as a constant-curvature arc. Wheel slip and parameter errors
accumulate over time. A synchronized Webots ground-truth comparison is
needed to quantify positioning error.

## Build an occupancy grid

From the repository root, replay a sensor log with:

```powershell
py -m webots.localization.mapping webots/results/slam/drive-validation.jsonl --output webots/results/slam/occupancy.json --plot webots/results/slam/occupancy.png
```

Omit `--plot` to run without Matplotlib. The default resolution is 0.10 m
per cell. Each scan is paired with the encoder pose from the same log row.
The mapper rotates and translates the LiDAR offset and beams into the
startup odometry frame. It marks ray interiors as free evidence and hit
endpoints as occupied evidence. Evidence is capped, and an endpoint takes
priority over a free observation of the same cell within a scan.

The JSON supplies the timestamp, final pose, grid dimensions, resolution,
and origin in the startup odometry frame. The origin is the lower-left
corner of cell `grid[0][0]`; increasing indices follow +x and +y. Convert a
position to a grid index by subtracting the origin, dividing by resolution,
then rounding down. Values are `-1` unknown, `0` free, and `100` occupied.
Map bounds can change as new areas are observed, so use each snapshot's origin.

This baseline assumes a single horizontal cylindrical LiDAR layer. It skips
null or out-of-range returns because a null in the sensor log does not
identify whether a return was too near or too far. The map uses encoder
odometry and will inherit drift and wheel slip. Scan matching, loop closure,
and live map publishing remain future integration work.

## Live mapping in Thomas's final warehouse

The separate live mapper combines encoder prediction with local scan-to-grid
alignment. It initializes at AMR01, integrates selected scans after motion,
and atomically publishes the latest warehouse-frame grid as new scans arrive.
From the repository root in PowerShell:

```powershell
$env:G18_WAREHOUSE_MAP = Join-Path $PWD 'webots/results/mapping/live.json'
$world = Join-Path $PWD 'webots/worlds/final_warehouse.wbt'
& 'C:/Program Files/Webots/msys64/mingw64/bin/webots.exe' $world
```

Use the normal keyboard controller to visit additional aisles. Starting
elsewhere requires setting G18_LOCALIZATION_START to the initial [x,y,yaw]
in warehouse metres and radians. This mapper targets the final warehouse.
The JSON provides the grid, origin, estimated pose, time, and surveyed station
locations. Station labels are annotations only. LiDAR supplies the occupancy
observations. Read the current warehouse order to pair a lane with a dock.
Transient obstacles may appear in the map; use fresh local sensing for driving.

To reproduce the scripted delivery-corridor evaluation, run
`py webots/tools/smoke_final.py --mapping`, then
`py -m webots.localization.check_route_map TRIAL_FOLDER webots/results/mapping/final-P03-D01`
with Matplotlib installed. Substitute the run directory printed by the smoke
test. The test driver uses privileged steering; the mapper sees only encoders,
LiDAR and the configured start. Evaluation compares poses at integrated scans
and searches the known free grid between the P03 and D01 approach points
with 0.825 m radial clearance plus a cell-corner margin. The output describes
the observed corridor. Unseen parts of the hall remain unknown.
