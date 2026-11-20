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
