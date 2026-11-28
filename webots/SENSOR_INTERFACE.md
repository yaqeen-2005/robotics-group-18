# LiDAR and wheel odometry — SLAM handover

Webots R2025a, native Python. No ROS, ground truth, or plotting required by the sensor interface. Use the existing robot controller and step loop.

## Controller integration

Import `SensorInterface` from `sensor_interface`. After creating the existing `lidar` and `odometry` objects, create:

```python
sensors = SensorInterface(
    lidar.resolution, lidar.fov,
    lidar.lidar.getMinRange(), lidar.lidar.getMaxRange()
)
```

Inside the existing simulation loop, after collecting `now`, `scan` and `angles` (left/right encoders), replace the existing odometry update with:

```python
odom_updated = odometry.update(angles[0], angles[1], now)
sensor_frame = sensors.capture(now, scan, odometry, odom_updated)
```

Do not update odometry twice in a step. The first encoder sample establishes the baseline, so the first frame is skipped. Invalid or duplicate odometry updates and scans with unexpected length yield `None`. Keep the existing motor and lift controls unchanged. SLAM can consume `sensor_frame` in this loop, only if it is not `None`.

## Frame fields

| Field | Meaning |
|---|---|
| `time_s` | Webots simulation time when the observations were read; seconds, not a per-ray timestamp |
| `ranges_m` | Original ordered scan; metres, including infinity and other invalid measurements |
| `angles_rad` | Approximate ray-centre angles in radians, same order and length as ranges; left positive |
| `points_lidar_m` | Valid 2D obstacle endpoints relative to LiDAR, metres |
| `points_base_m` | Same endpoints relative to drive axle, metres |
| `odom_pose` | Estimated `(x, y, heading)` relative to initial axle position and heading; metres, radians |
| `velocity` | Estimated `(linear, angular)` velocity, m/s and rad/s |
| `lidar_offset_m` | Sensor origin relative to axle `(0.52, 0.0)` metres; no yaw offset |
| `min_range_m`, `max_range_m` | Range limits (original model: 0.05 and 12 m) |

All local frames use +x forward, +y left and positive yaw counterclockwise. The original model has a single 180-degree cylindrical LiDAR layer with 360 readings. Angular bin centres are an approximation consistent with the current `lidar.py`. Filtered points do not retain raw indices; use the paired `ranges_m` and `angles_rad` for ray-based mapping. Infinity can represent either too near or too far, so do not automatically mark an infinite ray free. An all-invalid scan has no hits, not proof of empty space. The low LiDAR can see beneath cargo; it does not detect all overhead obstacles.

Use `points_base_m` for robot-relative scan matching; for occupancy-grid raycasting, start each ray at the transformed `lidar_offset_m`, not at the axle. SLAM owns the map-frame transform and drift correction. Odometry is not warehouse ground truth. The physical motor-control track remains 0.82 m; the current 0.911 m odometry-only effective track is a provisional calibration based on manual turns, not a measured physical dimension. Revalidate with simulator ground truth in evaluation only. No autonomous path following or cargo motion control is provided here.

## Tests

From repository root:

```powershell
python -m unittest discover -s webots/tests -p "test_*.py"
```

Then open the small warehouse in Webots and check that frames update while driving, turning and stopping. Check raw range count, pose, velocities, and that all existing controls and snapshot plots still work. Absolute sensor accuracy, effective track calibration and actual SLAM integration remain to be validated. This is a proposed in-process interface; agree it with mapping/localisation teammates before they depend on it.
