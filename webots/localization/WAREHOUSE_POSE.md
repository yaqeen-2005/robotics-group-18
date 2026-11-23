# Warehouse-frame localization

The Webots manual controller can publish the latest estimated pose at 10 Hz
of simulation time. For the unmodified final warehouse spawn, set these
variables before launching Webots from PowerShell in the repository root:

```powershell
$env:G18_LOCALIZATION_START = '[-38, -3.3, -1.570796327]'
$env:G18_LOCALIZATION_POSE = Join-Path $PWD 'webots/results/localization/pose.json'
$world = Join-Path $PWD 'webots/worlds/final_warehouse.wbt'
& 'C:/Program Files/Webots/msys64/mingw64/bin/webots.exe' $world
```

Use an unused output path for each run. The initial pose is the configured
parking bay in warehouse ENU metres and radians. Supply a different pose if
the robot starts elsewhere. The JSON fields are `frame`, `time_s`, `x_m`,
`y_m`, and `yaw_rad`. Consumers should check that simulation time advances
before using a sample. Odometry updates on every valid encoder pair.

## Repeat the final warehouse trial

```powershell
py webots/tools/smoke_final.py --localization
py -m webots.localization.evaluate <path-to-paired.csv>
```

The fixture uses seed 18000 and a scripted P03-to-D01 delivery with privileged
steering. The estimator receives the configured starting pose and wheel
encoders. Supervisor truth is recorded alongside estimates for evaluation.
The closing sample is after the robot backs out of the receiving dock.

In the local trial, over 3,995 paired samples, position RMSE was 4.92 m,
heading RMSE was 14.01 degrees, and the final error was 8.34 m and
10.16 degrees. The estimate uses the same frame as the station coordinates.
This encoder baseline requires external pose correction before closed-loop
autonomous delivery or docking. Keep it as an odometry input when integrating
a scan-based localization system.
