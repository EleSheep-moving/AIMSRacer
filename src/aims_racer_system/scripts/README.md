<a id="系统脚本"></a>
# System scripts

**English** · [简体中文](README.zh-CN.md)

The current vehicle runs through [entry points in one workspace](../../../docs/operations/bringup.md). Python scripts are explicitly installed by the package; the table below lists scripts used in the main workflow. C++ `imu_to_rear_axle` and `localization_monitor` are nodes compiled by this package.

| Script | Purpose |
| --- | --- |
| `livox_gyro_bias.py` | Outputs gyro bias corrected IMU and diagnostics, retaining raw IMU |
| `lio_to_rear_axle.py` | Converts raw LIO to rear axle odometry; owns odom/base_link TF during mapping |
| `longitudinal_acceleration.py` | Optional wheel + IMU acceleration/status publisher; canonical observer is in the package Python module |
| `activate_ndt.py` | Configures and activates the NDT lifecycle |
| `relocalize_known_map.py` | Initializes from an explicit six-dimensional map/base_link pose and waits for a new trusted epoch |

For acceleration evaluation, add `longitudinal_acceleration:=true` to vehicle/race (default `false`) or run `ros2 launch aims_racer_system longitudinal_acceleration.launch.py` against an existing graph. The standalone launch starts only this observer. Use one instance; it publishes no commands or TF and is not fused into navigation EKF. See the [acceleration guide](../../../docs/operations/longitudinal-acceleration.md) and [offline/runtime verification](../../../verification/acceleration/README.md).

Known-map initialization uses the installed entry point:

```bash
ros2 run aims_racer_system relocalize_known_map.py \
  "$HOME/maps/20260928_010503/map.pcd" --pose-frame base_link \
  --x 0 --y 0 --z 0 --roll 0 --pitch 0 --yaw 0
```

Replace values with the actual rear axle pose; angles are in rad. RViz can also be used; see [known maps](../../../docs/operations/known-map-mpcc.md) for full details. Localization replay and development audits are in root [verification](../../../verification/README.md) and are not installed as production entry points.

Save maps with [tools/save_map.sh](../../../tools/save_map.sh); associate closed-loop CSV with a saved PGO map using [tools/reference](../../../tools/reference/README.md). Historical calibration collection and response evidence are in [calibration materials](../docs/calibration.md); that page links the archives and current operation guides.
