<a id="系统脚本"></a>
# System scripts

**English** · [简体中文](README.zh-CN.md)

The current vehicle runs through [entry points in one workspace](../../../docs/operations/bringup.md). The table lists installed helper entry points. Persistent gyro correction, rear-axle odometry and the acceleration observer were migrated to C++ on 2026-10-15; `imu_to_rear_axle` and `localization_monitor` were already C++. See [migration and validation](../../../docs/reports/2026-10-15-native-state-nodes.md). Retained Python reference wrappers are not installed as production nodes.

| Script | Purpose |
| --- | --- |
| `livox_gyro_bias` (C++) | Outputs gyro bias corrected IMU and diagnostics, retaining raw IMU |
| `lio_to_rear_axle` (C++) | Converts raw LIO to rear axle odometry; owns odom/base_link TF during mapping |
| `longitudinal_acceleration` (C++) | Optional wheel + IMU acceleration/status publisher; native observer has Python numerical references |
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

Save maps with [tools/save_map.sh](../../../tools/save_map.sh); associate closed-loop CSV with a saved PGO map using [tools/reference](../../../tools/reference/README.md). Current calibration instructions are in [calibration materials](../docs/calibration.md); that page links the current source and operation guides.

## Current and lateral-acceleration calibration

[Current calibration A/B/C and selected results](../../../docs/operations/calibration.md#current-calibration) · [Lateral AY validation and grip status](../../../docs/operations/lateral-acceleration-calibration.md). Source is unified under `src/calibration/`; use the restored C++ lateral entry. Historical Python commands are provenance only.
