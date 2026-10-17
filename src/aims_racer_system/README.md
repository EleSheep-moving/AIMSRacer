<a id="aimsracer-系统集成"></a>
# AIMSRacer system integration

**English** · [简体中文](README.zh-CN.md)

This package maintains vehicle launch, rear axle data adaptation, gyro bias correction, EKF/localization configuration, and known-map initialization. Current production entry points use one workspace:

| Entry point | Purpose |
| --- | --- |
| `vehicle.launch.py` | MID360, FAST-LIO2, rear axle adaptation, EKF, RC/VESC |
| `mapping.launch.py` | Mapping while driving manually, PGO, and optional recording |
| `race.launch.py` | vehicle, known-map NDT/monitor, native MPCC, and optional recording |
| `longitudinal_acceleration.launch.py` | Independent wheel + IMU acceleration observer only, using an existing vehicle graph |

[Bringup](../../docs/operations/bringup.md) describes source preparation and building; [architecture](../../docs/architecture.md) defines topics, TF, timing, and command ownership consistently.

<a id="已知地图运行"></a>
## Known-map operation

```bash
ros2 launch aims_racer_system race.launch.py \
  map_file:="$HOME/maps/20260928_010503/map.pcd" \
  artifact_directory:="$HOME/aimsracer-data/bundles/field-v35-20261010" \
  record:=true
```

`map_file` and `artifact_directory` are required; `initial_pose` is optional, formatted as **base_link**'s `x y z roll pitch yaw` in map, with angles in rad. If omitted, initialize through the tool or RViz:

```bash
ros2 run aims_racer_system relocalize_known_map.py \
  "$HOME/maps/20260928_010503/map.pcd" --pose-frame base_link \
  --x 0 --y 0 --z 0 --roll 0 --pitch 0 --yaw 0
```

Replace example values with the actual rear axle placement. The [known-map guide](../../docs/operations/known-map-mpcc.md) explains epoch, map identity, and eligibility. `auto_start=false` defaults to manual enable; true lets the native node enable once when ready, and explicit stop cancels waiting. `repeat_laps=false` defaults to stopping after one lap; true continues lapping until stop. Completion/fault does not trigger automatic restart or re-enabling.


## Recording and estimation

`record=true` automatically records raw/corrected IMU, LIO, wheel/EKF, TF, NDT/MPCC status, and the optional acceleration estimate/status when published; race also saves runtime. An empty `session_directory` generates a dated session automatically; an explicitly specified directory must be new. See [recording](../../docs/operations/recording.md) for the complete list and raw IMU reliable/depth=4096 QoS.

`base_link` is the rear axle midpoint. C++ `imu_to_rear_axle` retains the IMU source stamp and unavailability information, converts units, and compensates the lever arm; `lio_to_rear_axle` provides odometry at the same point. The EKF fuses LIO, wheel vx, and IMU yaw rate, retaining 2 s of delayed history. Gyro correction retains separate raw and corrected topics. `livox_gyro_bias`, `lio_to_rear_axle` and the optional `longitudinal_acceleration` now run in C++; see the [2026-10-15 migration and NX tests](../../docs/reports/2026-10-15-native-state-nodes.md).

`/rear_axle/lio_odom` keeps the LIO scan-end stamp and uses gyro matched/interpolated at that source time. `/rear_axle/imu` stays current with incoming IMU and has no dependency on LIO latency; it supplies specific force `f = a − g` and marks orientation unavailable. In a z-up world `g = [0,0,−9.80665] m/s²`; restoring inertial acceleration uses `a = f + g` in the same frame. There is no attitude extrapolation or additional aligned-IMU topic. See [the timestamp correction report](../../docs/reports/2026-10-11-rear-imu-source-time.md).

Body cloud and raw LIO odometry retain the same scan-end stamp for NDT/PGO; world cloud/path are optional display outputs. In race, EKF owns `odom → base_link` and NDT owns `map → odom`; during mapping, rear axle LIO and PGO own these two edges respectively.

## Optional longitudinal acceleration

Add `longitudinal_acceleration:=true` to vehicle/race for field evaluation; the default is `false`. With an existing graph, start only the observer:

```bash
ros2 launch aims_racer_system longitudinal_acceleration.launch.py
```

Use either integrated or standalone launch, with one observer instance. It consumes wheel vx and rear IMU specific force and publishes `/state_estimation/longitudinal_acceleration` plus `/state_estimation/longitudinal_acceleration/status`. Output linear.x is `d(body_vx)/dt`; covariance is model uncertainty, not calibrated field uncertainty. It uses no LIO pose prediction and publishes no commands or TF. Navigation EKF does not fuse this output. Source-age warnings are diagnostics only. See the [operation guide](../../docs/operations/longitudinal-acceleration.md) for initialization, timestamps and two-lap recording.

<a id="构建与证据"></a>
## Build and evidence

```bash
source /opt/ros/humble/setup.bash
export MAKEFLAGS="-j2 -l2"
colcon build --packages-up-to aims_racer_system aims_mpcc_rt --executor sequential \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DROS_EDITION=ROS2 -DDISTRO_ROS=humble
source install/setup.bash
```

See the [root README](../../README.md) for fixed dependency preparation. Software verification is in `verification/`; this package's [calibration notes](docs/calibration.md) and dated [field report](../../docs/reports/2026-10-10-mpcc-field-review.md) preserve specific conditions. The current default v35 bundle's planned peak is approximately 2.99 m/s and the vehicle peak approximately 2.89 m/s; source, build, software checks, and field-run evidence are explained separately.

Optional planar G: `planar_acceleration:=true` (default false); see [operations and known quality limits](../../docs/operations/planar-acceleration.md).

## Current and lateral-acceleration calibration

[Current calibration A/B/C and selected results](../../docs/operations/calibration.md#current-calibration) · [Lateral AY validation and grip status](../../docs/operations/lateral-acceleration-calibration.md). Source is unified under `src/calibration/`; use the restored C++ lateral entry. Historical Python commands are provenance only.

[当前标定包](../calibration/README.zh-CN.md) · [独立状态／Vicon 现场入口](field/README.zh-CN.md)
