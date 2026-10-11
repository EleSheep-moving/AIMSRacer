# Native acados/C++ MPCC

**English** · [简体中文](README.zh-CN.md)

`aims_mpcc_rt` is the current online control package. Offline Python freezes vehicle configuration, periodic quintic reference and curvature-based speed planning, then generates acados SQP-RTI C. Online C++ loads and validates the bundle and handles state/command history, RTI, takeover, stopping and ROS output.

The current default field-run configuration is **N15, dt=.1 s, at most two RTI passes, a 20 Hz request cap and 50 Hz commands**; the complete delivery budget is 50 ms, anticipated takeover lead time 20 ms and original source TTL 1.2 s. Six principal states including virtual progress, three controls and three auxiliary history states retain rear-axle kinematics, first-order steering lag and the exact periodic reference. The current configuration uses `rate_bounded_v2` and independent acceleration/braking constraints, with the combined acceleration ellipse and corridor disabled; planning ay=1 still affects curvature-based speed.

The [field review](../../docs/reports/2026-10-10-mpcc-field-review.md) records multiple field-run settings on Orin NX with the actual FAST-LIO2/EKF/NDT chain. 3.5 m/s is configured cruise; the planned peak was approximately 2.99 and the measured peak approximately 2.89 m/s. The label does not establish that 3.5 was reached. Motion windows include startup and braking and are not flying laps.

<a id="准备与构建"></a>
## Preparation and build

Build in the same workspace:

```bash
python3 tools/setup_dependencies.py --workspace .
bash tools/setup_acados.sh --workspace . --jobs 2
source /opt/ros/humble/setup.bash
export MAKEFLAGS="-j2 -l2"
colcon build --packages-up-to aims_racer_system aims_mpcc_rt --executor sequential \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DROS_EDITION=ROS2 -DDISTRO_ROS=humble
source install/setup.bash
export LD_LIBRARY_PATH="$PWD/dependencies/work/acados/install/lib:${LD_LIBRARY_PATH:-}"
```


Offline export/build uses this package's `scripts/export_bundle.py` and `scripts/build_bundle.py`; full parameters and environment are in [MPCC usage](../controller/docs/usage.md#离线生成与目标平台构建). The pinned acados commit and submodules are in the [dependency manifest](../../dependencies/manifest.json). The target installation is `dependencies/work/acados/install`; offline, set `ACADOS_SOURCE_DIR` to `source` and `ACADOS_INSTALL_DIR` to `install`. Runtime needs only the current installation's shared-library path.

The complete bundle must retain `input_reference/`, `input_config.yaml`, source/native manifests, generated source and target CPU libraries. The manifest binds configuration, reference, generator, platform and loaded-dependency hashes. Online startup rejects incompatible artifacts and performs no online generation/compilation. Every configuration/reference change requires a new bundle created offline.

<a id="启动"></a>
## Launch

For the complete vehicle, use:

```bash
ros2 launch aims_racer_system race.launch.py \
  map_file:="$HOME/maps/20260928_010503/map.pcd" \
  artifact_directory:="$HOME/aimsracer-data/bundles/field-v35-20261010" \
  record:=true
```


To connect to an already running vehicle/localization graph, launch separately:

```bash
ros2 launch aims_mpcc_rt mpcc.launch.py \
  artifact_directory:="$HOME/aimsracer-data/bundles/field-v35-20261010"
```


The launch derives reference/config automatically from the bundle; N/dt comes from the manifest. Defaults are disabled, `auto_start=false` and `repeat_laps=false`. Optional `auto_start=true` performs enable once after readiness, waiting 60 s by default; explicit stop cancels the wait, successful startup does not re-enable repeatedly, and fatal faults do not restart. `repeat_laps=true` runs continuous laps until stop. Initial pose and full parameters are in [known-map operation](../../docs/operations/known-map-mpcc.md) and the [usage guide](../controller/docs/usage.md#在线启动参数).

<a id="接口与执行边界"></a>
## Interfaces and execution boundaries

| Interface | Purpose |
| --- | --- |
| `/odometry/filtered` | Rear-axle `odom/base_link` state |
| `/ackermann_cmd` | Forwarded input history |
| `/control/autonomy_speed_enabled` | RC autonomous speed permission |
| `/localization/status`, anchor/map identity | Trusted map qualification and alignment |
| `/drive` | Unique native control proposal, executed by the RC selector |
| `/mpcc/status`, reference/prediction | Status and visualization |
| `/mpcc/enable` | SetBool explicit enable or stop |

Takeover validates original source age, actual prefix, new physical state and alignment; rejecting a candidate does not extend the old plan's lifetime. Input validity, localization qualification, command ownership, finite values and physical limits have separate responsibilities; faster solving does not relax these boundaries. The full execution chain is in the [implementation guide](../controller/docs/implementation.md#measurement-time-computation-and-takeover).

`log_directory` writes `runtime.csv`; complete vehicle `record=true` automatically stores it in the same session as a bag of raw/corrected IMU, LIO, EKF/wheel, TF and NDT/MPCC status. Worker and delivery duration, takeover and output timing are retained separately; see [recording](../../docs/operations/recording.md).

Software verification programs live under `verification/` and are not installed as vehicle production entry points. Historical numerical, isolated ROS, NX combined-load and field-run reports are retained in [reports](../../docs/reports/README.md). Copyright and attribution are in [NOTICE.md](NOTICE.md) and the [controller NOTICE](../controller/NOTICE.md).
