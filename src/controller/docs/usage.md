<a id="native-mpcc-使用"></a>
# Native MPCC usage

**English** · [简体中文](usage.zh-CN.md)

Current online control runs in `aims_mpcc_rt/mpcc_rt_node`. `aims_mpcc` only provides reference recording/preparation, with model and numerical tools used for offline generation. Complete vehicle operations are covered by [bringup](../../../docs/operations/bringup.md), [map initialization](../../../docs/operations/known-map-mpcc.md) and [recording](../../../docs/operations/recording.md).

<a id="当前实跑配置"></a>
## Current field-run configuration

The default [vehicle.yaml](../config/vehicle.yaml) contains the 2026-10-10 v35 parameters: N15/dt=.1 (1.5 s preview), at most two RTI passes, a 20 Hz request cap and 50 Hz commands. Cruise is 3.5, the hard limit 4, acceleration/braking 1 and curvature planning ay=1, with units m/s and m/s² respectively. The rear-axle footprint has front 0.52, rear 0.10 and half-width 0.16 m; the steering lag of 0.08 s is a lumped model approximation of the current vehicle yaw response.

`enforce_corridor=false` and `combined_accel_constraint_enabled=false` are conditions of these field runs. Physical speed/acceleration/braking/steering angle/steering rate, finite values and input/localization qualification are still checked; planning ay provides no runtime tire-constraint guarantee. The planned peak was approximately 2.99 and measured peak approximately 2.89 m/s. The [field review](../../../docs/reports/2026-10-10-mpcc-field-review.md) explains the different timing definitions for motion windows and COMPLETE.

<a id="准备参考"></a>
## Prepare a reference

The current deployed bundle already includes its reference, so normal operation does not require repeated preparation. First record a new reference:

```bash
ros2 run aims_mpcc record_path --ros-args -p output:="$HOME/aimsracer-data/lap.csv"
```


Drive forward manually for one lap and retain a small overlap. By default this records `odom/base_link` from `/odometry/filtered`; for mapping, `odom_topic` can be changed to `/rear_axle/lio_odom`. An `odom` CSV is tied to the current session origin. For reuse after restarting, first associate it offline with the saved map poses to obtain a `map/base_link` CSV. Historical association examples are in the [map-reference report](../../../docs/reports/2026-09-28-map-reference.md). Retained recovery scripts and input descriptions are in [tools/reference](../../../tools/reference/README.md).

```bash
ros2 run aims_mpcc prepare_path /absolute/path/to/closed_map.csv \
  "$HOME/aimsracer-data/reference-new" \
  --vehicle-config src/controller/config/vehicle.yaml \
  --left-width 0.5 --right-width 0.5 \
  --map-file "$HOME/maps/20260928_010503/map.pcd"
```


`--map-file` binds the PCD SHA-256 to a map-frame CSV and does not transform coordinates. Left/right widths must reflect confirmed space; the example 0.5 m value is a course model. `--start-time` and `--end-time` can extract one closed loop. Preparation rejects invalid closure, reversing, discontinuities, crossings, curvature and frame/geometry mismatches; it retains the original CSV and outputs `path.csv`, `metadata.json` and `raw.csv`. The output directory must be new.

A periodic quintic spline defines fixed geometry; speed is planned from curvature and cyclic acceleration/braking propagation, rather than copied from the driver's CSV speed. Currently there is no online trajectory topic input, position-based speed zones or online racing-line optimization.

<a id="离线生成与目标平台构建"></a>
## Offline generation and target-platform build

First install pinned acados following [source preparation](../../../docs/operations/bringup.md). The offline Python environment needs CasADi 3.7.2, NumPy, SciPy, PyYAML and acados_template from the pinned source. Set the following at the workspace root:

```bash
export ROOT="$PWD"
export ACADOS_SOURCE_DIR="$ROOT/dependencies/work/acados/source"
export ACADOS_INSTALL_DIR="$ROOT/dependencies/work/acados/install"
export LD_LIBRARY_PATH="$ACADOS_INSTALL_DIR/lib:${LD_LIBRARY_PATH:-}"
export PYTHONPATH="$ROOT/src/controller:$ACADOS_SOURCE_DIR/interfaces/acados_template:${PYTHONPATH:-}"
python3 src/aims_mpcc_rt/scripts/export_bundle.py \
  --config src/controller/config/vehicle.yaml \
  --reference "$HOME/aimsracer-data/reference-new" \
  --output "$HOME/aimsracer-data/bundles/reference-new-v35" \
  --horizon 15 --dt 0.1 --source-only
python3 src/aims_mpcc_rt/scripts/build_bundle.py \
  --bundle "$HOME/aimsracer-data/bundles/reference-new-v35" \
  --acados-install "$ACADOS_INSTALL_DIR"
```


`--source-only` generates fixed C and model data; the subsequent step compiles for the current CPU. A complete source bundle generated on x86 can be copied to NX, where `build_bundle.py` is then run; x86 shared libraries cannot serve as NX artifacts. After target compilation, retain the whole bundle, including `input_reference/`, `input_config.yaml`, `manifest.json`, `native_manifest.json`, generated C and shared libraries. Changes to config, reference or generator require a new bundle created offline.

The online node verifies hashes, ABI, platform and loaded libraries; it does not generate or compile online. Record build evidence separately from vehicle results; software regressions belong in `verification/`.

<a id="在线启动参数"></a>
## Online launch parameters

Recommended complete vehicle entry point:

```bash
ros2 launch aims_racer_system race.launch.py \
  map_file:="$HOME/maps/20260928_010503/map.pcd" \
  artifact_directory:="$HOME/aimsracer-data/bundles/field-v35-20261010" \
  record:=true
```


| race parameter | Default / meaning |
| --- | --- |
| `map_file` | Required; the exact immutable PCD |
| `artifact_directory` | Required; complete bundle already built for the target platform |
| `initial_pose` | Empty; optionally `x y z roll pitch yaw` of base_link in map, angles in rad |
| `auto_start` | false; starts once after waiting for the same enable conditions |
| `repeat_laps` | false; stops after one lap; true runs continuous laps |
| `record` | false; true automatically records bag and runtime |
| `session_directory` | Empty; automatic dated session directory; an explicit directory must be new |
| `log_directory` | Empty; native runtime only; record=true selects the session directory |

To connect to an existing vehicle/localization graph, use the standalone native entry point:

```bash
ros2 launch aims_mpcc_rt mpcc.launch.py \
  artifact_directory:="$HOME/aimsracer-data/bundles/field-v35-20261010"
```


The native launch derives `input_reference/` and `input_config.yaml` from the bundle, with horizon/dt from the manifest. Paths, vehicle configuration and horizon need not be repeated. The native entry point also provides `auto_start_timeout=60`, `solve_frequency=20` Hz, `solver_timeout=.05` s, `handover_delay=.02` s and `odom_topic=/odometry/filtered`; parameters are fixed after startup. The complete delivery budget includes worker execution and delivery wait and does not equal solver duration alone. Original source TTL is `0.8×N×dt=1.2 s`; discarded results or new deliveries do not renew it.

<a id="初始化启用与停止"></a>
## Initialization, enable and stop

Initialize with a map/base_link pose or RViz according to [known-map operation](../../../docs/operations/known-map-mpcc.md). `/mpcc/status` and `/localization/status` describe actual qualification. To enable/stop manually:

```bash
ros2 service call /mpcc/enable std_srvs/srv/SetBool '{data: true}'
ros2 service call /mpcc/enable std_srvs/srv/SetBool '{data: false}'
```


Enable requires a ready solver, fresh rear-axle state, actual forwarded history, RC permission, trusted map anchor, a unique `/drive` publisher, a stationary vehicle and matching reference direction. `auto_start=true` reuses these conditions and starts only once after success within the default 60 s window; explicit stop cancels the wait. Completion or faults do not automatically re-enable, and fatal faults do not restart.

`repeat_laps=false` counts one lap from the actual enable position and stops; true runs continuous laps until stop. Use RC/emergency stop for field emergencies, revoke permission, wait until stationary and then close the launch. Log content and timing definitions are in [recording](../../../docs/operations/recording.md).
