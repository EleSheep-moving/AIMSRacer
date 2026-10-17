<a id="aimsracer-参考与离线模型工具"></a>
# AIMSRacer reference and offline model tools

**English** · [简体中文](README.zh-CN.md)

The ROS package is named `aims_mpcc`, with source under `src/controller`. It provides `record_path` and `prepare_path`, and retains the vehicle configuration, periodic reference, model and speed-planning tools needed to generate native acados bundles. Online control runs in the C++ node of [aims_mpcc_rt](../aims_mpcc_rt/README.md); the complete vehicle uses [race.launch.py](../aims_racer_system/launch/race.launch.py).

The current default [vehicle.yaml](config/vehicle.yaml) matches the later field runs on 2026-10-10: cruise 3.5 m/s, hard limit 4 m/s, acceleration/braking 1 m/s², planning ay=1 m/s² and at most two RTI passes; the bundle uses N15/dt=.1. The vehicle reference is the rear-axle `base_link`, with footprint front/rear/half-width 0.52/0.10/0.16 m. `enforce_corridor=false` and the combined acceleration ellipse is disabled; saved reference widths are not actual track boundaries.

The field-run planned peak was approximately 2.99 m/s and the measured peak approximately 2.89 m/s; see the [field review](../../docs/reports/2026-10-10-mpcc-field-review.md).

<a id="使用"></a>
## Usage

- [Prepare a reference, export a bundle and run](docs/usage.md)
- [Model, costs and native execution implementation](docs/implementation.md)
- [System frames, topics and command ownership](../../docs/architecture.md)
- [Known-map initialization](../../docs/operations/known-map-mpcc.md)

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run aims_mpcc record_path --ros-args -p output:="$HOME/aimsracer-data/lap.csv"
ros2 run aims_mpcc prepare_path "$HOME/aimsracer-data/lap.csv" \
  "$HOME/aimsracer-data/reference-new" \
  --vehicle-config src/controller/config/vehicle.yaml --left-width 0.5 --right-width 0.5
```


Check the example left/right widths of 0.5 m against the available physical space. `record_path` outputs `odom/base_link`, tied to the localization origin of that session. A persistent map reference first requires a genuine map-frame CSV and a PCD bound with `--map-file`; this parameter does not transform CSV coordinates. Generation and target-platform builds use `src/aims_mpcc_rt/scripts/export_bundle.py` and `build_bundle.py`; detailed commands are in the [usage guide](docs/usage.md#离线生成与目标平台构建).

<a id="构建与来源"></a>
## Build and attribution

Build the complete main workflow from the workspace root:

```bash
export MAKEFLAGS="-j2 -l2"
colcon build --packages-up-to aims_racer_system aims_mpcc_rt --executor sequential \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DROS_EDITION=ROS2 -DDISTRO_ROS=humble
```


Software verification lives under `verification/` and is outside the vehicle startup workflow. Third-party sources and licenses are retained in [NOTICE.md](NOTICE.md), [LICENSE](LICENSE) and [vendor](aims_mpcc/vendor/); the reference and model retain their corresponding attribution.


[Offline map/reference and calibration-region editor](../../tools/reference/editor/README.md) · [轮速电流离线工具](../calibration/longitudinal_current/docs/wheel-current-model.zh-CN.md)
