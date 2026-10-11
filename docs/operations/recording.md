<a id="录制传感器与控制器数据"></a>
# Recording sensor and controller data

**English** · [简体中文](recording.zh-CN.md)

Launch manages field recording; no separate bag wrapper is needed. In the [same workspace environment](bringup.md), use:

```bash
ros2 launch aims_racer_system race.launch.py \
  map_file:="$HOME/maps/20260928_010503/map.pcd" \
  artifact_directory:="$HOME/aimsracer-data/bundles/field-v35-20261010" \
  record:=true
```

For mapping, use `ros2 launch aims_racer_system mapping.launch.py record:=true`. Recording itself does not grant autonomous control permission.

<a id="目录与数据"></a>
## Directory and data

When `session_directory` is empty, a directory named with time and mode is generated under `~/aimsracer-data/sessions/YYYY-MM-DD/`; an explicitly specified directory must be new.

```bash
ros2 launch aims_racer_system race.launch.py \
  map_file:="$HOME/maps/20260928_010503/map.pcd" \
  artifact_directory:="$HOME/aimsracer-data/bundles/field-v35-20261010" \
  record:=true session_directory:="$HOME/aimsracer-data/sessions/my-new-run"
```

A session contains `bag/`, `session.json`, and system `params/` snapshots; race also writes native `runtime.csv` into the session directory. `record=true` selects this directory as `log_directory`; use `log_directory` alone to record only runtime.

| Data | Automatically recorded contents |
| --- | --- |
| Raw and corrected IMU | `/livox/imu`, `/livox/imu_bias_corrected`, `/imu/gyro_bias/status` |
| Raw inputs for LIO replay | `/livox/lidar`, `/livox/imu` |
| LIO and rear axle data | `/fastlio2/lio_odom`, `/fastlio2/body_cloud`, `/rear_axle/lio_odom`, `/rear_axle/imu` |
| wheel / EKF | `/rear_axle/wheel_odom`, `/odometry/filtered` |
| RC and execution commands | `/rc/channels`, `/ackermann_cmd`, `/control/autonomy_speed_enabled`, motor speed/current, servo position, `/sensors/core` |
| TF | `/tf`, `/tf_static`, isolated `/fastlio2/tf` |
| race localization and control | `/localization/status`, anchor/ndt status, map identity/valid, NDT/bridge pose, `/drive`, `/mpcc/status`, reference/prediction |
| mapping | `/pgo/loop_markers` |

The complete list is maintained in [launch_support.py](../../src/aims_racer_system/aims_racer_system/launch_support.py). Recording uses [recording_qos.yaml](../../src/aims_racer_system/params/recording_qos.yaml): **raw `/livox/imu` is reliable, keep_last, depth=4096, volatile**, preserving high-frequency inputs; other topics use rosbag's publisher QoS adaptation. Keep `/fastlio2/body_cloud` aligned with raw LIO scan-end time for NDT replay; world cloud is optional display output.

<a id="结束与核验"></a>
## Finish and verify

Stop the vehicle and revoke autonomous permission, then close launch normally with Ctrl-C so rosbag can finish writing its index:

```bash
ros2 bag info "$HOME/aimsracer-data/sessions/my-new-run/bag"
```

Check topic count, message counts, duration, and `metadata.yaml`; retain the complete bundle, map file and hashes, source version, field start/end points, and intervention notes. Keep raw bags, bundles, and logs outside the source directory.

<a id="时间和圈时口径"></a>
## Time and lap-time conventions

The source stamp is the measurement time; bag receive time includes transmission and queuing, so analyze them separately. A fresh EKF header does not mean LIO measurements have no delay; `/ackermann_cmd` is a forwarded command, and servo command echo is not a steering angle measurement.

The motion window in the current [field review](../reports/2026-10-10-mpcc-field-review.md) is the interval between the first and last `abs(vx)>0.05 m/s` after the first RUNNING state, including a stationary start and finish braking; it is not a flying lap. RUNNING→COMPLETE additionally includes waiting at the end. For lap-time comparisons, fix the timing cross-section, repeat with the same configuration, and separately record motion, completion, stopped waiting, errors, and operator interventions.

<a id="mpcc-参考录制"></a>
## Recording an MPCC reference

`record_path` saves raw `odom/base_link` CSV, taking `/odometry/filtered` as input by default:

```bash
ros2 run aims_mpcc record_path --ros-args -p output:="$HOME/aimsracer-data/lap.csv"
```

Drive forward manually for one lap with a small overlap, then stop the recorder. During mapping, use `-p odom_topic:=/rear_axle/lio_odom` if needed. The CSV's odom origin belongs to that localization session; after saving the map, associate map poses offline to form a true `map/base_link` CSV before preparing a persistent reference. `prepare_path` does not transform frames merely through `--map-file`. See [MPCC usage](../../src/controller/docs/usage.md#准备参考) for reference and bundle preparation.
