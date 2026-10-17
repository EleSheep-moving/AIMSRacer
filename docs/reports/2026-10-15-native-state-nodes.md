# C++ rear axle conversion, gyro correction and acceleration observer

**English** · [简体中文](2026-10-15-native-state-nodes.zh-CN.md) · [Reports](README.md)

On 2026-10-15, three persistent Python sensor/observer nodes were replaced with C++17 executables and built/installed in the NX main workspace. The x86 Humble build and NX build each passed **20 CTest groups**; the NX `colcon test-result` summary reported **119 tests, 0 errors, 0 failures, 0 skipped**. This establishes software and synthetic ROS behavior. No post-migration vehicle-driving run or matched performance benchmark was performed.

## Runtime change

| Installed executable | Responsibility | Launch |
| --- | --- | --- |
| `livox_gyro_bias` | Calibrate gyro in Livox axes, then continuously forward bias-corrected IMU | `rear_axle_frames.launch.py` |
| `lio_to_rear_axle` | Transform source-stamped LIO pose, velocity and covariance to rear axle; optional mapping TF | `rear_axle_frames.launch.py` |
| `longitudinal_acceleration` | Independent wheel-speed/specific-force `[v,a,d]` Kalman observer | `longitudinal_acceleration.launch.py` |

[Rear frame launch](../../src/aims_racer_system/launch/rear_axle_frames.launch.py) and [acceleration launch](../../src/aims_racer_system/launch/longitudinal_acceleration.launch.py) select the native executables. `imu_to_rear_axle` was already C++. Topics, frame names, YAML parameters, default thresholds and launch arguments were retained; `longitudinal_acceleration` remains opt-in and is not navigation or control feedback. The three old Python wrappers are no longer installed by [CMake](../../src/aims_racer_system/CMakeLists.txt); retained Python modules provide numerical references and offline analysis.

The main workspace is the deployment target. Other calibration workspaces and pre-existing immutable MPCC bundles are separate artifacts; this port does not rebuild or alter their vehicle model.

## Behavior preserved and checked

**Gyro correction.** Defaults still require a 10 s stationary window and at least 1,000 samples, with fresh wheel/LIO observations and motion/noise checks. Raw Livox acceleration is interpreted in g during this check. Before calibration there is no corrected output. After success the bias is frozen and the process keeps forwarding real rotation with only that bias removed. Non-increasing IMU source time clears the calibration and external histories. Correction preserves headers, acceleration and unknown all-zero gyro covariance; known gyro covariance receives the existing diagonal increment.

**Rear axle odometry.** LIO retains its exact integer scan-end stamp. Lever-arm velocity compensation uses an exact gyro knot or interpolation between measured knots, with a maximum 30 ms bracket. Missing upper coverage waits; extrapolation and interpolation across excessive gaps are rejected. History remains bounded to 2,000 gyro samples and 20 pending LIO messages. Geometry tests compare every element of pose/twist covariance against a Python/SciPy oracle, including a rotated mount and cross covariance. Native DDS checks cover late LIO, the missing upper knot, TF contents/source stamps, gyro rewind clearing pending LIO and the monotonicity anchor, and excessive gyro gaps.

**Acceleration.** Source measurements retain integer nanoseconds and stable equal-time order. Late measurements replay the retained suffix; time/count trimming preserves a prefix posterior. Joseph covariance updates and numerical checks commit admission only after the whole suffix succeeds. Wheel callbacks update history without publishing; IMU callbacks trigger receipt-time predictions, including unavailable-force heartbeats after initialization. ROS time reversal resets initialization/history. Frame, nonfinite, future-source, age and timing diagnostics retain the established interface.

Native observer tests include 10,000 tiny-variance updates and a replay-suffix overflow that must leave state/history intact. Python/native parity covers seeded ordered, equal-time and late sequences, count/time anchors, invalid inputs and long prediction gaps at UNIX-sized epochs. Synthetic DDS outputs are checked against the Python reference on a controlled ROS clock. See [tests](../../src/aims_racer_system/tests) and [portable validation summary](assets/2026-10-15-native-state-nodes/validation-summary.json).

## Reproduce and inspect

After building the normal Humble workspace with dependencies:

```bash
colcon build --packages-select aims_racer_system --executor sequential \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON
source install/setup.bash
ctest --test-dir build/aims_racer_system --output-on-failure
colcon test-result --test-result-base build/aims_racer_system/test_results --verbose
```

The two counts describe different aggregation levels; do not add 20 CTest groups to 119 XML-reported tests. The NX binaries were verified as ARM aarch64 ELF executables, and 20 source/build/launch files matched x86 source hashes. A source/install backup was retained on NX before applying the migration. Source hashes identify the tested working tree rather than implying the base commit alone contains the migration.

Portable evidence: [desktop CTest](assets/2026-10-15-native-state-nodes/desktop-ctest.txt), [NX CTest](assets/2026-10-15-native-state-nodes/nx-ctest.txt), [summary and source hashes](assets/2026-10-15-native-state-nodes/validation-summary.json), [artifact provenance](assets/2026-10-15-native-state-nodes/artifact-manifest.json). The full archive is external at `$AIMSRACER_DATA/experiments/native-state-port-20261015/`; the NX backup is under `$HOME/aimsracer-native-state-port-20261015/backup-before-deploy.tgz`. Neither external archive is included in a clone.
