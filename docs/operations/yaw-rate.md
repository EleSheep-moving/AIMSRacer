# Independent body yaw rate and angular acceleration

> 兼容单分量接口说明：当前推荐使用[后轴／质心统一五量节点](state-observer.zh-CN.md)。对应源码保留在 `src/aims_racer_system/`，按当前构建流程生成。


**English** · [简体中文](yaw-rate.zh-CN.md) · [Evidence](../reports/2026-10-15-yaw-rate-quality.md)

The C++17 `yaw_rate` node consumes only `/livox/imu_bias_corrected` angular velocity
(rad/s in `livox_frame`). Shared mounting rotation produces body rate, followed by
a causal `[r, alpha_z]` constant-angular-acceleration Kalman observer. There is no
wheel/LIO/navigation-EKF/current/steering input, online bias learning, control or
TF publication. Upstream startup gyro bias is not subtracted a second time.

`r=omega_body.z` is not general Euler yaw derivative when the car rolls/pitches.
Angular velocity needs mounting rotation but no translational lever correction.
The accepted longitudinal-acceleration chain remains unchanged.

| Output | Valid field |
| --- | --- |
| `/state_estimation/yaw_rate` (`sensor_msgs/Imu`) | `angular_velocity.z`, rad/s |
| `/state_estimation/yaw_acceleration` (`AccelWithCovarianceStamped`) | `accel.accel.angular.z`, rad/s² |
| `/state_estimation/yaw_rate/status` (`DiagnosticArray`, 10 Hz) | validity, source/receipt ages, states/covariance, faults, bounded callback timing |

Numeric outputs share the original IMU source stamp and `base_link` frame.
Rate output orientation/acceleration covariance flags are -1; unobserved angular
x/y covariance diagonals are 1e6. The acceleration output has 1e6 diagonals for
its five unobserved components. Placeholder zeros are not measured zeros.
Covariance is conditional model uncertainty, not calibrated accuracy or a bound
on residual bias. Do not treat the rate message as a complete IMU.

The model uses `F=[[1,dt],[0,1]]`, `H=[1,0]`,
`Q=q*[[dt³/3,dt²/2],[dt²/2,dt]]`, and Joseph covariance update. Defaults are q=4,
R=1e-4 and initial alpha variance=100. They are effective assumptions; response
has gain peaking around 5–10 Hz. No prediction to receipt time or attitude wait.
Duplicate/reversed source times are rejected. A source gap above 30 ms resets
both states; three consecutive samples are required before publication. Source
age above 50 ms, future/invalid stamps, wrong frame, unavailable/nonfinite gyro
are rejected. Diagnostics continue through outages. Clock reversal starts a new
epoch; strict monotonic/unique publication stamps apply within an epoch.
Node-specific parameters are read-only after startup.

With the gyro-bias pipeline already active:

```bash
ros2 launch aims_racer_system yaw_rate.launch.py
```

The launch starts only this observer; accepts `config`, `geometry_config` and
`use_sim_time`. vehicle/race has `yaw_rate:=true`, default false; `record:=true`
captures the three output topics, source IMU and configuration.

NX isolated CMake installation (main install unchanged):

```bash
source /opt/ros/humble/setup.bash
source /home/aims/aimsracer-yaw-observer-20261015/install/share/aims_racer_system/local_setup.bash
ros2 launch aims_racer_system yaw_rate.launch.py
```

Source/receipt ages and the maximum 512-callback window P99 are not physical
response delay or whole-run P99. This remains an observation output, with no
independent dynamic truth or temperature-drift acceptance. The B/C recordings
already show residual gyro-bias variation; smoothing does not identify that bias.
