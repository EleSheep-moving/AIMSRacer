# Independent rear-axle Vx observer

> 兼容单分量接口说明：当前推荐使用[后轴／质心统一五量节点](state-observer.zh-CN.md)。对应源码保留在 `src/aims_racer_system/`，按当前构建流程生成。


[简体中文](velocity.zh-CN.md) · [Measured evidence (Chinese)](../reports/2026-10-15-vx-quality.zh-CN.md)

This parallel observer propagates three-dimensional IMU-origin world position/velocity and an effective acceleration residual. Historical LIO position/orientation updates are replayed to the latest IMU sample. Wheel speed, EKF, LIO twist, current and commands are absent from estimation and tuning. Vicon accuracy acceptance remains outstanding.

| Interface | Contract |
|---|---|
| `/livox/imu_bias_corrected` | IMU frame `livox_frame`; specific force **in g**, gyro in rad/s; original source stamp |
| `/fastlio2/lio_odom` | `odom` → `livox_frame`; physical IMU-origin pose only; zero covariance means unknown |
| `/state_estimation/vx` | `TwistWithCovarianceStamped`, rear `base_link`; only `linear.x` is valid |
| `/state_estimation/vx/status` | Source/receipt ages, replay, rejections, validity, reference and covariance status |
| `/state_estimation/vx/debug` | Internal 3D world velocity, effective residual and position innovation; unverified diagnostic components |

Other twist components are placeholders with diagonal covariance 1e6. Output stamps identify the latest IMU source sample actually used. Effective model covariance is uncalibrated and does not model shared LIO/IMU information as independent evidence. Sensor subscriptions request reliable QoS, matching the production publishers.

After building `aims_racer_system` in a ROS Humble workspace and sourcing its installation, add the observer to an already running sensor graph:

```bash
ros2 launch aims_racer_system velocity.launch.py
```

The validated isolated NX installation is available directly. Keep the sensor graph running and start only the observer in a separate terminal:

```bash
source /opt/ros/humble/setup.bash
source /home/aims/aimsracer-vx-observer-20261015/install-final/share/aims_racer_system/local_setup.bash
ros2 launch aims_racer_system velocity.launch.py
```

Ctrl-C stops this process. The existing vehicle/race `vx_observer` argument defaults to `false`; explicitly enabling it adds this node. Recording captures all three output topics and the configuration snapshot. There are no TF or control publications. Do not launch a second vehicle graph alongside an existing one.

Shared mounting comes from `rear_axle_geometry.yaml`: the inherited approximate rear→IMU lever is `[0.30,0,0.03] m`, with identity rotation. Do not apply FAST-LIO's internal lidar/IMU extrinsics again. CG conversion is deferred; unknown CG height is not filled with zero.

Moving initialization needs at least three LIO positions spanning 0.2 s. Regression after removing IMU preintegrated displacement estimates initial velocity. Neither Vx nor internal vy is constrained to zero.

Failure handling:

- An IMU source gap above 30 ms resets initialization; force is not integrated across the gap.
- IMU source age above 50 ms, future headers and invalid observations are rejected.
- An accepted LIO anchor older than 0.3 s degrades the estimate; above 0.5 s numerical output stops. Diagnostics also check freshness against the receipt clock.
- History is bounded by 2 s and 4096 events. Duplicates, expired data, frame errors, invalid quaternions, nonfinite values and arithmetic failures are rejected transactionally.
- Position NIS above 16.266 or orientation innovation above 0.35 rad rejects an anchor without refreshing it. These are model gates, not calibrated fault probabilities.
- A fresh LIO pose after anchor expiration starts reinitialization. Output stays suppressed until at least three fresh poses span 0.2 s; `lio_reinitializations` records these recoveries.
- Clock rollback clears state and starts a new publication epoch. Output source stamps strictly increase within each epoch; historical correction never republishes old values.

Inspect with `ros2 topic echo /state_estimation/vx/status`. Parameters are read-only. Frozen position standard deviation is 0.03 m; residual random-walk PSD is 1e−4 m²/s⁵. Verification tools（recorded workspace path：`verification/velocity/README.md`） distinguish synthetic truth, field consistency, ROS runtime and pending independent Vicon accuracy. The observer is not accepted for control or sideslip supervision.
