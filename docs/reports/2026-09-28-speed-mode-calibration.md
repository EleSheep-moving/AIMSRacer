# Speed-mode low-speed calibration — 2026-09-28

Source: local bag `/home/aims/mpcc-calib-20260928-005010/`, 324.37 s and
631,357 messages. All 64,040 `/ackermann_cmd` samples had `drive.jerk == 0`
(speed mode). The bag has commanded speed/steering, VESC telemetry, wheel/LIO/EKF
odometry and 200 Hz raw IMU; it lacks raw `/livox/lidar` and was recorded before
the later mapping session. The VESC ERPM gain was already 3465 ERPM/(m/s).
The core ratios, lags and timing can be recomputed from the workspace root with:

```bash
python3 src/controller/tools/analyze_speed_mode_bag.py \
  /home/aims/mpcc-calib-20260928-005010/mpcc-calib-20260928-005010_0.db3
```

## Operating region and speed scale

Moving samples selected for the steering model had corrected wheel speed
0.55–1.45 m/s, median 1.009 m/s (P10–P90 0.852–1.148 m/s). This bag mostly
tests roughly 1 m/s, not exactly 1.2 m/s. On 412 approximately straight LIO
samples with wheel speed 0.4–1.4 m/s, steering command below 0.04 rad and LIO
yaw rate below 0.12 rad/s, the median source-time-aligned LIO/wheel forward
speed ratio was **1.005** (P10–P90 0.970–1.089). Within 0.8–1.2 m/s, the
median was 1.004 on 333 samples. This supports retaining 3465; it does not
establish a new millimetre-accurate ground-speed scale or wheel-slip estimate.

## Longitudinal response

Eleven straight starts had a speed command rising above 0.05 m/s after at least
0.3 s near zero, wheel speed below 0.05 m/s, and steering below 0.05 rad through
the onset window. For each event, subtract the preceding 0.3 s median raw-IMU
x acceleration, apply a causal five-sample median, and require three consecutive
samples above `max(0.5 m/s², 5 × local MAD scale)`. The first sustained forward
acceleration appeared **45–60 ms after the command** (median 52 ms). This is a
detector crossing, not the exact physical instant of motor motion.

A simple speed-command-to-wheel-speed first-order fit over positive-speed
samples from bag seconds 33–150 gave approximately **40 ms delay + 0.16 s
response constant**, with fitted static gain 1.006 and RMS error 0.049 m/s.
These two time terms are correlated and describe the closed VESC speed loop and
drivetrain together. They are **not** acceleration/braking capability. Current
MPCC has no separate longitudinal actuator-delay state, so this result is
recorded for prediction validation; `accel_limit`, `brake_limit` and `jerk_limit`
remain conservative assumptions rather than newly identified parameters.

## Steering-to-yaw response

At 200 Hz, subtract the stationary raw-gyro z bias (about 0.006 rad/s), then
form an effective turn angle `atan(0.36 × yaw_rate / forward_speed)`. A
single-first-order model of this angle following the stamped steering command,
with unit static gain and no separate dead time, fit to moving samples at
**0.0806 s using wheel speed** or **0.0798 s using LIO speed**. The wheel-speed
fit used 15,364 correlated IMU samples and had 0.028 rad RMS error. Repeating
the fit only in the 0.8–1.2 m/s region gave 0.082 s (wheel) and 0.080 s (LIO).
Thus the existing MPCC `steering_tau: 0.08` is supported in speed mode around
1 m/s; it still combines command path, servo, tire/yaw response and delay.

Ten isolated neutral-to-about-±0.475 rad events at wheel speed 0.88–1.03 m/s
had a robust raw-IMU yaw response after median **52 ms**, reached 50% after
**109 ms** and 90% after **159 ms**. Selection required a neutral 0.2 s
baseline, a post-command steering plateau above 0.18 rad, speed above 0.55 m/s,
and a same-sign yaw change above 0.25 rad/s. The event set contains seven
positive and three negative turns, so left/right asymmetry is not precisely
identified. These are command-to-vehicle yaw times, not physical servo-angle
measurements; the available servo topic only echoes a command. Full-lock tests
also exceed the present MPCC's 1 m/s² lateral-acceleration envelope, so 0.08 s
must still be checked during smooth MPCC steering.

The fitted steady effective-angle gain was near 0.96 at about 1 m/s, but that
small difference also admits wheelbase, steering-angle calibration, tire and
gyro errors. With no independent steering-angle or sideslip measurement, an
`understeer_coefficient` cannot be separated reliably from those effects. It
remains zero as a low-speed modelling assumption, **not** a measured zero slip.

## Estimator age and limits

For `/rear_axle/lio_odom`, bag receipt minus source stamp was 82 ms median and
123 ms P95 over all samples. While positive speed was commanded, the median
was 96 ms and P95 **142 ms**; about 40.5% of those LIO messages exceeded
100 ms. The nearly 2.9 s maxima cluster at bag startup/shutdown. The bag did
not capture raw LiDAR, so it cannot decompose scan acquisition, LIO execution
and delivery for this run. A fresh EKF output stamp alone does not establish
that delayed LIO was replayed correctly. Verify this timing in the complete
localization/controller pipeline before autonomous use.
