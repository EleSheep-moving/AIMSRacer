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

Across the bag, raw FAST-LIO odometry receipt age was 78.2 ms median / 119.6 ms
P95; the rear-axle output was 81.9 / 122.6 ms. Matching the two streams by
source stamp gives a 3.6 ms median / 5.8 ms P95 extra bag-receipt interval for
the rear-axle adapter and delivery. Source stamps stayed close to 100 ms apart
(100.8 ms P95), whereas rear-axle bag-receipt intervals had a 129.2 ms P95.

| Bag elapsed time | Operating phase | Rear-axle LIO age median / P95 |
| --- | --- | ---: |
| 0–40 s | Mostly stationary | 74.6 / 100.6 ms |
| 40–80 s | Driving, mostly straight | 90.6 / 125.8 ms |
| 80–120 s | Driving, substantial turning | 105.5 / 149.1 ms |
| 160–300 s | Mostly stationary again | 77.0 / 96.9 ms |

The age rose during the driving phase and returned near its starting level
after stopping; it did not grow monotonically with node uptime. Motion,
environment, CPU scheduling and ROS/bag delivery are confounded here, so this
does not establish that turning itself increased LIO compute time. Around bag
seconds 315–318, LIO and raw Livox IMU both delivered buffered old stamps and
several topics had multi-second receipt gaps. Those outliers cannot be assigned
to FAST-LIO computation from this bag. Per-scan execution time requires timing
inside FAST-LIO or at its publisher; raw LiDAR would additionally separate
scan acquisition, but was not recorded.

Within bag seconds 33–150 at wheel speed above 0.3 m/s, source-time-aligned
wheel speed and LIO receipt age had Spearman rank correlation 0.03. The
0.9–1.1 m/s group (536 samples) had 99 ms median age, while the faster
1.1–1.3 m/s group (108 samples) had 88 ms. This limited, confounded speed
range does not show a monotonic speed-versus-delay relationship; it says
nothing about operation above roughly 1.5 m/s.

Rapid yaw changes were checked separately using the absolute change in
5-sample-median-filtered raw IMU yaw rate over the preceding 100 ms, divided
by 0.1 s. Among 842 moving scans in the same 33–150 s window, this yaw-rate
change proxy had only 0.04 overall Spearman correlation with rear-axle LIO
receipt age. The highest proxy quartile had 99.3 ms median / 152.1 ms P95 age,
versus 95.6 / 142.8 ms in the lowest quartile. Within 10 s intervals, the
median high-minus-low-quartile age difference was 9.2 ms (positive in 10 of
11 intervals); using raw FAST-LIO odometry gave 7.3 ms (10 of 11). Further
stratifying those intervals by absolute yaw rate reduced the median difference
to 6.1 ms (positive in 13 of 20 strata). This is a weak association between
turning transients and message age, not evidence of a measured increase in
FAST-LIO's internal per-scan compute time. The bag has no internal timing,
raw LiDAR, or CPU scheduling trace to separate those causes.

### EKF versus delayed LIO at matching source times

An offline diagnostic used 839 samples from bag seconds 33–150 with measured
wheel-speed magnitude above 0.3 m/s. For each rear-axle LIO pose stamped at time
`t`, it selected the EKF output stamped within 8 ms of `t`, recorded no later
than `t + 35 ms` and before that LIO message arrived, then compared both poses
in `odom/base_link`. As causal baselines, it held the most recently received LIO
pose or propagated that pose to `t` with its reported planar twist. The later
LIO pose is only a proxy for truth; it is also an EKF input, so this tests
temporal consistency rather than independent localization accuracy.
The rear-axle twist uses FAST-LIO linear velocity and IMU angular velocity;
upstream FAST-LIO leaves its Odometry angular-velocity field at zero.

| Position difference from later LIO pose | Median | RMS |
| --- | ---: | ---: |
| EKF at matching source time | 8.54 cm | 9.51 cm |
| Most recently available LIO pose, held | 10.37 cm | 14.28 cm |
| Most recently available LIO pose, constant-twist propagated | 0.36 cm | 0.93 cm |

In this selection, LIO receipt age was 96 ms median; EKF receipt age was 7 ms.
The EKF pose was on average behind the matching LIO pose along the direction of
travel. Comparing EKF poses against LIO poses about 100 ms *earlier* reduced
position RMS from 9.51 cm to 2.12 cm, indicating an apparent pose lag despite
fresh EKF output stamps. This does not by itself prove which filter update or
configuration caused the lag. The repository EKF configuration at the time of
this diagnostic did not enable delayed-measurement history/replay.

### Isolated replay with delayed-measurement history — 2026-09-29

The same bag inputs were replayed at real-time speed into separate EKF runs on
the Orin NX, with a 400 Hz simulated clock and all other EKF parameters held
constant. The comparison used 226 moving LIO samples from bag seconds 80–103,
matched to EKF outputs published before their corresponding LIO result arrived.
Enabling `smooth_lagged_data` with `history_length: 0.5` changed the same-time
LIO/EKF position-difference median/RMS from **8.82/9.92 cm** to **0.78/1.37 cm**.
Yaw-difference median changed from 0.0966 rad to 0.0018 rad. EKF output age
remained a few milliseconds, but the observed output count was about 181 Hz
without replay and 168 Hz with it over this 28.4 s isolated run. These rates
include bag playback and collector scheduling and are not a full-stack vehicle
benchmark. History replay is now enabled in the main-repository EKF configuration;
verify rate and CPU headroom with all vehicle processes running before relying
on the improvement in MPCC control. In the 33–150 s moving window, LIO age
P99 was 167 ms and the maximum was 187 ms, so all those messages fit the 0.5 s
history. Across the whole bag, 27 LIO messages exceeded 0.5 s, concentrated
around startup/shutdown; those outliers are not covered by this history length.

In the same replay window, 131 samples had absolute rear-axle yaw rate above
0.3 rad/s (52 left, 79 right). Their median speed was 0.91 m/s and median
absolute yaw rate was 1.29 rad/s. Comparing EKF output against later LIO at
the same source time gave the following turn-only differences:

| History replay | Position median / P95 | Absolute yaw median / P95 |
| --- | ---: | ---: |
| Disabled | 8.55 / 16.86 cm | 0.1259 / 0.1863 rad |
| Enabled, 0.5 s history | 1.17 / 3.09 cm | 0.0028 / 0.0168 rad |

These values assess agreement with LIO during low-speed turns, not absolute
pose accuracy, sideslip, or performance at higher speed.
