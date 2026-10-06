# Vehicle speed and steering response — 2026-09-28 analysis

Source: local bag `mpcc-calib-20260927-234227_0.db3` in
`/home/aims/aimsracer-data/sessions/2026-09-27/calibration-234227/bag/` (233.87 s, recorded 2026-09-27).
The bag is not checked into the repository. Tests used V2 on the Orin NX with
MID360. At recording time the VESC conversion was 4650 ERPM/(m/s); afterward
the owner selected 3465 based on a straight-line LIO/wheel speed ratio of
1.342. This report labels wheel speeds after multiplying the recorded values
by 1.342. The MPCC real-car profile currently caps speed at 1.5 m/s.
The bag does not embed the exact repository commit used while recording.

## Speed from rest

Six speed-mode events had at least 0.3 s of wheel speed below 0.05 m/s before the first
`/ackermann_cmd.drive.speed > 0.05 m/s`. The command and IMU were aligned by
their `header.stamp`. For each event, subtract the median raw Livox x-specific
force over the preceding 0.30 s, after converting g to m/s². A causal five
sample (25 ms) median and three successive samples above
`max(0.5 m/s², 5 × 1.4826 × MAD)` define detected forward acceleration.

| Observable | Command-to-first-detection interval | Meaning |
| --- | ---: | --- |
| Raw 200 Hz IMU x acceleration | 52–72 ms; median 55 ms (six events) | Sustained specific-force increase; filter and vibration limit exact physical-onset inference |
| Rear-axle 200 Hz IMU x acceleration | Same six event times | Independent transformation check, but originates from the same physical IMU |
| VESC speed above 100 ERPM | 40–53 ms; median 50 ms (six events) | Motor/wheel rotation; not ground speed by itself |
| LIO body vx above 0.1 m/s | 86–151 ms (three approximately straight, LIO-stationary starts) | First observable ground-motion sample at about 10 Hz; not onset precision |

Stationary raw x-acceleration had a robust noise scale near 0.05 m/s².
At bag times 41.155 and 57.385 s, integrating the bias-subtracted x signal
over 0.2 s gave velocity changes of 0.73 and 0.72 m/s; source-interpolated
LIO vx was about 0.77 and 0.78 m/s. This supports genuine acceleration beyond
an isolated vibration spike, although pitch and mounting tilt remain possible
sources of IMU error. Eight isolated speed-mode command steps reached 50% of
their wheel-speed change in 0.16–0.40 s. This is settling behavior, distinct
from first movement. Those steps had command changes over 0.45 m/s and local
pre/post wheel-speed medians differing by over 0.25 m/s; 50% crossings were
estimated on a 20 ms grid. The values should be rechecked under the new VESC
scale.

## Steering command and yaw response

The 21 selected low-speed events were neutral-to-full or full-to-neutral
steering transitions to/from ±0.475 rad. Corrected wheel speed stayed within
0.55–1.5 m/s (median 1.23) and changed by at most 0.3 m/s near the event.
The previous and next steering plateaus lasted at least 0.25 s, and yaw-rate
change was at least 0.45 rad/s. Seventeen events used VESC current mode
(median speed 1.24 m/s); four used speed mode (median speed 0.87 m/s).

The first steering command change is the source-stamped
`/ackermann_cmd.drive.steering_angle` crossing 0.02 rad from its old plateau.
`/commands/servo/position` and `/sensors/servo_position_command` lack headers;
their delays are measured from bag receipt of the corresponding Ackermann
command to bag receipt of the same threshold crossing. The latter is a VESC
driver **command echo**, not a wheel-angle sensor or hardware acknowledgement.
The positive full command is clipped from 0.475 to about 0.471 rad equivalent
by the servo limit.

IMU response uses raw 200 Hz `/livox/imu.angular_velocity.z`, with a causal
five-sample median and an event-local pre-command baseline. First robust
response requires three successive samples above the larger of 0.05 rad/s
and five local MAD noise scales. The 50% and 90% times use each event's local
yaw-rate change, so they are different from a mechanical steering-angle
measurement.

| Condition | N | Median corrected speed | Command-to-driver echo | Command-to-first robust yaw response | Command-to-50% yaw | Command-to-90% yaw |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| All low-speed turns | 21 | 1.23 m/s | 0.55 ms | 67 ms | 111 ms | 151 ms |
| Current mode | 17 | 1.24 m/s | 0.47 ms | 71 ms | 111 ms | 157 ms |
| Speed mode | 4 | 0.87 m/s | 0.63 ms | 66 ms | 106 ms | 125 ms |

The converter's command topic crossed the threshold after a median 0.27 ms
relative to Ackermann bag receipt. The commanded 10–90% stroke itself took a
median 60 ms. Driver echo times only establish the ROS/VESC command path; they
do not bound the servo's physical travel time. The speed-mode subset is small,
and the two low-speed mode groups were recorded at different speeds.

For context beyond the current MPCC speed cap, 17 speed-mode steering-on
events near 2.92 m/s reached 90% yaw change after a median 221 ms, while
16 steering-off events near 2.84 m/s reached 90% after 148 ms. These are
vehicle yaw responses from a different operating region, not evidence of a
measured speed-dependent servo motor limit.

Using the corrected wheel speed and 0.36 m wheelbase, each event's local yaw
change corresponds to a median 0.431 rad **effective** turn-angle change.
Its 10–90% yaw-change portion lasted a median 74 ms, giving a median 4.6 rad/s
effective angle rate (event P10–P90: 3.3–7.4 rad/s). This is neither a measured
front-wheel angle nor a mechanical steering-speed limit.

During 2–38 s of standstill, raw gyro z had median bias 0.0134 rad/s,
MAD-based noise scale 0.0032 rad/s and 95th-percentile absolute deviation
0.0075 rad/s. The selected steering steps changed yaw rate by roughly
1.5 rad/s, so gyro noise was small relative to their response. Rear-axle IMU
gyro z samples were identical to raw gyro samples in the low-speed window.
LIO forward velocity at source time was used only to convert yaw rate to an
effective angle, `atan(0.36 × (gyro_z − bias) / vx)`; corrected wheel speed
produced a similar fit.

The current MPCC predicts a first-order steering state with no explicit dead
time. Fitting that **single** time constant at zero understeer and unit static
gain over the low-speed current/speed windows gave 0.078 s using LIO velocity,
or 0.081 s using corrected wheel velocity. The real-car profile now uses
`steering_tau: 0.08 s` instead of 0.115 s. In the separate low-speed speed-mode
window, 0.08 s lowered the RMS effective-angle prediction error from about
0.042 to 0.034 rad. A richer fit (roughly 48 ms pure delay plus 20–27 ms
response constant) describes the sharp test steps better. Thus 0.08 s is a
**lumped command-to-vehicle-response constant for the present MPCC model**,
not an observed delay to first motion, a 90% response time, or a servo-only
time constant. Its fit may change under MPCC's much smoother rate/acceleration
limits. The understeer coefficient remains zero for the initial low-speed
profile; the high-speed full-steer loss of curvature was not folded into tau.

Raw IMU lateral acceleration is unsuitable as an independent servo-angle
sensor. The Livox is about 0.30 m forward of the rear axle, so steering
transients add an angular-acceleration lever-arm term. In these events, raw
lateral acceleration often crossed 90% of its local change around 45 ms,
before yaw rate had even reached 10%. Rear-axle compensation removes the
nominal lever term but relies on differentiated gyro and LIO attitude. The
gyro is the timing observable for this effective steering-response estimate.

MPCC has no identified speed-command delay or motor speed-loop time constant:
its solver treats longitudinal acceleration as directly achieved, while the
runtime sends speed targets to the VESC. Acceleration, braking and jerk limits
do not constitute a measured longitudinal actuator model. Neither these bag
tests nor the command echo establishes physical front-wheel angle. Changes to
`steering_tau` alter the CasADi solver graph, so the native solver cache must be
prepared again for the real reference and vehicle configuration before use.
