# Independent yaw-rate / angular-acceleration validation (2026-10-15)

**English** · [Full Chinese report](2026-10-15-yaw-rate-quality.zh-CN.md) · [Operation](../operations/yaw-rate.md)

A C++ `[r, dr/dt]` kinematic observer now consumes only startup-bias-corrected gyro
and shared mounting rotation. No wheel/LIO/navigation-EKF/current/steering inputs,
control output, online bias state or replacement of the accepted ax chain.
`r=omega_body.z`, not general Euler yaw derivative. vehicle/race opt-in defaults
false. This is an observation/recording interface.

![Latest MPCC comparison](assets/2026-10-15-yaw-rate/yaw-rate-comparison.png)

| Recording | Corrected gyro HF residual std | New rate HF residual std | Source-waveform best offset |
| --- | ---: | ---: | ---: |
| Latest 022604 MPCC RUNNING | 0.046920 rad/s | 0.028362 rad/s | 4 ms |
| Oct14 B-full, excluding first/last 1 s | 0.006392 rad/s | 0.004124 rad/s | 7 ms |
| Overnight C-resume, excluding first/last 1 s | 0.021035 rad/s | 0.011845 rad/s | 7 ms |

The MPCC static window contains 5,672 common samples: gyro/new-rate std is
0.00239284/0.00117481 rad/s, mean 0.000105527/0.000105679 rad/s. Residual mean is
preserved, not independently identified bias. Other B/C static windows have known
larger residual-bias variation, which this filter does not remove.

HF means SG21/order3 residual on a 200 Hz locally interpolated grid with <=25 ms
gaps, contiguous segments, 10-point edge trimming and common finite support. It
includes real fast motion and vibration. This study does not additionally require
wheel speed >0.5 m/s; its support differs from the preceding yaw-quality review.
Alignment compares new(t) with corrected gyro(t-lag), -50…50 ms in 1 ms steps,
common samples across all shifts, no gain/bias fit. It is waveform alignment,
not physical, communication or compute delay. Displayed curves are never shifted.
EKF appears only as a graphical comparator.

An independent 200 Hz synthetic sine/white-noise sweep selected q=4, R=1e-4 as
the smallest tested q meeting <=10% rate gain error and <=15 ms equivalent lag
at .5/1/2/3 Hz. At 3 Hz, rate gain/lag is 1.0907/2.512 ms; **alpha gain/lag is
0.99257/25.271 ms**. The common stamp does not imply equal response. Rate gain
peaks around 5–10 Hz (1.1866 at 5 Hz); 14/23 Hz gains are 0.8130/0.4983. A seeded
white-noise input std 0.030345 yields output std 0.014919 rad/s. These are
filter-only results; alpha is not substituted into the acceleration observer.

Both x86_64 and NX aarch64 pass **24/24 CTest groups**, including new core cases
and three ROS/launch/recording contracts; offline analysis adds three unit tests.
Identical native input of 42,315 gyro frames produces equal discrete results and
maximum rate/alpha differences 4.44e-16 rad/s / 2.04e-14 rad/s². Each architecture's
20 s 1x receipt-scheduled ROS replay sends 4,000 frames and receives the complete
expected sequence of 3,998 rate and alpha outputs after initialization. Stamps
remain exact int64; graph independence and covariance semantics are checked.

Maximum reported rolling <=512-callback-window P99 is 0.061719 ms x86 and
0.096901 ms NX, including numeric publication. This is not whole-run P99 or
hardware/joint-vehicle-stack timing. Scheduling P99 is 0.184/0.510 ms, with no
hard real-time bound claimed. The first package test lacked the isolated install
environment; the first runtime harness repeatedly decompressed NPZ while sending
and was rejected stale. Correct setup and preload, followed by stricter complete
source-sequence checks, pass without weakening observer gates. Failure logs remain
archived alongside final evidence.

NX install is separate at `/home/aims/aimsracer-yaw-observer-20261015/install`.
Main installation remains unchanged. Covariance is conditional uncalibrated model
uncertainty; no independent dynamic truth, temperature-drift or closed-loop claim.
Full evidence: `/home/elesheep/aimsracer-data/experiments/yaw-rate-20261015/`.
[Quality/hashes](assets/2026-10-15-yaw-rate/quality-summary.json),
[validation summary](assets/2026-10-15-yaw-rate/validation-summary.json),
[x86 CTest](assets/2026-10-15-yaw-rate/desktop-ctest.txt),
[NX CTest](assets/2026-10-15-yaw-rate/nx-ctest.txt),
[native parity](assets/2026-10-15-yaw-rate/native-parity.json).
