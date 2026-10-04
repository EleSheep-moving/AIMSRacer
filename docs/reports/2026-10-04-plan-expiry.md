# Premature MPCC plan expiry, 2026-10-04

**Historical event configuration:** this report's 750 ms lifetime and 10 cm
position limit describe the recorded run, not current defaults. The subsequent
2026-10-04 policy update uses `horizon * 0.8 * dt` (800 ms for horizon 10),
30 cm position, 30 degree yaw, 20 degree steering and 0.30 m/s speed mismatch
limits. The future-forecast expiry fix below still applies. Current parameter
definitions and launch guidance are maintained in the
[runtime timing table](../../src/controller/docs/usage.md#runtime-timing-parameters).
Recorded ages, errors and test results below are preserved as event evidence.

The recorded fault came from evaluating old-plan expiry at a future bridge
timestamp and propagating that simulated fault into the real supervisor. The
old plan was still within its 750 ms source-age lifetime at the real control
tick. The working tree already contained the fix when this investigation began:
only future bridge evaluation disables the source-age upper bound; real output
continues to enforce it. No production timing or mismatch limits were changed
in this investigation.

## Recording and event

Source bag: `/home/aims/mpcc-logs/field-held-tf-20261004/trial-bag/`,
116.822 s, 234,006 messages. Matching controller log:
`/home/aims/mpcc-logs/field-held-tf-20261004/controller-1791046193013006170.jsonl`.
The bag was opened read-only and ROS messages deserialized offline; no topics
were replayed into the vehicle.

The first fault occurs at bag receipt offset **74.249421 s**, local time
**2026-10-04 00:51:36.653664 +08:00**, approximately 6.06 s after RUNNING began.
`/mpcc/status` and the JSONL log both identify the reason as
`Old plan cannot bridge scheduled handover: Plan expired`.

| Quantity | Recorded/reconstructed value |
| --- | --- |
| Last RUNNING plan age | 618.985 ms |
| Last RUNNING plan phase | 399.086 ms |
| Real tick at fault (monotonic) | 3555.257546976 s |
| Old plan original source epoch | 3554.618279801 s, reconstructed from the preceding tick and age |
| Old plan age at fault tick | 639.267 ms, reconstructed; fault clears the plan-age diagnostic |
| Scheduled bridge duration | 200 ms |
| Old plan age at the requested future takeover | 839.267 ms |
| Remaining real lifetime at fault tick | 110.733 ms |
| EKF receive age at fault tick | 4.806 ms |
| Latest solve time | 64.135 ms, `Solve_Succeeded` |

At that tick, `/drive` still published 0.843152 m/s before request preparation
faulted. The next `/drive` message at bag offset 74.267036 s published zero.
`/ackermann_cmd` first records the corresponding zero at 74.269843 s. These are
recorded command timings, not a measurement of when the vehicle physically
stopped.

## Why the old plan remained active

Two successive successful solver candidates were rejected at handover:

| Bag receipt offset | Position mismatch | Position limit |
| --- | --- | --- |
| 74.054826 s | 0.115977 m | 0.10 m |
| 74.249421 s | 0.121490 m | 0.10 m |

The corresponding yaw mismatches were 6.5857 and 6.5669 degrees; modeled
steering mismatches were 1.168 and 0.276 degrees. Command-steering mismatches
were 3.031 and 0.863 degrees. These were inside their historical limits.
Modeled steering comes from forwarded-command history through the same 0.08 s
first-order response used by the solver, without a physical steering sensor;
small steering mismatch does not independently validate the physical model.
The subsequent [LIO/EKF investigation](2026-10-04-lio-delay.md) reproduced
delay-induced EKF yaw distortion and identified a LIO convergence defect,
but did not reproduce the incident's cumulative LIO backlog.

Rejection retained the old plan and its original source epoch. After the second
rejection, the same tick attempted another solve; its future bridge crossed TTL
and caused the premature fault. This was not a solver deadline overrun.

## Fix and practical limit

`MPCCNode.prepare_request()` copies the supervisor and evaluates its future
commands with `forecast.command(stamp, enforce_plan_age=False)`.
`Supervisor.command()` defaults to `enforce_plan_age=True`; the actual output
tick uses that default. Negative age, prediction horizon exhaustion, input
freshness and scheduling checks remain active in forecasts. Forecasts do not
modify the real supervisor, applied command history or original source epoch.

This removes the premature fault, but does not guarantee continuous driving
for this exact event. Only about 111 ms of real old-plan lifetime remained,
while the next planned handover was 200 ms away. Without an intervening valid
activation, real TTL would still stop output before that handover. Avoiding
repeated genuine expiry also requires addressing the handover prediction
mismatch and validating any shorter handover schedule against measured solver
delivery times. Increasing TTL or relaxing mismatch limits was not part of
this investigation.

## Regression verification

Seven local regression cases cover the actual
`prepare_request()` path across TTL, nonzero future controls, isolation of real
execution/history, real expiry after forecasting, rejected-candidate expiry,
and retained horizon/epoch guards. All seven passed; the existing
controller suite passed all 13 tests. The additional regression source and
generated outputs remain local and are not included in this publication.
This verification does
not establish outdoor tracking performance; the existing fix still needs a
new vehicle trial.
