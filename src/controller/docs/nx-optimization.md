# NX controller optimization: evidence and remaining gates

## Scope and source identity

Work branch: `feat/mpcc-nx-optimization`, isolated worktree `mpcc-nx-optimization`.
The original desktop and NX worktrees were preserved. The live NX baseline is
frozen in `9629618`; implementation starts from upstream `403b5e1` plus that
baseline. Default IPOPT, launch planning frequency and vehicle weights have not
been switched to an experimental profile. Live parameter tuning and MAP/PP are
outside this work.

The approved implementation plan is
[`2026-10-09-mpcc-nx-optimization.md`](../../../docs/superpowers/plans/2026-10-09-mpcc-nx-optimization.md).
Raw measurements live outside Git at
`aimsracer-data/experiments/mpcc-nx-optimization/` on desktop and NX. Failed
experiments are retained alongside passing ones.

## Implemented control and solver contracts

- One optimizer request in flight; no queued stale requests. Plan lifetime is
  anchored to its original source timestamp. Handover time and planning period
  are separate. Actual state and selector-forwarded input prefix are checked.
- A single failed solve can retain an unexpired plan. Two planning periods with
  no usable update enter bounded recovery. Two usable candidates can resume
  while moving; a stopped recovery requires manual re-enable. Worker stall and
  crash have a bounded offline-cache restart policy.
- Actuator limits, operating envelope and optional finite recovery slack have
  distinct checks. An independently integrated braking comparator shares the
  first 100 ms input prefix and steering sequence. Physical caps and deadlines
  remain explicit. Optional recovery is not enabled by default.
- acados generated C SQP_RTI/HPIPM and native OSQP are selectable candidates.
  IPOPT remains available. Artifacts are prepared offline for the host ABI.
  Missing or modified acados artifacts fail without online compilation.
- `acados_rti_steps: 2` is an experimental fixed two-pass option, with both pass
  statuses/times/residuals logged. Status zero still requires independent
  physical validation. The QP is a reduced lateral/speed controller with a
  nonlinear output rollout; it does not optimize the full MPCC progress state.
- The wire speed target continues from the actual forwarded target at handover;
  physical speed remains the measured state. They are never silently equated.
- Native independent rollout and batched geometry reduce Python overhead.
  QP cost/CSC assembly changes were compared on identical frozen inputs.

## Measurements: separate timing boundaries

The AsyncSolver benchmark includes request preparation, worker preparation,
optimization, numerical diagnostics, worker independent validation, IPC and
caller independent validation. It **does not** include the ROS 50 Hz polling
timer or actual Supervisor handover. The production-node shadow harness measures
those separately. A small optimizer duration does not equal command activation
latency.

NX: Orin NX, JetPack R36.5.2, existing MAXN_SUPER mode unchanged; eight CPUs.
Native optimizer threads are bounded locally; estimator threading is retained.
Source `9eb309d`, experimental native profile with common reserve 0.01 and two
RTI passes. Each idle cell below contains 100 synthetic moving-state requests;
this is a short comparison, not the three-repeat acceptance gate.

| Backend | Horizon (s) | Full request P95 (ms) | P99 (ms) | Max (ms) | Rejected | >50 ms |
|---|---:|---:|---:|---:|---:|---:|
| acados, two passes | 1.0 | 20.01 | 22.62 | 54.71 | 2 | 1 |
| acados, two passes | 1.5 | 25.50 | 28.28 | 62.42 | 0 | 1 |
| acados, two passes | 2.0 | 32.13 | 33.87 | 67.87 | 0 | 1 |
| QP | 1.0 | 23.16 | 30.11 | 39.57 | 0 | 0 |
| QP | 1.5 | 34.18 | 44.58 | 52.21 | 0 | 1 |
| QP | 2.0 | 35.74 | 57.77 | 60.25 | 0 | 3 |
| IPOPT | 1.0 | 40.72 | 48.25 | 54.71 | 0 | 1 |
| IPOPT | 1.5 | 51.74 | 68.70 | 68.92 | 0 | 8 |
| IPOPT | 2.0 | 64.25 | 85.52 | 85.53 | 0 | 34 |

Artifact: `nx-rti2-fastqp-idle-horizons.json`. All failures and cold requests are
included. Worker CPU cores for acados/QP/IPOPT at 1 s were 0.337/0.360/0.557.
These separate runs do not prove a paired causal speedup on NX. Paired desktop
QP assembly comparisons accepted the same 500/500 frozen requests and reduced
full-call P95 from 7.666 to 6.228 ms.

First five-minute FAST-LIO2 + EKF + NDT shared-load repeat at `9eb309d`:
5,998 requests, full-request P95 25.117 ms, P99 26.509 ms, maximum 92.265 ms;
two requests exceeded 50 ms, with maximum consecutive count one. 88 numerical
candidates were rejected. This is **not overall acceptance**: the P95 target is
25 ms and the rejected candidates fail the physical envelope checks. There were 3,000 NDT
updates during the window, maximum update gap 0.166 s, and 3,519 committed
anchors over the complete bounded replay. The replay completed normally and
had one authority each for map/odom and odom/base_link. Artifact:
`acados-rti2-long-matrix/shared-1/`. Further repeats are in progress. The localization monitor also reported tracking,
recovering and lost samples; committed NDT anchors and CPU-load coverage do not
establish continuous localization health or MPCC field acceptance.

## Closed-loop and fault evidence

Desktop independent lagged bicycle ROS runs with two RTI passes passed nominal
geometry and speed checks at 1 m/s with 1 s horizon, and 0.5 m/s with 1/1.5/2 s
horizons. Those four runs had no raw or handover rejections. Artifacts begin
`rti2-acados-`. A one-pass failure and its exact replay remain in evidence:
268/292 fixed requests were accepted with one pass; 292/292 with two passes.
The newest long NX benchmark still has failures, so that small replay cannot be
used as a universal convergence guarantee. Exact desktop replay reproduced all
5,998 two-pass results and all 88 rejected requests. All 88 violate the future
operating envelope; 75 also violate its terminal bound. Native status is zero
in both passes, and the worst independent utilization is 2.320435. Rejections
repeat at the same bend before the map lap seam; spline yaw and curvature are
continuous at the seam. The violation already exists in the native iterate,
so forward reconstruction did not create it. Four passes resolve only 36/88;
some failures worsen. Constant acceleration zero with unchanged steering is
independently feasible for all 88, and cold QP/IPOPT comparisons accept all 88
with unchanged configuration and references. IPOPT needs 15–24 iterations.
The benchmark prescribes zero applied acceleration and steering rate on each
synthetic moving-state request; this is not recorded physical command history.
Nevertheless, each frozen problem has feasible alternatives. Native convergence
needs further investigation; raising the pass count is not a demonstrated fix.
Artifacts under `acados-rti2-long-matrix/shared-1/` include
`desktop-four-step-forensics.json`, `desktop-cold-qp-ipopt-on-88.json`, and
`desktop-feasible-seed-same-frozen-ocp.json`.

The QP ROS harness passed the worker stall/crash recovery tests and operator,
RC loss, odometry drop and clock-reset fault tests. The two-pass acados restart
cases remain a separate pending verification. Simulation PASS is not a real-car
or held-out model validation.

The optional `--topic-prefix /mpcc_shadow` isolates all harness driving topics.
Before enable, during execution and at completion it checks zero publishers on
six original driving topics. The harness uses a resolved-topic graph-query
adapter because Humble publisher counts do not apply remapping. The production
controller uses its original authority checks. The desktop shadow lap passed;
its final pre-enable audit addition has 27 focused passing regressions and needs
fresh full-route verification on the final source.

## Review findings and remaining work

Review of `9eb309d` found that command smoothing changes the inputs validated by
the optimizer rollout. A raw accepted optional recovery with a 0.24 s deadline
produced a model replay of actual command outputs with utilization 1.115235 at
that deadline. This is a software schedule mismatch independent of hardware
motor uncertainty. A shared execution schedule and additional independent
execution check are being implemented; earlier raw recovery results do not
establish execution safety.

A malformed independent-rollout cache could escape as a loader exception.
`41742b2` converts loader/missing-symbol errors to normal candidate rejection,
preserving an existing plan. Six real isolated-cache regressions first failed,
then passed; the focused suite passed 116 tests. No online repair is attempted.

Remaining acceptance includes final execution-schedule regressions, production
ROS shadow timing on NX under estimation load, backend choice after full load
measurements, and supervised real laps. Car venue changed; all current NX load
runs use map-matched raw bags. They start no vehicle driving publisher.

## Model and field boundaries

Steering tau 0.08 s came from a previous lumped response fit. Separate servo
transport delay is not measured. The longitudinal prediction is an ideal
acceleration approximation; speed-mode PID delay and speed-dependent response
are not identified. Adding an execution schedule gate does not validate these
hardware assumptions. Identification and held-out prediction errors remain
necessary before claiming higher-speed physical accuracy.

Weights were held fixed during backend comparisons. No new weight recommendation
or final higher-speed profile has been selected. Real acceptance remains at least
three laps each at 0.5 and 1 m/s, followed by controlled higher-speed envelopes.

## Deployment and rollback

Keep the feature branch isolated until remaining gates pass. Build and cache
preparation, launchability, simulation, replay and real-car acceptance are separate
claims. `experimental_native_rti2.yaml` is a comparison profile, not the default
vehicle configuration. Changing backend/configuration/reference requires offline
preparation for the matching native artifact fingerprint.

Rollback is to the preserved original NX worktree and its launch procedure. The
optimization tests have not modified it or deployed a default backend change.
Never run the ordinary integration harness on a connected vehicle without the
shadow topic prefix: its default topic route intentionally matches the original
hardware-free ROS integration test.
