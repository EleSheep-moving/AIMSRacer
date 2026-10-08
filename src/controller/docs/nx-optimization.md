# NX controller optimization: evidence and remaining gates

## Scope and source identity

Work branch: `feat/mpcc-nx-optimization`, isolated worktree `mpcc-nx-optimization`.
The original desktop and NX worktrees were preserved. The live NX baseline is
frozen in `9629618`; implementation starts from upstream `403b5e1` plus that
baseline. Default IPOPT, launch planning frequency and vehicle weights have not
been switched to an experimental profile. Live parameter tuning and MAP/PP are
outside this work.

Execution-policy repairs in this feature branch apply to the shared controller,
including IPOPT. Keeping the default backend does not mean that its execution
behavior is unchanged. The preserved production worktree has not been deployed
with these repairs.

The approved implementation plan is
[`2026-10-09-mpcc-nx-optimization.md`](../../../docs/superpowers/plans/2026-10-09-mpcc-nx-optimization.md).
Raw measurements live outside Git at
`aimsracer-data/experiments/mpcc-nx-optimization/` on desktop and NX. Failed
experiments are retained alongside passing ones.

Latest source result: `e123c27`, 709 desktop tests pass; both complete
replay-backed NX node repeats fail. The historical worker matrix below does
not qualify later QP/path changes. Execution-check cost and activation epoch
consistency remain open; no backend is promoted or field-qualified.

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

Fresh 1 s idle comparison after diagnostics batching and projection caching
(`87a218c`, 100 requests each):

| Backend | Full request P95 (ms) | P99 (ms) | Max (ms) | Rejected | >50 ms |
|---|---:|---:|---:|---:|---:|
| acados, two passes | 14.88 | 17.51 | 50.52 | 2 | 1 |
| QP | 23.23 | 30.18 | 38.57 | 0 | 0 |
| IPOPT | 40.86 | 48.27 | 54.65 | 0 | 1 |

Artifact: `idle-87a218c.json`. This is sequential short testing, includes cold
requests and rejected candidates, and excludes the ROS timer/handover. Faster
acados diagnostics do not resolve its numerical rejection problem.

Artifact for the earlier horizon matrix: `nx-rti2-fastqp-idle-horizons.json`. All failures and cold requests are
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
`acados-rti2-long-matrix/shared-1/`. All three shared repeats were completed: P95 25.117/25.072/25.248 ms,
P99 26.509/26.778/26.976 ms and rejected counts 88/89/87. Each had two >50 ms
requests, maximum consecutive count one. One limited-stress repeat had P95
26.502 ms, P99 29.583 ms, 87 rejections and two >50 ms requests. After the
repeated failure was established, the old-source matrix was stopped at a case
boundary to qualify the repaired source instead. The partially started stress-2
case is retained and ineligible; `controlled-stop.json` records the reason. The
planned nine-repeat acceptance has therefore NOT been completed. The localization monitor also reported tracking,
recovering and lost samples; committed NDT anchors and CPU-load coverage do not
establish continuous localization health or MPCC field acceptance.

## Closed-loop and fault evidence

The original seven frozen failures were repeated at `ffbfbab` using the explicit
optional recovery profile and zero recovery speed references. Recorded initial
state, applied inputs and IPOPT primal seeds are preserved. All seven converge,
but independent candidate checks accept only requests 246/247/248/249 from
235908. Requests 246/247/248 have unavoidable initial utilization excess
0.1041/0.0178/0.0621 under their recorded input/jerk intervals. This experiment
distinguishes convergence from permission to execute; the profile is not default.

| 000919 request | Independent rejection reason |
|---|---|
| 243 | Peak future excess is worse than the matching braking comparator. |
| 244 | Integrated lateral excess is worse than that comparator; initial lateral utilization is already 1.2006. |
| 285 | Integrated operating-envelope excess is worse than that comparator. |

All seven direct calls exceed 50 ms on desktop (P95 183.94 ms), excluding solver
construction and IPC. This is a constraint/feasibility regression, not the NX
worker timing matrix, nor a handover/executed-schedule certificate. The comparator
and rejection rules remain unchanged. Artifact: `final-ffbfbab-seven-requests.json`.

Earlier desktop independent lagged bicycle ROS runs at `f35e9c8` with two RTI passes passed nominal
geometry and speed checks at 1 m/s with 1 s horizon, and 0.5 m/s with 1/1.5/2 s
horizons. Those four runs had no raw or handover rejections. Artifacts begin
`rti2-acados-`. They precede the current nominal executed-schedule gate and
direct acceleration/steering proposals; they do not establish current acados
nominal-lap acceptance. The fresh acados fault runs below test restart behavior.
The 1.5 s run's manifest records an unused QP-backend edit during measurement;
its selected acados sources stayed unchanged. Retain its result with that
limitation rather than treating it as an entirely immutable-source run.
A one-pass failure and its exact replay remain in evidence:
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

The earlier QP ROS harness passed worker stall/crash recovery and operator,
RC loss, odometry drop and clock-reset fault tests. Fresh fault repeats after
the execution-policy repairs are recorded below. Simulation PASS is not a
real-car or held-out model validation.

The final execution-gate ROS run also exposed a startup failure: the speed
tracking smoother's catch-up acceleration reached 0.5 m/s² while the raw final
acceleration was 0.4879345 m/s². Raw terminal utilization was 0.962855; nominal
execution reached 1.008964. This run had no minimum-drive clamp and no initial
wire/internal speed gap. The new physical gate correctly rejected those plans,
but a gate alone does not repair their execution. `b854892` repairs ordinary RUNNING execution to follow the selected
acceleration proposal directly, retaining microstep jerk, angular stopping reserve
and stop/recovery/cap overrides. The startup regression accepts the bounded
executed schedule. Later steering repairs and three synthetic complete-route
passes are recorded below. The original failed
route is preserved at `sim-qp-execution-p1-5886dfd-02`. Its execution-check P95
was 17.2 ms on desktop, so the check also needs a verified performance repair.

The optional `--topic-prefix /mpcc_shadow` isolates all harness driving topics.
Before enable, during execution and at completion it checks zero publishers on
six original driving topics. The harness uses a resolved-topic graph-query
adapter because Humble publisher counts do not apply remapping. The production
controller uses its original authority checks. The earlier desktop shadow lap
passed; its immediate pre-enable audit addition has 27 focused passing
regressions. Fresh final-source fault runs below exercise that audit too.

## Review findings and remaining work

Review of `9eb309d` found that command smoothing changes the inputs validated by
the optimizer rollout. A raw accepted optional recovery with a 0.24 s deadline
produced a model replay of actual command outputs with utilization 1.115235 at
that deadline. This is a software schedule mismatch independent of hardware
motor uncertainty. `5886dfd` adds nominal executed-schedule validation, approved in independent
review with 178 focused passing regressions. It preserves strict microstep jerk,
the original 100 ms comparator prefix and recovery epoch, and physically
projected finish progress distinct from virtual progress. The certificate covers
activation with nominal 20 ms outputs; later authority, expiry or scheduling
overrides are outside that claim. A subsequent complete ROS run exposed the
startup/cost failures above. Later direct-proposal repairs and synthetic QP
route passes are recorded below. NX publication timing and field-model
qualification remain separate. Earlier raw recovery results do
not establish execution safety.

A malformed independent-rollout cache could escape as a loader exception.
`41742b2` converts loader/missing-symbol errors to normal candidate rejection,
preserving an existing plan. Six real isolated-cache regressions first failed,
then passed; the focused suite passed 116 tests. No online repair is attempted.

`48391d9` replaces repeated physical microstep rollouts/projections only when a
finish-independence certificate proves that progress cannot change any nominal
command. The certificate bounds the exact coarse/refinement projector and checks
finish speed cap, stop/deadzone and completion thresholds at every sample.
Uncertain branches, custom projector overrides, and near-finish cases retain the
full scalar physical projection. Map arithmetic preserves the original scalar
matrix product. Independent reviews, 226 focused regressions, 16 independent
seam/status cases and 5,000 exact map-transform comparisons passed. Desktop
100-pair schedule P95 changed 17.725 to 2.011 ms; full physical gate P95 was
3.176 ms with 100 accepted cases. The complete-route measurements below retain
the slower near-finish branch; this paired saving does not prove uniform 50 Hz.

`90a08dd` precomputes configuration-independent ellipse facet trigonometry and
computes the configuration-dependent radius once per QP assembly. 600 frozen
paired assemblies across horizons, corridor settings, map alignment and mutable
margin produced identical P/q/A/l/u arrays. 47 QP regressions passed. N10 desktop
assembly median/P95 changed 2.671/2.724 to 1.803/1.840 ms. On prior `87a218c`,
a 60 s shared-load QP comparison accepted 1,200/1,200 requests; P95 26.180 ms,
P99 30.237 ms, one >50 ms request. It had 599 NDT updates, maximum gap 0.178 s,
normal replay exit and complete initialization/TF audit. This motivated the
assembly repair. At `48391d9`, the same 60 s condition accepted
1,200/1,200 requests: P95 23.541 ms, P99 29.172 ms, maximum 44.070 ms and
zero >50 ms requests. It had 600 NDT updates, maximum gap 0.154 s and complete
normal replay/init/TF audits. These sequential short runs do not replace the
three-repeat matrix. The nine worker-only repeats below used immutable
NX source; steering/harness/operator-stop commits through `ffbfbab` keep all 13
measured worker/native source digests identical. Subsequent QP/path changes
require new measurements and are not qualified by this matrix.
Artifact: `qp-short-shared-87a218c/` on NX and its copied matrix on desktop.

### Completed five-minute QP worker matrix at `48391d9`

All nine planned repeats completed at frozen NX source `48391d9`. All 13
worker/native files are byte-identical at runtime source `ffbfbab`, confirmed
separately in `final-ffbfbab-worker-source-comparison.json`. Runtime/node changes
are validated separately; these timings do not include their polling, handover,
execution check or publication. N10, 20 Hz synthetic moving-state requests use
the saved fixed field reference and experimental native profile. Stress adds
two independent CPU loops at approximately 60% of one core each.
This identity applies only through `ffbfbab`. The later inaccurate-status policy
and QP geometry batching change the measured worker source, so this matrix is
historical evidence and cannot qualify the latest source without fresh tests.

| Condition | Repeat | Requests | Failed | P95 (ms) | P99 (ms) | >50 ms |
|---|---:|---:|---:|---:|---:|---:|
| Idle | 1 | 5,999 | 2 | 17.889 | 24.190 | 1 |
| Idle | 2 | 6,000 | 1 | 18.936 | 25.307 | 1 |
| Idle | 3 | 6,000 | 0 | 18.890 | 23.191 | 1 |
| FAST-LIO2 + EKF + NDT | 1 | 5,998 | 1 | 24.133 | 28.862 | 2 |
| FAST-LIO2 + EKF + NDT | 2 | 5,999 | 1 | 23.905 | 28.834 | 2 |
| FAST-LIO2 + EKF + NDT | 3 | 5,999 | 0 | 23.920 | 28.761 | 1 |
| Estimation + limited stress | 1 | 5,999 | 0 | 25.712 | 30.833 | 1 |
| Estimation + limited stress | 2 | 5,998 | 1 | 25.902 | 31.556 | 2 |
| Estimation + limited stress | 3 | 5,999 | 0 | 27.109 | 31.709 | 1 |

Total: **53,991 requests, six failures and 12 >50 ms requests (0.0222%)**.
Maximum consecutive overruns was one in every repeat. Cold and failed requests
are included. Shared cases meet the worker P95/P99 targets; all three limited
stress P95 values exceed the 25 ms target, so this is not an overall timing PASS.
All six replay-backed windows have valid continuous native-registration coverage,
normal replay exits and complete initializer/TF authority audits. Those load
checks do not prove continuous global-localization health. Each replay's
tracking/recovering/lost observations remain in the matrix.
Artifacts: NX `qp-long-matrix-48391d9/`, desktop
`qp-long-matrix-48391d9-matrix.json`.

The first final-source NX ROS attempt (`ros-shadow-qp-shared-ffbfbab/`) failed
before activation because the external publication observer omitted its Python
multiprocessing main guard. This was a measurement-launcher defect; source and
prepared caches were stable and original driving publishers stayed zero. The
guard was repaired and its spawn-child import verified before a fresh run.
The failed run, natural replay audit and exact observer script are retained.

The earlier desktop controller suite passed **570 tests** at `48391d9` (61.88 s;
63 acados option-deprecation warnings). The isolated NX package built in 4.99 s and the matching suite passed
570 tests in 156.70 s, with the same 63 warnings. Matching QP artifacts were
prepared offline. The subsequent complete desktop ROS run at that source failed:
car started, then candidate/rebase/execution rejection caused bounded recovery
and a stopped re-enable latch. `sim-qp-execution-final-48391d9` preserves the
source-stable run and zero-publisher shadow audit. That source did not qualify;
the next steering repair and subsequent synthetic passes are recorded below.

`2ff2e79` fixes the matching steering target catch-up error. The saved first
turn-reversal rejection at `48391d9` passes raw and rebased checks but fails the
executed gate: macro command maximum 0.2447 rad versus emitted maximum 0.2713
rad, utilization 1.002388. Using ordinary async RUNNING macro slope proposals
through the same micro angular limits gives utilization 0.993646. No target
angle/rate reset or physical gate relaxation occurs. STOPPING, RECOVERING and
legacy branches remain unchanged. Both independent reviews passed, including
36 unchanged-branch comparisons. The full desktop suite passed **577 tests**
with the correct private native-library environment. One earlier test invocation
omitted that environment and had 15 native-loader failures; its log is retained.

Three source-stable 20 Hz, QPN10, 1 m/s synthetic ROS laps at `2ff2e79` passed
with zero rejected plans. Central speed P50 was about 1.000 m/s, minimum body
margins 0.4948/0.4961/0.4952 m, finish errors 0.0354/0.0357/0.0354 m.
Execution-check P95 was 16.56/15.62/15.91 ms because near-finish full projection
remains necessary. Actual publication-gap P95 was 37.35/36.95/37.18 ms, despite
tick-entry-gap P95 near 20 ms. The first run's request-to-activation P95 was
58.17 ms. These are proposal/ROS measurements, not physical actuator latency or
uniform 50 Hz publication proof. Artifacts: `sim-qp-steering-2ff2e79-*` and
`execution-p1-final-desktop-report.json`.

`4a8fd7a` repairs the harness's field asymmetric body geometry: its earlier
half-length assumption crashed with `half_length: null`. The original four-corner
annular metric and acceptance thresholds are preserved; 100 symmetric arithmetic
comparisons and 122 focused tests passed. The failed pre-activation run is retained.

The repaired field-config simulation reaches COMPLETE but remains **FAIL**:
it stops 0.252–0.256 m before the reference end against the unchanged 0.2 m
criterion. Current field minimum speed 0.2 m/s produces a 0.30 m stop-distance
rule and 0.35 m COMPLETE tolerance, exposing a finish-contract mismatch.
Near-finish candidate rejection also exposes the unidentified longitudinal model:
physical speed 0.171066, internal target 0.190394, wire target zero. Integrating
internal target braking from measured physical speed predicts -0.019328 m/s,
which the physical gate correctly refuses under its current model. This is a
surrogate prediction, not measured vehicle reversal. The original field run
also had an isolated earlier rejection that the forensic repeat did not reproduce.
No physical-speed reset/clamp, tolerance change or acceptance relaxation was made.
The field central speed check passes; that does not qualify its finish or model.
Evidence: `execution-field-nearfinish-model-boundary.json`.

`ffbfbab` repairs a separate operator-stop state-machine defect. Once the old
plan expired, STOPPING was mistaken for an initial-planning timeout; the node
also continued submitting new optimization requests during operator braking.
STOPPING now cancels pending handover, ignores late candidates, submits no new
requests and continues the existing bounded braking/steering hold beyond plan
TTL or horizon. Freshness, authority, negative-age and phase guards remain.
Two independent reviews approved 600 paired numerical fallback comparisons,
focused regressions and 15 stop-specific guard cases. The final desktop suite
passed **613 tests** in 61.64 s with 63 acados deprecation warnings.
The isolated NX package then rebuilt at this source and passed the same
**613 tests** in 171.32 s with 63 warnings. Own colcon build/install/log trees
and the project-local environment were used; production source stayed on its
original branch with its 22 local worktree entries preserved.
This preserves the existing bounded fallback; it does not establish a strict
operating-ellipse certificate for operator braking. Fresh complete ROS fault
repeats at this source passed all seven QP cases: operator disable, manual
takeover, RC loss, odometry drop, clock reset, worker stall and worker crash.
The acados two-pass stall and crash cases also passed after injection during
moving RUNNING. Each restarted the worker once; stall stopped at the re-enable
latch and crash resumed RUNNING with three post-restart activations. Disable
submitted exactly 60 solves before stopping and none during STOPPING, then
reached READY at 0.002314 m/s. All nine runs retained exact source, prepared
cache and native dependency-library hashes and zero publishers on the six
unprefixed driving topics. Artifacts: `fault-final-*-ffbfbab/`.

Remaining acceptance includes production ROS
shadow timing on NX under estimation load, backend choice after full load
measurements, resolving the field longitudinal/finish assumptions, and supervised
real laps. Car venue changed; all current NX load
runs use map-matched raw bags. They start no vehicle driving publisher.

## Full ROS NX shadow failures at `ffbfbab`

Worker timings do not establish node acceptance. Two complete shadow repeats
under map-matched FAST-LIO2/EKF/NDT replay failed before 1 m of synthetic
motion. The synthetic plant is separate from the bag vehicle; replay supplies
real estimator CPU load. No unprefixed vehicle driving publisher was present.
The guarded external observer, controller source and prepared caches stayed
unchanged during these runs. Both retain their failed results.

Case 02 first rejects sequence 15 while RUNNING, 79.76 ms after submission;
queue/solve/delivery were 17.84/51.16/8.31 ms. The next tick enters RECOVERING
102.11 ms after the last good update, while the old plan's 0.8 s TTL is still
valid. Its exact first rejected activation context was not captured; the
initiating physical subcheck remains unresolved. Later recovery failures
cannot establish the cause of that first RUNNING rejection.

Case 03 has an exact frozen activation snapshot. Sequence 20 reports OSQP
`solved inaccurate` and is rejected by the native-status policy despite matrix
violation 8.77e-7 and passing raw physical diagnostics. Sequence 21 reports
`maximum iterations reached` and fails separately. With no usable activation
for 118.386 ms, the unchanged two-period guard enters RECOVERING. Sequence 22
passes raw and rebased checks but fails the nominal execution check: full
-0.5 m/s² recovery braking plus lateral load produces maximum ellipse
utilization 1.0078446. Independent replay reproduces the commands exactly and
states within 1.1e-16. The physical gate correctly rejects that schedule.

`f3afc28` now allows native status 2 to supply a candidate only through all
existing finite-primal, matrix and nonlinear bounds. Native status/residuals,
convergence and candidate feasibility are separate diagnostics; status 7 remains
ineligible. This is an engineering candidate policy, not an optimality or motor
safety guarantee. [OSQP status definitions](https://osqp.org/docs/interfaces/status_values.html)
state that inaccurate statuses use ten times the configured native tolerance.
The paired desktop sequence-20 experiment holds its primal fixed and preserves
controls/states exactly; raw and downstream gates accept. Desktop itself
returned status 1, so controlled status 2 tests the policy rather than claiming
the original NX primal/status was reproduced.

`14c23cd` changes asynchronous RECOVERING braking proposals to account for
remaining combined acceleration capacity. It bounds nominal 20 ms lateral
load using measured signed physical speed, the existing jerk/speed-feasible
acceleration endpoints, actual steering and the newly emitted steering target.
Both signed longitudinal halfaxes are checked. Existing jerk/angular/speed
reserves remain; empty or unavailable capacity retains bounded emergency
behavior with explicit diagnostics and no nominal certificate. STOPPING and
legacy synchronous policy stay distinct; straight recovery matches the old
smoother exactly. `dab5ac2` emits this budget in controller diagnostics.
Exact same sequence-22 activation now passes the unchanged gate: no excess,
terminal utilization 0.99953024, hard violation 2.0e-15, margin +0.567507 m.
Measured speed 0.443649906 and internal target 0.532218196 remain separate;
validation preserves the live command prefix and leaves good-count at one.
Thirty new recovery tests and the 144-test focused suite pass. Independent
specification review approved both fixes. Neither a larger scheduling timeout
nor relaxed physical thresholds was used. Current full-node acceptance remains
FAIL until fresh tests pass.
Artifacts: `ros-shadow-qp-shared-ffbfbab-02/`,
`ros-shadow-qp-shared-ffbfbab-03/first-cause-report.json`, and the latter's
`activation-first-rejection.json` / `run-manifest.json`.

## Exact performance follow-ups at `912d3d1`

`aa9412a` batches only physical QP corridor reference positions and tangents.
Unused curvature evaluations are omitted; scalar yaw/trigonometry, alignment
matrix multiplication, rotated-corner dot products and minimum order remain.
Captured original method identity and exact ReferencePath type preserve custom
instance/class/subclass fallbacks. In 100 paired desktop requests per horizon,
all controls, states, native status, envelope and margin matched exactly.
Diagnostic median changes: N10 3.260→1.897 ms, N15 4.578→2.574 ms, N20
5.981→3.290 ms. This measurement is tied to `aa9412a`, before the separate
`c5710a1` finite-native-residual sanity repair; it is not final NX timing.
The latter preserves status/code, marks convergence false and refuses native
eligibility when either residual metadata value is nonfinite. Existing
`finish→json_safe` already normalized nonfinite telemetry; no earlier logging
crash was established. Ninety focused QP/contract tests passed.

`912d3d1` adds an exact theta-only projector for execution checks that consume
only the longitudinal path coordinate. The full projector retains its lateral
error calculation. Coarse first-tie selection, bounded scalar minimizer,
xatol and modulo arithmetic remain identical. Both original function identities
are captured, so instance or class overrides use the full custom method and
cannot receive the standard-method fast certificate. Frozen case03: 408 query
coordinates match bit-for-bit; full RECOVERING schedules match states, controls,
internal/wire targets, elapsed intervals, progress and statuses exactly.
Paired desktop full-schedule P95 is 15.700→13.767 ms; this is a local prototype
comparison, not a promised NX saving. Ninety-three focused owner tests and
ninety frozen-source independent quality checks passed.

`e4736af` records immutable request submission and reply tick identities in node
diagnostics. The external shadow runner uses submission identity when counting
received results, avoiding sequence collisions after worker restarts. It labels
accepted-only proposal timings and separately counts raw failed/unactivated
results. Its run qualification checks synthetic ROS/load/source integrity;
backend timing and real-car acceptance remain separate.

The full tests and NX map-matched replay shadow repeat at this stage used
frozen controller source `912d3d1`. Original production checkout remains untouched.
No candidate backend has been promoted.

## Full-node follow-up: diagnostics and RUNNING overrides

At `912d3d1`, desktop full-suite verification passed **695 tests** in 63.67 s.
The full NX replay-backed shadow reached 12.36 m but failed with a native JSON
TypeError in diagnostic publication. Its input/load coverage was valid for the
entire 19.10 s process window: 189 native registrations, maximum gap 0.290 s,
normal replay drain, unchanged source/cache and zero original driving
publishers. This run remains FAIL. Both external scripts are archived against
their exact manifest hashes. `420e176` reproduces the `numpy.bool_` exception
using finite NumPy-valued live prefixes and emits native diagnostic scalar
types. Frozen before/after recovery schedules and all physical checks match
exactly; 124 focused tests passed.

This run also captured a distinct first RUNNING rejection, sequence 201.
Raw and rebased candidates pass. At nominal t=0.60 s the finish cap first falls
below its interpolated speed target; at 0.62 s output catch-up requests full
-0.5 braking. At 0.96 s achieved -0.46 exceeds available -0.455655 braking
capacity. Peak utilization becomes 1.08398364, terminal 1.07713883. Commands
match the recorded snapshot exactly; cross-host yaw differs by at most
1.1e-16. The single rejection retains plan 200 and RUNNING; the next recorded
tick enters RECOVERING through the unchanged two-period usable-update guard.
This supplies its own complete causal context; it does not prove the older
case02 had the same cause.

`e123c27` extends the nominal capacity proposal to asynchronous RUNNING speed
overrides, applies both signed halfaxes through existing smooth bounds, and
reports generic `braking_budget` with its purpose. Operator STOPPING remains
separate. Any such budget in the scalar fast forecast discards that speculative
trace and reruns the full physical scheduler from the original prefix, because
the budget reads evolving actual steering. Far-from-finish cruise/max-cap tests
show exact fast-versus-forced-slow outputs and unchanged live supervisor state.
Same sequence201 full activation now passes the unchanged gates, terminal
utilization 0.99313180. All 298 focused tests passed and both independent
reviewers approved. No timeout, TTL, physical tolerance or stopped re-enable
rule was relaxed. Fresh full-suite and NX shadow verification use `e123c27`.
Artifacts: `ros-shadow-qp-shared-912d3d1-01/`,
`first-running-finish-exact-replay.json`, `recovery-json-native-differential-result.json`.

### Latest full-node result and timing contract

Frozen `e123c27` passes **709 desktop tests** (63.89 s), but its first complete
NX replay-backed ROS shadow remains **FAIL**. Of 337 received results, 257
report success with accepted validation and 80 report failure (validation
absent). Eight additional activation rejections occur despite their upstream
candidates passing. Only the first, sequence 19, has a complete activation
context in this run. Raw peak
combined utilization is 0.990033, rebased peak is 0.998183, and the emitted
schedule reaches 1.000109403 at nominal t=0.50 s. Speed and acceleration agree;
physical steering differs by 0.004147 rad because of output smoothing. This is
a model/execution mismatch, not a failed native solve. The 1e-4 physical
tolerance remains unchanged.

The final recovery starts after 100.253 ms without an activation. Later
accepted candidates cannot release recovery once its measured-speed predicate
closes. The controller stops 0.293249 m before the goal, outside this synthetic
test's 0.20 m finish criterion. Later rejection causes are not inferred from
the first one; a separate all-rejection capture is necessary.

The manifest's 249 unique accepted activations have
request-to-activation-completion P95 **152.12 ms** and execution-check P95
**47.14 ms**. All 1,137 callbacks have P95 **53.73 ms**; the 1,075 active
RUNNING/RECOVERING/STOPPING callbacks have P95 **54.00 ms**, with 138 exceeding
20 ms. The 27.21 s controller window has valid replay load: 271 native map
updates, maximum gap 0.2813 s, normal replay drain, unchanged controller/cache
hashes, and zero original driving publishers. Valid load does not make the
controller result or timing pass.

The synchronous node captures `now` at callback entry, then validates, publishes
and prepares the next request against that old epoch. At the last good update,
activation completion is approximately 50.82 ms after entry. Consequently,
`worker_queue` includes parent computation before the actual IPC send; it is
not an isolated queue-wait measurement. Retagging only `last_usable_update` or
the command timestamp would mask held-prefix aging or execute a different
schedule from the one checked. Epoch changes require propagation and validation
of the actual held prefix. First investigate exact projection acceleration;
any sufficiency claim requires a fresh complete NX node result. No timeout,
source TTL, physical tolerance or re-enable rule is relaxed.

Artifacts: `ros-shadow-qp-shared-e123c27-01/`,
`first-rejection-exact-replay.json`, `rejection-chronology.json`,
`full-e123c27-desktop.log`.

The same-source all-rejection run (`ros-shadow-qp-shared-e123c27-all-02/`)
also fails. Its one captured rejection, sequence25, passes raw validation and
control reprojection, then fails the rebased envelope gate at terminal
utilization 1.005073. The executed gate is never reached; an older execution
diagnostic must not be attributed to this rejection. Handover lateness is
79.983 ms, raw physical speed 0.503802 m/s versus rebased 0.536897 m/s.
The later fatal event follows an 85.998 ms callback with 74.179 ms execution
validation. Header-derived odometry age increases from 15.792 to 105.154 ms
on the next tick and exceeds the unchanged 100 ms freshness threshold.
Selector age is not recorded. The synthetic plant and controller share a
single-threaded executor, so this proves a harness scheduling limitation;
raw LIO replay is concurrent load and does not supply the shadow odometry.
It does not prove a FastLIO input delay. The 16.96 s controller window still
has valid estimator-load coverage: 168 native updates, maximum gap 0.317 s,
normal replay drain, unchanged source/cache, zero original driving publishers.
Exact replay and causal limits are retained in `forensic-summary.json` and
`forensic-report.md` under that run directory.

## Model and field boundaries

Steering tau 0.08 s came from a previous lumped response fit. Separate servo
transport delay is not measured. The longitudinal prediction is an ideal
acceleration approximation; speed-mode PID delay and speed-dependent response
are not identified. Adding an execution schedule gate does not validate these
hardware assumptions. Identification and held-out prediction errors remain
necessary before claiming higher-speed physical accuracy.

`96ea98b` repairs warm seed angular stopping feasibility. 56 of the original
88 failed-request seeds violated steering acceleration due to endpoint clipping.
All 5,998 repaired seeds satisfy the angular limits, but candidate failures only
change from 88 to 86. The actual input prefix and speed seed policy are unchanged.
198 focused tests passed, including the old exact seed oracle cases.

`7087100` batches CasADi acados constraint/dynamics diagnostics and forward
reconstruction. 51 focused tests passed and 20 frozen requests matched exactly.
Desktop diagnostics median/P95 changed 2.726/2.918 to 1.118/1.330 ms. These are
paired desktop measurements; the updated-source NX short idle table is above. Both source
changes require fresh native artifact preparation.

`87a218c` caches the immutable coarse reference grid and curve points used by
projection. Three new regressions and the 150-test focused suite passed; 300
paired desktop projections matched exactly. Median/P95 changed from
0.235/0.272 to 0.197/0.235 ms. This small saving alone does not repair the
17.2 ms execution-check cost. Native caches were prepared again on NX after
the source fingerprint changed.

Forensic full SQP with exact Hessian and merit globalization produces physically
feasible candidates for all original 88 failures. 79 converge within 35 iterations;
the remaining nine miss the native stationarity tolerance and remain rejected.
This isolates a numerical strategy limitation. No production Hessian,
globalization, tolerance or acceptance change was made from these experiments.

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
