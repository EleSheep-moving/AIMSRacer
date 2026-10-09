# MPCC acados C++ runtime: implementation and verification

Date: 2026-10-09. Branch: `feat/mpcc-acados-runtime`. Implementation commit:
`dffe343` (report/documentation commits follow). Base: `8e4f6b2`.

> Follow-up audit: **runtime behavior acceptance is reopened**. The held-output
> execution certificate is missing, and recorded `complete_s` excludes result
> delivery mutex wait. The 3-run numbers below remain archived measurements,
> but do not establish full execution equivalence or the complete delivery
> deadline. See [whole-chain audit](2026-10-09-mpcc-native-contract-audit.md).

## Scope and architecture

Retained the AIMSRacer rear-axle kinematic bicycle model, six physical states
`[x,y,yaw,v,theta,actual_steering]`, three controls
`[acceleration,steering_endpoint,virtual_progress_speed]`, current vehicle
weights/limits/geometry and steering tau=0.08 s. Three auxiliary previous-control
states enforce existing jerk and steering acceleration constraints. Foreign NPU
tire parameters and dynamic model were not imported.

Adopted generated acados SQP-RTI, Gauss-Newton, partial-condensing HPIPM and a
native C++ ROS 2 runtime. NPU reference is `npu-ius-lab/Roboracer_China_2026`,
`real-car-original`, commit `6c5012b9cd310c8fca5281c408298ffb5d4b3885`.
Pinned acados 0.5.5 to `59d93e17d2985fdd73fc58b8a83ed8f83a024171`.

Default delivery: N10, dt=0.1 s, 20 Hz latest-only solving, 50 Hz publication,
50 ms configured request budget (delivery accounting gap noted below), 20 ms forecast lead, 0.8 s TTL measured from
the original odometry stamp. The launch still defaults to `implementation:=legacy`;
new runtime defaults to shadow output. Existing physical worktrees were preserved.

```text
EKF state + NDT map alignment + actual forwarded command history
   -> future state forecast -> frozen per-stage quintic geometry
   -> generated RTI solver -> one nonlinear candidate validation
   -> actual-history takeover check -> reanchored candidate validation
   -> bounded 50 Hz sampler -> RC selector -> VESC conversion
```

The receive and output callbacks have separate callback groups. A dedicated
solver worker has no accumulating request queue. Logger I/O runs asynchronously
with a bounded queue; dropped rows fail benchmark qualification. No online code
generation/compilation occurs. Source/native bundles hash config, reference,
generator closure, ABI/platform and actual loaded acados dependencies.

## Changes that affect behavior

- Preserve exact periodic quintic interpolation; precompute stage geometry outside
  the solver. Shift warm starts using actual elapsed time, including fractional
  stages and failed-update elapsed time.
- One RTI pass, at most one corrective pass within the request budget. Native
  status, finite values and full nonlinear constraints are required. A stale
  solution cannot extend the original measurement TTL.
- Takeover uses actual forwarded old commands. Unexecuted new commands are never
  mistaken for history. Late arrivals produce a separate reanchored candidate,
  whose validation time is included in the publisher callback measurement. If
  unchanged controls fail because the actual applied prefix moved, a small bounded
  endpoint/acceleration transport forms a separate candidate; the complete original
  certificate still applies, without adding RTI passes or extending source TTL.
- Preserve odom/frame/clock checks, map identity, localization protocol-v1
  epoch/sequence/anchor freshness, RC ownership and enable semantics. No TF is
  broadcast by the controller. Manual/stale/localization faults latch zero.
  Decision time is sampled after locking state; receive timestamps remain original.
  This avoids rejecting fresh messages as future-dated while output waits for mutex.
- Preserve bounded acceleration, braking, jerk and steering rate/acceleration.
  Recovery speed targets are zero. Nominal lateral braking budget is applied only
  if feasible; an already excessive lateral load must not suppress bounded braking.
- Keep `enforce_corridor=false` as configured. Experimental soft-envelope ROS
  profiles fail startup until their independent recovery comparator is ported.
- Correct explicit native cost scaling to match the original stage sum at dt=.1:
  stage scaling dt/.1, terminal scaling 1. Otherwise acados default dt scaling
  changes the relative terminal weight.
- Optional N20/N25 at dt=.05 integrates the full duration using 20+20+10 ms RK4;
  default .1 retains five 20 ms steps and legacy Python dt validation is unchanged.

## Verification method

Desktop container: `aimsracer-mpcc-acados-runtime`, image
`aimsracer-mpcc:humble-dev`, new worktree mounted at `/workspace`, evidence at
`/evidence`. No physical driver mounts. Old-controller regression uses its pinned
acados 0.5.3 dependencies; new generated bundles use 0.5.5.

NX: Orin NX 16 GB, R36.5.2, MAXN_SUPER, eight CPUs at 1984 MHz. New source,
ROS overlay, ARM-native acados install and bundles are isolated under
`/home/aims/aimsracer-data/experiments/mpcc-acados-runtime` and the new git worktree.
Power/clock settings were read, not changed. Per-run manifests retain source,
executable/dependency hashes, hardware settings and tegrastats.

Joint-load replay: `2026-10-09/000919-b-1mps-a0p6-j2/bag`, map
`/home/aims/maps/20260928_010503/map.pcd`, recorded initialization seed.
FAST-LIO2 + EKF + NDT run genuinely, with accepted anchors and native registration
throughout the same measurement window. Replay supplies CPU load. Controller
feedback is a separate 2 ms midpoint bicycle plant with speed tau=.2 and steering
tau=.15, through the actual RC selector and VESC converter on isolated topics.
This tests timing and synthetic tracking; it does not test actual vehicle dynamics
or prove field localization/tracking.

Every submission, worker outcome, activation/rejection and publication including
zero is accounted for. Recorded computation timing includes failed outcomes. The
original harness applies the timing thresholds below to its pre-delivery metric,
which does not qualify the complete-delivery target. Its checks require complete
log accounting, >=99% request/publication coverage, continuous estimation load,
no source changes, P95 request<=40 ms, P99<=50 ms, deadline/failure fraction<=0.1%,
no three consecutive failures, publication P99<=30 ms and max<=60 ms.

## Test results

- Legacy controller regression: **809 passed**, 63 existing warnings, 69.47 s.
- New runtime Python checks: **9 passed** (5 fractional interval + 4
  startup/accounting), including fractional interval and
  startup/accounting checks.
- Release contract tests explicitly retain assertions with `-UNDEBUG`; otherwise
  the compiler would remove health/history/output checks. This test-only fix does
  not change the runtime binary.
- C++ CTest: **7/7** on desktop and NX, including model/cost/constraints/path,
  independent numerical loop, reanchor, output, health/history and bundle rejection.
- Final rerun: **10 passed in 29.24 s** for fractional interval + five ROS
  scenarios; startup/accounting separately **4 passed in 0.52 s**.
- ROS protocol probe: **5 scenarios, 136 checks passed** with the synthetic map
  fixture. Faults are injected after observing positive commands and RUNNING,
  then require FAULT and sustained zero within the stated limits. The additional
  valid-input stress repeats original-stamp odometry at ~1 kHz and identical
  authority at ~8 kHz; it requires continuous RUNNING/positive output. Six old-binary
  stress runs did not reproduce the race and are not claimed as RED evidence.
- The deterministic callback-clock test uses the actual C++ callback and a test-only
  entry marker excluded from production. It forces a newer receive/sampler reset
  while output waits for mutex: entry-epoch code fails (negative age 0.47 us),
  after-lock decision time passes.
- Both recorded-map production bundles load on NX with `simulation:=false`,
  `shadow:=true`, worker ready, zero output and no public drive publishers. This
  proves loading and contracts only; the controller was not enabled on hardware.
- Isolated dt evaluation: N10/.1, N20/.05, N25/.05 model tests/independent numerical
  laps passed, with assertions and UBSAN checks for N20/N25.

### Final NX joint-load measurements

All three use the identical saved 33.47 m route geometry, 1.0 m/s target,
strict profile and independent synthetic feedback under genuine estimation load.
Times below are milliseconds; lateral errors are synthetic, not field measurements.
Here the original harness result means its recorded timing/log/load checks
plus the harness absolute synthetic tracking check. Recorded timing excludes
result delivery mutex wait, and executed-envelope equivalence was not checked. The plan's relative tracking gate against a successful
same-condition legacy run remains **unestablished** on this route.

| Run (180 s) | Requests | Request P95 | P99 | Max | Contour P95 (cm) | Laps | Worker fail / takeover reject / late | Original harness result |
|---|---:|---:|---:|---:|---:|---:|---|---|
| 1 | 3594 | 1.023 | 1.188 | 1.897 | 1.64 | 5.275 | 0 / 0 / 0 | PASS |
| 2 | 3595 | 1.022 | 1.171 | 3.607 | 1.59 | 5.292 | 0 / 0 / 0 | PASS |
| 3 | 3595 | 1.043 | 1.169 | 1.446 | 1.55 | 5.292 | 0 / 0 / 0 | PASS |

| Run | Native P95 | Preparation P95 | Validation P95 | Publisher callback P95 | Publish gap P99 | Gap max | Request to first publication P95 |
|---|---:|---:|---:|---:|---:|---:|---:|
| 1 | 0.722 | 0.171 | 0.059 | 0.488 | 20.283 | 28.257 | 39.402 |
| 2 | 0.718 | 0.171 | 0.058 | 0.485 | 20.289 | 26.985 | 39.387 |
| 3 | 0.726 | 0.172 | 0.060 | 0.490 | 20.510 | 29.312 | 39.287 |

Preparation includes seed propagation, warm-start transport, frozen stage
parameters and initial solver setup; it is not a pure geometry timing.

The recorded `complete_s` covers submission through the worker's pre-delivery
timestamp, including solve and candidate checks; it excludes reacquiring the
state mutex and installing the pending result. It must be called reported
worker computation time, not complete request delivery latency.
Request-to-first-publication P95 is about **39 ms**, including the forecast
lead/publication scheduling and delivery wait for plans actually published.
Every run has zero dropped log rows and zero public actuator publishers.
Run 2 has one valid result left unactivated at test shutdown; this is not a rejection.

Run 1: 1802 native NDT updates in the 180 s measurement, maximum native gap 173.6 ms.

Run 2: 1802 native NDT updates in the 180 s measurement, maximum native gap 163.1 ms.

Run 3: 1800 native NDT updates in the 180 s measurement, maximum native gap 192.3 ms.

Evidence: `nx-results/delivery-recorded-{1,2,3}-manifest.json` and remote
`delivery-recorded-20hz-{1,2,3}` directories. Runtime executable SHA256:
`b7cfa98a12ab18f0de6260ce09600a4dcf589b0be0f0c3721e38bfb944394c56`.
Bundle source manifest SHA256:
`c89a3d6f820932bf803a67e5ea69a23a744062227bde992a064a8fd8d5d9b85a`.

Earlier tests at `a6c012d` passed two R2-circle 180 s runs, but the third
stopped at 47.8 s with an input-freshness FAULT (955 requests, no late requests or
handover rejection). The source audit exposed the pre-lock decision-clock race,
subsequently reproduced deterministically. Those tests are not final acceptance.

Preliminary tests at `5092857` also passed three 180 s runs with request P95
0.812/0.814/0.823 ms, but preceded the braking fix. They are retained as preliminary
evidence and are not substituted for final-version measurements above.

### Legacy comparison and tracking

NX comparison uses the same saved route, vehicle profile, synthetic plant, genuine
estimation load, prebuilt IPOPT/JIT cache and input/output contracts.

| Runtime | Frequency / budget | Observed measurement | Requests / accepted activations | Request failures (including acceptance) / late | Request P95 / max (ms) | Result |
|---|---|---:|---|---|---|---|
| IPOPT legacy | 20 Hz / 50 ms | 1.000 s | 10 / 0 | 10 / 10 | 71.299 / 79.972 | Stopped: all requests late, recovery requires reenable |
| IPOPT legacy | 5 Hz / 250 ms | 1.001 s | 5 / 1 | 4 / 0 | 76.043 / 80.027 | Stopped: executed schedule fails physical/recovery bounds |
| Native acados | 20 Hz / 50 ms | 3 x 180 s | 10784 / 10783 | 0 / 0 | 1.022-1.043 / 3.607 | Original harness PASS, all runs complete |

Legacy was scheduled for 30/60 s respectively but the acceptance harness stops
when nominal RUNNING is lost. Every request is included, including all failures;
these short failed attempts are not full-duration latency qualifications.
At 5 Hz the direct-core P95 was 56.059 ms, publisher callback P95 7.589 ms,
publish gap P99 42.009 ms; the only accepted activation reached first publication
at 101.985 ms. All 20 Hz replies exceeded 50 ms. Both have valid overlapping
load coverage and no cleanup error. The 20 Hz replay completion audit is clean.
The 5 Hz attempt also has a separate replay-drain failure: controller exit after
1 s leaves too much of the 120 s replay for the fixed 90 s drain, so its replay
is stopped without a final replay summary (exit None/drain timeout in manifest).
That incomplete replay completion/TF audit is retained as a failed audit; it does
not erase the earlier controller failure or establish a successful comparison.
Compilation/JIT preparation was completed before
measurement (75.56 s one-time preparation, not online solve time).

There is therefore no qualified matched NX legacy tracking comparator and no
claim that native field precision is better or a formal matched tracking ratio
was proved. Evidence: remote `delivery-legacy-recorded-{20hz,5hz}` directories.


Desktop legacy at its old 5 Hz / 250 ms settings tracks the 0.5 m/s synthetic route;
its request P95 was 20.01 ms, direct-core P95 15.82 ms, publisher callback P95
9.11 ms, contour P95 1.82 mm. Native 20 Hz at the same profile had contour P95
3.00 mm and heading P95 .00591 rad, within old*1.1+1 cm and old*1.1+0.5 degree.
These cadence settings differ. At matched 20 Hz/50 ms, legacy desktop runs enter
RECOVERING after repeated independent candidate/activation validation failures;
no successful complete matched run exists. Both native optimizer outcomes and
supervisor rejections are retained; successful subsets are not presented as a
qualified comparison.

### Optional 40 Hz and longer horizon

First-publication distributions below include only plans actually published
(724 for the lead20 ms case), not all submitted requests. Pending/rejected
results remain in the separate request/disposition counts; reported pre-delivery computation
timing includes all returned outcomes.

Each is **one 60 s exploration**, 40 Hz requests / 25 ms budget, unchanged
50 Hz publisher, genuine estimation load throughout. These are not three-run
180 s qualification. Times are milliseconds. R2 circle rows are not comparable
to saved-route tracking precision. All have zero native worker failures and zero
late replies; log/load/source checks pass.

| Profile | Requests / accepted activations | Request P95 / P99 | Takeover rejects | Maximum failure streak | First-publication P95 | Exploratory gates |
|---|---|---|---:|---:|---:|---|
| Saved route, N10/.1, lead20ms | 2392 / 724 | 1.103 / 1.229 | 1 | 1 | 26.096 | PASS |
| Saved route, N10/.1, lead2ms | 2393 / 2393 | 1.137 / 1.297 | 0 | 0 | 21.463 | PASS |
| R2 circle, N20/.05, lead2ms | 2393 / 2364 | 1.334 / 1.717 | 29 | 29 | 21.297 | FAIL |
| R2 circle, N25/.05, lead2ms | 2393 / 2392 | 1.992 / 2.147 | 0 | 0 | 21.415 | PASS |

N10/20 ms lead only accepted 724 of 2392 results (12.07 accepted plans/s),
despite ~40 requests/s; newer pending future plans superseded older ones.
N10/2 ms lead accepted all 2393 results (39.88/s). N25 has one result not yet
activated at shutdown; that is not a rejection.

N20 fails the candidate-disposition criteria: 29 takeover rejections (1.212%)
and a 29-failure streak, despite fast solving and continued synthetic tracking.
The runtime reasons for all 29 are `nonlinear candidate rejected; prefix transport
ineligible`; the transport stayed within its eligibility contract. Exact rejected
constraint rows were not instrumented in this exploration, so a narrower physical
cause is not claimed. This is an activation certificate failure, not a
request-timeout problem. It is
not hidden by the harness absolute tracking PASS. Longer horizons are not
automatically more reliable; N25 happened to pass this short circle test.

Evidence: remote `delivery-recorded-40hz-lead{20,2}`,
`delivery-n{20,25}-40hz-lead2`, and matching downloaded manifest JSON files.


The default remains 20 Hz/N10/.1. A 40 Hz request count is not automatically a
40 Hz accepted-plan rate: the 20 ms forecast lead and 20 ms publisher cadence can
supersede a pending future candidate before activation. The 2 ms comparison lead
is an explicitly selected benchmark candidate, not a delivery default. Reports retain request and
activation counts separately. Native sub-millisecond solve times do not describe
waiting for a future activation/publication slot.

## Recorded-route takeover diagnosis

The saved 33.47 m route geometry was copied with identical `path.csv` into a
synthetic odom reference. Only its frame label/test metadata changed; physical
map reference and localization health contracts remain separately tested.

The first 80 s run tracked 2.31 laps but had 61/1598 takeover rejections and a
12-failure streak. An instrumented repetition stopped at 70.2 s after plan expiry.
Exact replay of 1,401 activation snapshots reproduced all 51 rejections and their
violation values: 47 slew, 5 jerk and 1 envelope row (groups overlap). Changing the
past applied endpoint changes first-stage rate and therefore second-stage slew;
this was a correct rejection, not a false numerical threshold.

The revised conditional transport restored 50 frozen cases; 1,350 original valid
cases kept identical controls; the true ellipse case remained rejected. On the
new 80 s ROS route run: 1598 requests, zero worker failures/late replies, one true
ellipse takeover rejection (0.0626%, maximum streak1),29 transports,2.31laps,
contour P95 **1.55 cm**, maximum **5.64 cm**, heading P95 **0.0509 rad**.
These satisfy the stated failure bound for this test; hardware replay verification
is listed above. The source state, TTL and optimizer pass count are never renewed
by transport. Large prefix jumps and failed alternate certificates remain rejected.

## Historical failures remain a separate issue

Seven captured field requests were cold-started independently against the recorded
map reference and the two matching acceleration/jerk profiles. Four have minimum
initial strict-envelope utilization >1 (1.104, 1.018, 1.062, 1.201); their initial
state already violates the configured strict model constraint. Of the remaining three,
one is accepted and two are not certified after the allowed two RTI passes.
Native status 0 alone does not authorize nonlinear-infeasible candidates.

The new solver computes these requests in well under the 50 ms budget, but it does
not make all seven executable. This tool does not reconstruct the old warm-start
history. Changing envelope/recovery policy or physical model is separate work;
no limit was weakened to hide a failure. Reports are JSON files separate from
native error stdout. Empty feasibility intervals/nonfinite metrics are null.

## Artifacts, launch and rollback

The new runtime should remain in shadow while the follow-up audit blockers are
resolved and revalidated. Loading a production bundle is not field release.

Read [package instructions](../../src/aims_mpcc_rt/README.md) for offline export,
target compilation, selected runtime and rollback. Important NX locations:

- Source: `/home/aims/.config/superpowers/worktrees/AIMSRacer/mpcc-acados-runtime`
- Overlay: `/home/aims/aimsracer-data/experiments/mpcc-acados-runtime/ros-install`
- Pinned target acados: `.../mpcc-acados-runtime/deps/acados/install-runtime`
- Recorded-map bundle: `.../mpcc-acados-runtime/bundles/final-field-n10`
- Final recorded-route synthetic timing bundle: `final-recorded-route-odom`.
- Other synthetic timing bundles: `final-r2-v10`, `final-r2-v05`; optional
  `final-r2-n20-dt05`, `final-r2-n25-dt05`.

The production reference was copied read-only from
`recordings/20260928-mapping-lap/prepared_map_1m_rear10cm`, map SHA256
`1db8c1905dc99ed4c0897838421118b998d15eb7cf57f50cee96a8b0744870dc`.
Physical `recordings/current` and dirty vehicle worktrees were not changed.

Example review/shadow launch, after sourcing vehicle/localization and the new overlay:

```bash
ros2 launch aims_mpcc mpcc.launch.py implementation:=acados_cpp shadow:=true \
  cpp_solve_frequency:=20.0 \
  artifact_directory:=/home/aims/aimsracer-data/experiments/mpcc-acados-runtime/bundles/final-field-n10 \
  vehicle_config:=/home/aims/aimsracer-data/experiments/mpcc-acados-runtime/bundles/final-field-n10/input_config.yaml \
  path_directory:=/home/aims/aimsracer-data/experiments/mpcc-acados-runtime/bundles/final-field-n10/input_reference \
  log_directory:=/home/aims/aimsracer-data/experiments/mpcc-acados-runtime/operator-shadow-new
```

This does not drive the vehicle. Physical closed-loop acceptance is still pending.
Rollback stops/disables the new runtime and selects `implementation:=legacy` with
the existing legacy arguments. Do not start a second public `/drive` owner.

The complete final NX run logs (including failed comparisons and exploratory
cases), target bundle metadata, field loading checks and telemetry were copied to
`nx-results/delivery-nx-evidence.tgz` (25,968,952 bytes), SHA256
`6dd5dd6590491192f22609f5b3cbd7a9f252f92ce4243e2888d7d3e93573daf2`.
The archive excludes original sensor bags; their paths remain in every manifest.

Full evidence root (desktop and downloaded NX reports):
`/home/elesheep/aimsracer-data/experiments/mpcc-acados-runtime`.
Remote run directories include controller CSV, measurement summary, replay events,
TF/initializer audits, native bundle manifests, source hashes and hardware telemetry.

Whole-attempt tegrastats (includes startup and replay drain, not just controller
measurement) sampled at 1 Hz: mean utilization across eight CPU cores was
40.50%, 39.70%, 40.24%; peak sampled mean was 71.63%, 78.88%, 71.13%.
Maximum RAM was 3115/3138/3120 MB; CPU temperature peaked at
62.75/62.94/63.19 C. These are total-system statistics, not controller-only CPU.
No on-target build, artifact preparation or archive compression ran during the
measurement windows.
