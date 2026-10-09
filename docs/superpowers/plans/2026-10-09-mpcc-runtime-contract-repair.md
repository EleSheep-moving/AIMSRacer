# MPCC output and runtime contract repair implementation plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking. Preserve evidence and commit each independently verified repair.

**Goal:** Remove the additional hard jerk and steering-command acceleration limits, make predicted and emitted commands consistent, and repair the four blocking native-runtime findings before renewed NX qualification.

**Architecture:** Retain the six physical states, kinematic bicycle, steering response, acados backend, fixed map reference and authoritative forwarded-command history. Introduce an explicit versioned command profile shared by bundle generation, validation and output; preserve a bounded pending-plan lifecycle and measure delivery after acquiring result ownership. Bind localization snapshots to accepted anchor identity and repair clock/history reset as one lifecycle operation.

**Tech Stack:** ROS 2 Humble, C++17, Python, CasADi/acados 0.5.5, existing private RC/converter numerical loop, desktop container and isolated Orin NX worktree.

**Status:** Planning only. The user selected removal of the two additional higher-order limits and requested this plan. No production source, configuration, generated solver or installed binary is changed by this document.

**Baseline:** `6265192af4b45d0a6bb819da9334ee34a8e8128f`, branch `feat/mpcc-acados-runtime`, worktree `/home/elesheep/.config/superpowers/worktrees/AIMSRacer/mpcc-acados-runtime`. The audited runtime source is unchanged since the previous validation snapshot. See [whole-chain audit](../../reports/2026-10-09-mpcc-native-contract-audit.md). This plan supersedes the earlier plan's requirement to preserve all higher-order constraints; its baseline measurements remain archived.

## Design decisions and alternatives

1. Select a new `rate_bounded_v2` command profile. Remove hard jerk and steering-command acceleration constraints from both the native OCP and execution path. Changing only output would leave unnecessary optimizer restrictions and two different contracts. Keep `legacy_bounded_v1` as the default meaning of old configurations/artifacts, for reproducible rollback and controlled comparison.
2. Retain speed, steering-angle, acceleration/deceleration, steering-rate and combined operating-envelope limits. Keep the original objective weights during the first comparison, including the existing soft steering-rate-change penalty. Add a separate `steering_acceleration_scale` for v2 cost normalization, initialized to the old normalization value so the initial cost is unchanged; it is not a physical acceleration limit. Record this retained penalty so a remaining slow response is not incorrectly blamed only on output.
3. Retain the nominal 20 ms output period. Implement acceleration-to-speed integration and the model's staged steering ramp without the second higher-order slew operation. Removing all output reconstruction would change the current endpoint-control model and is outside the selected design.
4. Keep the rear-axle model, physical state count, tau and existing reference geometry. Auxiliary previous-control states may remain for objective/history bookkeeping. Do not import NPU tire parameters or change weights, speed targets or model family during the output comparison.
5. Prefer one owned pending candidate: while it awaits activation, do not submit another solve. Continue a still-valid active plan. This deliberately permits fewer solves when forecast lead exceeds the request period; it prevents spending computation on candidates that repeatedly displace the next executable result. A multi-slot plan queue is unnecessary for this repair.
6. Preserve the explicitly documented hold-until-source-TTL recovery policy for isolated failed solves. Keep TTL at 0.8 s for the N10/.1 baseline; faults in input freshness, localization or authority still act independently. Do not restore the old two-solve-period recovery rule silently.
7. Frame all rollout certificates as checks of the existing ideal-acceleration/steering-lag model. The VESC path receives speed setpoints. Removing command jerk does not identify motor acceleration, braking or tire behavior.

## File responsibilities

| Area | Files | Responsibility |
|---|---|---|
| Profile and generated problem | `src/controller/aims_mpcc/config.py`, `acados_backend.py`; `src/aims_mpcc_rt/scripts/export_bundle.py` | Profile validation, hard-constraint groups, cost semantics, artifact fingerprints |
| Native candidate and output | `src/aims_mpcc_rt/include/aims_mpcc_rt/core.hpp`, `output.hpp`; `src/aims_mpcc_rt/src/core.cpp` | Seed/reanchor rules, candidate checks, emitted command reconstruction |
| Executed-trace certificate | New `src/aims_mpcc_rt/include/aims_mpcc_rt/execution.hpp`, `src/aims_mpcc_rt/src/execution.cpp` | Prospective output simulation and independent physical/corridor checks |
| Runtime lifecycle | `src/aims_mpcc_rt/src/node.cpp`, `include/aims_mpcc_rt/history.hpp`, `health.hpp` | Delivery deadline, pending ownership, epoch reset, history coverage, diagnostics |
| Anchor provenance | `src/aims_racer_system/scripts/localization_monitor.py` | Carry the accepted transform and its epoch/sequence atomically with health |
| Entry points | `src/controller/launch/mpcc.launch.py`, `src/controller/aims_mpcc/node.py` | Resolve actual configuration, reject unsupported profile/backend combinations |
| Evidence | `src/aims_mpcc_rt/tests/`, `tools/acceptance.py`, `tools/protocol_probe.py`, `tools/nx_joint_load.py` | Non-vacuous regression tests, independent plant, corrected timing accounting |

## Task 1: Freeze the baseline and strengthen the test oracle

**Files:** `src/aims_mpcc_rt/tests/test_core.cpp`, `tests/test_runtime_clock.cpp`, `tests/test_benchmark_accounting.py`, `tools/audit_contracts.cpp`, `tools/audit_contracts.py`, `CMakeLists.txt`; new `tests/test_execution.cpp` and `tests/test_runtime_lifecycle.cpp`.

- [ ] Capture current branch/status, source and bundle hashes, compiler/acados versions. Copy old bundles into immutable evidence directories; never overwrite the archived `final-*` bundles.
- [ ] Correct numerical assertions before using them as an acceptance oracle:

```cpp
if (!std::isfinite(a) || !std::isfinite(b) || std::abs(a-b) > tolerance)
  throw std::runtime_error(message);
```

- [ ] Convert the four audit findings into failing behavior assertions: emitted trace differs from accepted candidate; late delivery is accepted; ready-new-epoch plus old state permits enable; 20 Hz/100 ms lead yields no activated plans. Preserve the original counterexample outputs separately because their old exit-zero means reproduction, not successful repair.
- [ ] Add test prerequisites: require positive RUNNING command output before fault injection; require the specific fault/recovery reason and response time after injection. A final zero command by itself cannot pass.
- [ ] Register the new C++ test targets in CMake with assertions active in Release; use isolated ROS domains and actual node callbacks, not a rewritten scheduler model.
- [ ] Run the baseline assertions and save expected failures. Commit only oracle/harness work at this stage.

## Task 2: Define and implement the selected command profile

**Files:** profile/generation and native candidate/output files in the responsibility table; `src/controller/aims_mpcc/node.py`; new `src/controller/config/native_rate_bounded.yaml`; `src/aims_mpcc_rt/tests/test_export.py`, `test_bundle.cpp`, `test_output.cpp`, `test_reanchor.cpp`.

- [ ] Add `command_profile`, default `legacy_bounded_v1`, to the config schema. Permit `rate_bounded_v2` only for the matching native acados path. Legacy node startup must reject that profile explicitly rather than silently enforce the old hard limits.
- [ ] Include the profile in exported/source/native provenance checks. Rebuild generated C for the changed OCP; do not load a v1 bundle under a v2 runtime configuration.
- [ ] For v2, omit the `jerk` and `steering_acceleration` hard-constraint rows. Preserve steering-rate and envelope rows at stage zero, intermediate substeps and terminal state. Keep finite-value/status checks.
- [ ] Remove corresponding v2 seed clipping, angular stopping-distance bounds and reanchor transport eligibility based on jerk/angular acceleration. Attempt the unchanged reanchored controls first; any alternate adjusted candidate still requires a new physical certificate. Old transport code must not silently retain the removed limits.
- [ ] Implement nominal v2 command semantics with actual elapsed publication time and stage-boundary splitting:

```text
acceleration = clamp(planned_acceleration, -brake_limit, accel_limit)
speed_next = clamp(speed_previous + integral(acceleration, elapsed), 0, max_speed)
steering follows the piecewise-linear command endpoints at the plan's phase
abs(steering_change) <= steer_rate * elapsed
```

Do not apply a second acceleration-ramp or steering-rate-ramp limit. Preserve the nominal model's 20 ms held steering samples. Define behavior for nonuniform tick intervals by elapsed time; never silently discard elapsed time with an unreported dt clamp. Intervals outside the supported scheduling bound take the existing scheduling-fault path.
- [ ] Separate continuous internal speed from the minimum motor setpoint and record both. The motor floor remains an explicit output mapping, never a fictitious physical-state jump. Verify zero/low-speed boundary transitions through the real converter.
- [ ] Remove `brake_limit / jerk_limit` from v2 stop-reserve and finish-cap computations. Derive braking distance with the retained delay allowance and brake bound: `distance = speed * delay + speed^2 / (2 * brake_limit)`. Recompute the inverse speed cap and preserve the minimum-drive-speed reserve. This is required maintenance of existing stopping, not a new finish algorithm.
- [ ] Verify a nominal zero-prefix request with `a=0.5`, steering endpoint `0.2`, dt=0.1: at the first 20 ms output, continuous acceleration is 0.5, continuous speed is 0.01 and steering is 0.04 when other caps are inactive. The old values 0.02 acceleration and 0.04 steering rate must not survive v2.
- [ ] Test nonzero prefixes, steering reversal, zero/max speed, angle limits, stop and expiry. Each failure must identify a retained constraint rather than a removed higher-order constraint. Commit profile, generated-contract changes and unit coverage together.

## Task 3: Repair finding A — certify the commands that will execute

**Files:** new `execution.hpp`/`execution.cpp`, `output.hpp`, `src/node.cpp`, `tests/test_execution.cpp`, `tests/test_reanchor.cpp`, `tools/audit_contracts.py`, `CMakeLists.txt`.

- [ ] Create a prospective executed-trace certificate seeded from the actual sampler state and actual forwarded input prefix at takeover. Reconstruct the same output phases and controls as publication, then independently integrate/check the retained physical envelope and optional footprint bounds. Do not call the generated constraint evaluator as the only oracle.
- [ ] Test nominal 20 ms phases, activation between stage boundaries, and explicit 10/20/30/40 ms publication sequences. Report sampling assumptions; a nominal trace test does not prove every possible future jitter sequence safe.
- [ ] Compare nominal model and emitted command trajectories sample by sample. Treat an output clamp that changes a nominal certified plan as a diagnostic event requiring consistency analysis; do not label it redundant protection.
- [ ] Re-run the original 0.79 m/s, 0.4 rad, 0.4 m/s²-prefix counterexample under v1 and v2. V1 must still show its archived mismatch. V2 must either execute a retained-envelope-feasible continuation or reject it based on the actual trace. The test must not assume an initially safe prefix: the supplied prefix is already outside the strict envelope.
- [ ] Add initially feasible independent cases, finite-value failures and genuine envelope/corridor violations. Removing jerk cannot become a blanket acceptance of physically infeasible candidates.
- [ ] Measure certificate and total activation callback duration separately. Keep one final executed certificate after reanchor, remove only checks demonstrably testing the same candidate/trace. If this causes output timing to fail, move preparation outside shared-state ownership and revalidate the takeover prefix before installing; timing failure blocks acceptance rather than loosening the limits.
- [ ] Commit the certificate and its independent tests.

## Task 4: Repair finding B — include result delivery in the deadline

**Files:** `src/aims_mpcc_rt/src/node.cpp`, `tests/test_runtime_clock.cpp`, `tests/test_benchmark_accounting.py`, `tools/acceptance.py`, `tools/nx_joint_load.py`.

- [ ] Keep computation completion timestamp, then obtain the state mutex and timestamp actual result installation/rejection. The same protected decision checks authority, generation and deadline:

```text
compute_end = steady_now()
acquire state ownership
delivered = steady_now()
delivery_elapsed = delivered - submitted
accepted = candidate_valid AND same_generation AND enabled
           AND delivery_elapsed <= request_budget
```

- [ ] Log preparation/native solve/candidate validation, delivery-lock wait, total request delivery, activation, and first publication as separate fields. Capture installation overhead in the reported delivery endpoint. Keep ROS source time and monotonic duration separate.
- [ ] Account for every submitted request: delivered accepted, solver/validation failed, deadline expired, generation cancelled, or still in flight at measurement cutoff. Drain or explicitly classify the cutoff; do not omit unpublished or rejected requests from request latency/miss accounting.
- [ ] Run the real-worker 60 ms contention probe with its pre-hold assertion that the result is not already delivered. With a 50 ms budget it must be rejected and counted late even if compute time is less than 1 ms.
- [ ] Verify late replies cannot replace a valid pending/active plan or renew original source TTL. Commit the timing decision and accounting together.

## Task 5: Repair finding C — bind localization, source history and clock epochs

**Files:** `src/aims_mpcc_rt/src/node.cpp`, `include/aims_mpcc_rt/health.hpp`, `history.hpp`, `tests/test_runtime_lifecycle.cpp`, `tests/test_health.cpp`, `tests/test_history.cpp`, `tools/protocol_probe.py`; `src/aims_racer_system/scripts/localization_monitor.py` and its localization protocol tests.

- [ ] On localization epoch change, invalidate the old snapshot/alignment, active/pending plans and warm-start generation whether the controller is currently enabled or disabled. New ready health alone cannot authorize enable.
- [ ] Extend health publication with the accepted anchor's existing `map_odom_{x,y,z,qx,qy,qz,qw}` payload, tied to its exact epoch and anchor sequence. The monitor already receives these fields from `/localization/anchor_status`; retain them only for a valid committed anchor and clear them on epoch change. Publish readiness, identity and this transform from the same accepted record; do not relabel a cached TF lookup with a new epoch.
- [ ] Native state construction uses that qualified alignment record and tags the odometry snapshot with its epoch. Require an odometry callback accepted after the new anchor record, matching identity, fresh health and source-covering command history before reenable. Preserve existing NDT TF ownership for other consumers. Older health packets lacking the required alignment payload must produce an explicit incompatible-protocol diagnostic for v2, not fallback to unqualified TF.
- [ ] Add source-history coverage to enable/request checks: there must be a forwarded record at or before the original measurement epoch. Never use `AppliedHistory::at()`'s zero fallback as proof of actual coverage.
- [ ] Implement one clock-reset operation that clears ROS source watermark, snapshots, history, proposal matching cache, active/pending plans and progress state, and changes request generation. Reset localization protocol time bookkeeping when simulated ROS time resets; require fresh identity/anchor/health rather than carrying old monotonic-to-ROS offsets forward. Automatic driving reenable is not part of recovery.
- [ ] Exercise actual services/callbacks: old snapshot + new ready epoch + enable before new odometry fails; matching new anchor + odometry + history then permits stationary enable; late old-generation solve cannot install. Publish a sustained ROS clock reset 100 s backward followed by multiple new messages and verify recovery without waiting 100 s.
- [ ] Test reordered/duplicate/retired health packets, new transform with unchanged sequence, malformed quaternion and source history starting after measurement. Commit lifecycle and producer/consumer protocol changes together.

## Task 6: Repair finding D — give the pending plan ownership until disposition

**Files:** `src/aims_mpcc_rt/src/node.cpp`, `tests/test_runtime_lifecycle.cpp`, `tools/acceptance.py`, `tests/test_protocol_ros.py`.

- [ ] Skip request submission while a pending plan awaits takeover. Clear pending only when activated, explicitly rejected/expired, disabled or invalidated by generation change. Give each disposition a sequence and reason in diagnostics.
- [ ] Maintain the request-frequency cap after takeover; record nominal requested frequency, actual submissions and activations separately. Do not classify deliberately skipped pending slots as solver failures.
- [ ] On every eligible publication callback, attempt pending activation once. Fault/stop invalidation wins over activation. Continue the old valid plan while waiting; do not extend its original measurement TTL.
- [ ] Repeat the actual private ROS/selector/converter 20 Hz/100 ms-lead case: achieve nonzero activations, continue RUNNING for the full 30 s nominal test and show no perpetual pending replacement. Default 20 Hz/20 ms must retain its cadence behavior.
- [ ] Cover leads 0/20/50/100 ms at 10/20/40 Hz with supported budgets and different output phases. Some combinations intentionally submit below the nominal rate; assert bounded ownership and eventual disposition rather than identical request counts.
- [ ] Commit scheduling behavior and diagnostics with the regression cases.

## Task 7: Close remaining configuration, recovery and geometry gaps

**Files:** `src/controller/launch/mpcc.launch.py`, `tests/test_runtime_launch_selection.py`; `src/aims_mpcc_rt/src/node.cpp`, `core.cpp`, `tests/test_node_startup.py`, `tests/test_export.py`, `tools/audit_objective.py`, `README.md`.

- [ ] Resolve native horizon/dt from the verified artifact. An explicit conflicting horizon is a startup error. Default TTL derives from `0.8 * N * dt`; an explicit TTL is validated against actual coverage. Resolve an explicit solver timeout for both implementations with implementation-specific defaults (native 0.05 s, legacy 0.25 s).
- [ ] Make native RTI policy explicit: a maximum of one or two passes, subject to the request budget. Map the existing `acados_rti_steps` to that declared maximum or reject a conflicting setting; log actual pass count. Do not silently ignore it.
- [ ] Restore starting/measured footprint checks when `enforce_corridor=true`, using the same transformed rear-axle geometry and widths as candidate checks. Test enabled and disabled corridor configurations separately.
- [ ] Restore authority, live source/receipt age, plan source age/phase, cross-track, corridor flag, effective horizon/dt/budget/lead/TTL, anchor identity and limiter-activation diagnostics. Distinguish last-request observation age from current state age.
- [ ] Test the selected TTL recovery policy explicitly: valid old plan continues through an isolated failure; expiry initiates bounded deceleration; stale state/authority/localization faults independently. A solver result never refreshes its original source age.
- [ ] Address native-worker availability without claiming timeout preemption: stop issuing work to an unavailable worker; publication remains responsive until existing validity limits require stopping. Add bounded process-level termination/restart handling for a wedged solve, with restart disabled for automatic driving reenable. Fault-inject a stalled native call and verify shutdown does not hang indefinitely; do not attempt unsafe cancellation of a C++ thread inside acados.
- [ ] Keep frozen stage geometry as an intentional approximation within each RTI pass. Measure seed-to-solution geometry shifts and compare real legacy and generated objectives at identical points. When a corrective second RTI pass is used, refresh geometry from the first-pass trajectory before that pass, then rebuild final candidate validation parameters consistently; count all refresh/validation work in budget. Run this as a separate comparison after the output-profile A/B. Preserve the maximum of two passes, and reject an overdue delivered result under Task 4. This does not establish equivalence to a converged exact-geometry IPOPT objective.
- [ ] Expand reference tests to wrap, off-path projection, nearby branches and sparse versus dense paths. Record inherited chord-parameter/physical-speed approximation and unidentified longitudinal response as model limitations, not silently fixed migration defects.
- [ ] Commit interfaces separately from any measured geometry-policy change so the latter can be reverted independently.

## Task 8: Independent desktop closed-loop and output comparison

**Files:** `src/aims_mpcc_rt/tools/acceptance.py`, `tools/protocol_probe.py`, new `tools/compare_output_profiles.py`; new report `docs/reports/2026-10-09-mpcc-runtime-repair-validation.md`.

- [ ] Build four immutable bundle cases: circle and saved route, each at 0.5 and 1.0 m/s. Use matching vehicle/weights/horizon/reference for v1 versus v2; vary only the declared output/constraint profile. Label the change to OCP feasibility explicitly.
- [ ] Run real controller → private RC selector → private converter → independent plant. Compare contour/heading P95, steering reversals, requested-versus-emitted command error, limiter activation, stop response, envelope violations, recovery events and complete delivery latency. Keep the numerical plant separate from recorded estimator inputs.
- [ ] Use the existing plant default speed/steering lag and a documented ±25% lag sensitivity set. This explores uncertainty; it is not vehicle identification. Require finite results, route completion, no unexplained lifecycle fault and no retained physical/actuator bound violation. Compare profile tracking with the predeclared allowance: contour P95 <= baseline * 1.1 + 0.01 m, heading P95 <= baseline * 1.1 + 0.5 degrees. If a baseline fails to complete, report the comparison unavailable rather than manufacture a passing ratio.
- [ ] Run fault cases only after positive RUNNING evidence. Require fault/stop output within the existing 100 ms maximum scheduling/freshness response allowance after the condition becomes actionable. Keep command-stop latency separate from the plant's physical stopping duration.
- [ ] If v2 tracks worse, inspect retained soft rate-change costs and command/model mismatch before tuning weights. Any weight change is a separate recorded experiment, not part of the initial profile A/B result.

Build/test commands run inside the existing desktop container after sourcing `/opt/ros/humble/setup.bash` and `/workspace/install/setup.bash`:

```bash
cd /workspace
colcon build --packages-select aims_mpcc_rt --cmake-args -DBUILD_TESTING=ON -DCMAKE_BUILD_TYPE=Release
python3 -m pytest src/aims_mpcc_rt/tests src/controller/tests/test_runtime_launch_selection.py -q
ctest --test-dir build/aims_mpcc_rt --output-on-failure
```

Bundle-dependent CTest entries must be configured with `-DAIMS_MPCC_RT_TEST_BUNDLE=<new verified bundle directory>` before the test run; a run that omitted those entries is incomplete. The new comparison driver must take explicit v1/v2 bundle directories, VESC config, output directory and scenario manifest, log resolved commands, and run the existing acceptance tool; it must not alter original configurations in place.

## Task 9: Corrected NX joint-load qualification and handoff

**Files:** `src/aims_mpcc_rt/tools/nx_joint_load.py`, the new validation report, `docs/operations/known-map-mpcc.md`, original acados runtime plan/report links.

- [ ] Commit/push verified source, pull only the isolated NX branch and compile new ARM libraries/bundles. Record source/provider/native hashes. Preserve the original vehicle worktrees, installed baseline and old evidence.
- [ ] Use the known-map recording and saved initialization with FAST-LIO2 + EKF + NDT genuinely registering. Run the controller on its independent numerical plant concurrently under private ROS topics. Explicitly record that estimator replay supplies CPU load, not feedback/ground truth for this closed-loop test.
- [ ] Repeat three 180 s baseline N10/.1/20 Hz runs with 50 ms budget, 20 ms lead and 0.8 s original-source TTL. Include complete delivered requests, failures and cutoff accounting. Target delivery P95 <=40 ms, P99 <=50 ms, deadline misses <=0.1%; output interval P99 <=30 ms, max <=60 ms; no unbounded request backlog, no three consecutive unexplained candidate failures. Conditional published-plan latency is reported separately.
- [ ] Repeat the 100 ms-lead scheduling counterexample on NX for 30 s. Run both 0.5 and 1.0 m/s numerical route profiles. Optional 40 Hz or shorter mesh is considered only after default-contract qualification and does not replace default evidence.
- [ ] Record actual CPU load, thermal/clocks/power mode, process versions and NDT accepted updates. Synthetic stress alone cannot substitute for estimator joint load.
- [ ] Publish the repaired four-case evidence table, changed constraint semantics, remaining limitations, launch/rollback commands and exact tested bundle names. Keep the current runtime in shadow until these checks pass. Field readiness then means eligible for a supervised low-speed closed-loop trial, not established racing performance.

## Completion accounting

| Finding | Completion proof |
|---|---|
| A: output/candidate mismatch | Matching nominal command trace, independent executed-envelope check, original and initially feasible counterexamples resolved |
| B: delivery deadline | Controlled 60 ms wait rejected under 50 ms budget; all request dispositions counted |
| C: old alignment after epoch | Real enable interleaving rejected until qualified new anchor/state/history; sustained clock-reset recovery verified |
| D: pending starvation | 20 Hz/100 ms lead produces activations and completes nominal runs on desktop and NX |
| Other migration differences | Effective launch parameters, corridor, diagnostics, recovery and worker availability verified; geometry approximation explicitly measured |
| User-selected simplification | No hidden v2 hard jerk/angular-acceleration enforcement in generation, seed, reanchor, output or braking reserve; retained soft costs disclosed |

No acceptance item is satisfied merely by compilation, node READY, a final zero command, fast native solve time, or the continued freshness of republished TF.
