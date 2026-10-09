# Orin NX MPC/MPCC Optimization Implementation Plan

Goal: reliable fixed-map tracking under FAST-LIO2 + EKF + NDT shared CPU load.
Approved scope: repair constraints and timing, compare IPOPT/acados/QP, validate models and weights, then NX shadow and supervised field acceptance. Live tuning and MAP/PP are excluded.

## Baseline

- Upstream integration HEAD: 403b5e1fe18fc9021dfa66c20633e138386fb671.
- NX working files frozen in commit 9629618; original NX worktree remains unchanged.
- Desktop worktree: /home/elesheep/.config/superpowers/worktrees/AIMSRacer/mpcc-nx-optimization.
- Branch: feat/mpcc-nx-optimization.
- Raw evidence: /home/elesheep/aimsracer-data/research/2026-10-09-controller/nx_audit.
- Fresh backup: /home/elesheep/aimsracer-data/maintenance/2026-10-09-mpcc-optimization.

## Execution checklist

- [x] Preserve live NX tracked and untracked changes and create isolated branch.
- [x] Verify baseline tests and capture fixed failure requests with map/reference hashes.
- [ ] Separate actuator bounds from operating envelope; diagnose fixed initial violations, bound/penalize future slack, compare strict jerk with recovery-only relaxation. Independently validate recovery against a bounded deceleration rollout.
- [x] Decouple handover from solve period, validate the actual command prefix, preserve source-based TTL, keep one request in flight and correctly age warm starts.
- [x] Introduce recoverable degradation: single failure retains an unexpired plan; two planning periods without usable updates triggers bounded deceleration; two valid candidates can resume while moving; stopped/localization-lost/operator-disabled requires re-enable. Isolate and restart hung workers.
- [ ] Use consistent actuator assumptions in optimization, history, validation and simulation; distinguish measured parameters from unknowns. Add curvature and forward/backward acceleration speed planning.
- [x] Implement generated-C acados SQP_RTI/HPIPM and fixed-sparsity C/C++ OSQP candidates behind a common versioned plan/result interface. Preserve IPOPT baseline. Precompile ARM64 offline.
- [ ] Run fixed-input and independent closed-loop comparisons at matched 1.0/1.5/2.0 s horizons and 0.1 s steps. Parameter tuning uses fixed per-run YAML and held-out evaluation.
- [ ] Run NX idle/shared-estimation/limited-stress performance conditions, three five-minute repeats each; include failed/late requests and upstream source ages.
- [ ] Run 0.5 and 1 m/s supervised real laps, then 1.5/2/2.5 straight targets with curvature-limited turns.
- [ ] Select a passing backend by P99 then CPU, retaining acados when both differ by <10%; publish evidence and rollback configuration.

## Acceptance

Regression: requests 246/247/248 (235908) and 244 (000919) recover or explain degradation; requests 243/285 remain separate diagnosed rejection regressions. Request 249 is a frozen-input regression; handover correctness requires separate runtime evidence. No TTL renewal, unexecuted-input replay, unsafe slack acceptance or queue buildup.
New-core targets (not measured guarantees): 20 Hz planning, 50 Hz output; full-request P95 <=25 ms and P99 <=40 ms; >50 ms rate <0.1%, max consecutive overruns <=2. Include prep, solve, validation and IPC.
Field: >=3 laps each at 0.5 and 1 m/s, at 1 m/s lateral P95 <=0.10 m and max <=0.30 m, no expiry-induced stop or persistent oscillation. Higher speeds P95 <=0.15 m and max <=0.30 m. Replay, simulation and hardware proof are separate.

## Progress records

Implementation commits and reports append evidence below. Any unmet test/field gate remains explicit; candidate backends do not become the default until acceptance.

### Current execution status

- Evidence and precise timing boundaries are recorded in
  [`nx-optimization.md`](../../../src/controller/docs/nx-optimization.md).
- Numerical recovery gates are implemented; final output smoothing exposed an
  execution-schedule mismatch during independent review. Nominal executed
  schedule checks exposed acceleration and steering target catch-up errors.
  Direct proposals fix both, with three synthetic ROS laps passing. Field
  stopping still exposes the unidentified longitudinal model and mismatched
  finish rule, so constraint/model consistency stays open.
- Matched 1/1.5/2 s solver and ROS comparisons are recorded. Weights were held
  fixed; held-out tuning and real-car model identification remain pending.
- The old-source NX matrix completed three shared and one stress five-minute
  repeats at `9eb309d`, then stopped at a case boundary after repeated numerical
  failure. All four have valid estimator-load coverage, but 87–89 rejected acados
  candidates; none is overall PASS. Partial stress-2 is retained and ineligible.
  QP source `48391d9` completed all nine five-minute worker repeats: 53,991
  requests, six failures and 12 >50 ms requests. Shared P95 23.905–24.133 ms
  meets the worker target; limited-stress P95 25.712–27.109 ms exceeds it.
  All six replay-backed load windows qualify. This historical matrix does not
  qualify the later QP/path source changes.
- Exact native path projection is integrated with guarded live geometry and
  offline preparation. All 27,744 production-wrapper queries and all 19 fields
  of three execution checks are bit equal on desktop and ARM. Unsupported
  representations retain the Python path; artifact replacement requires restart.
- `858dcc3` repairs a narrowly triggered QP stopped-reference-to-cruise warm
  seed. `333a7ce` preserves live cache selection while removing eager home
  lookup; `37bcccb` preallocates dense QP rows with exact masks/bounds and
  layout fallback. All pass independent spec and quality reviews. 338 captured
  QP arrays/masks and production execution certificates remain exact.
- Latest `37bcccb` complete desktop/NX suites each pass 807 tests and the NX
  package builds. The shared-load synthetic lap passes unchanged geometry/speed
  criteria: central median 1.0000 m/s, cross-track max 0.00716 m, finish error
  0.03369 m. Full-process load is valid, source/native artifacts stay unchanged
  and no original driving publisher is enabled. Raw replay events are retained.
- Complete timing remains unqualified: request-to-reply P95 60.29 ms,
  accepted-only request-to-activation P95 87.37 ms and active callback
  P95 31.07 ms. One native iteration-limit failure and one executed-schedule
  rejection remain. Fresh long performance matrix, final output scheduling,
  held-out model/weight tests and physical laps remain open. Backend >50 ms
  rate is 2/331 (0.604%), exceeding the 0.1% target. Independent forensic
  accounting and full raw replay coverage are retained with the run.
- External projection-elision feasibility preserves ten captured schedules
  and gate verdicts in both native and Python fallback comparisons. Four
  desktop budget schedules improve, but all fail rebasing; accepted-plan and
  ARM benefit remain unproved and budget-free guard overhead regresses.
  The prototype is not integrated into product source.
- The car is now in a different venue. Map-matched bag replay supplies estimator
  load; no vehicle driving publisher is started. Supervised laps remain pending.
- No backend has been promoted to the default or selected as passing.

### User-directed priorities after the current milestone

- The captured seq350 finish-only scheduling/model mismatch is deferred from
  the current acceptance work at the user's request. Preserve its reproduction
  and stopping limitation; defer its repair. General in-motion output timing
  remains an active concern.
- Investigate and run the original NPU C++ dynamic MPCC as a complete
  replacement candidate, including its model, constraints, command management
  and execution structure. Keep the existing localization/vehicle interfaces;
  live tuning and MAP/PP remain excluded. Original-source tests and ROS 2/NX
  adaptation are separate qualification stages.
- Reuse the existing speed-mode actuator evidence first: first reported wheel
  motion after 40–53 ms and eight isolated command steps reaching half of the
  wheel-speed change in 0.16–0.40 s. These are response observations, not a
  separately fitted 40 ms dead time plus 0.16 s motor constant. Steering combined
  response is about 0.08 s. The 2026-09-27 calibration also records an equivalent
  steering fit of about 48 ms delay plus 20–27 ms response; the user previously
  chose the combined 80 ms model. These are regional fits, not a requirement
  for a new full identification campaign before a low-speed MPC baseline.
- Original NPU source `6c5012b` now passes 33 selected desktop tests, baseline
  and MPCC C++ solver smoke calls on desktop and NX, and one stock Python
  simulation lap with 250 successful solves. NX reported single-call times
  are 3.778/4.132 ms, with no estimator load or ROS node. The Python reported
  timing excludes preparation and extraction. These trials do not qualify
  ROS 2 integration, this vehicle's model or current joint-load timing.
  Evidence is in `aimsracer-data/experiments/npu-mpcc-adaptation/`.
