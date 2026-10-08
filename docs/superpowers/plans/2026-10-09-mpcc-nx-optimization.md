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
- [ ] Verify baseline tests and capture fixed failure requests with map/reference hashes.
- [ ] Separate actuator bounds from operating envelope; diagnose fixed initial violations, bound/penalize future slack, compare strict jerk with recovery-only relaxation. Independently validate recovery against a bounded deceleration rollout.
- [ ] Decouple handover from solve period, validate the actual command prefix, preserve source-based TTL, keep one request in flight and correctly age warm starts.
- [ ] Introduce recoverable degradation: single failure retains an unexpired plan; two planning periods without usable updates triggers bounded deceleration; two valid candidates can resume while moving; stopped/localization-lost/operator-disabled requires re-enable. Isolate and restart hung workers.
- [ ] Use consistent actuator assumptions in optimization, history, validation and simulation; distinguish measured parameters from unknowns. Add curvature and forward/backward acceleration speed planning.
- [ ] Implement generated-C acados SQP_RTI/HPIPM and fixed-sparsity C/C++ OSQP candidates behind a common versioned plan/result interface. Preserve IPOPT baseline. Precompile ARM64 offline.
- [ ] Run fixed-input and independent closed-loop comparisons at matched 1.0/1.5/2.0 s horizons and 0.1 s steps. Parameter tuning uses fixed per-run YAML and held-out evaluation.
- [ ] Run NX idle/shared-estimation/limited-stress performance conditions, three five-minute repeats each; include failed/late requests and upstream source ages.
- [ ] Run 0.5 and 1 m/s supervised real laps, then 1.5/2/2.5 straight targets with curvature-limited turns.
- [ ] Select a passing backend by P99 then CPU, retaining acados when both differ by <10%; publish evidence and rollback configuration.

## Acceptance

Regression: requests 246/247/248 (235908) and 244 (000919) recover or explain degradation; requests 243/285 remain separate unidentified failures; request 249 proves handover correctness. No TTL renewal, unexecuted-input replay, unsafe slack acceptance or queue buildup.
New-core targets (not measured guarantees): 20 Hz planning, 50 Hz output; full-request P95 <=25 ms and P99 <=40 ms; >50 ms rate <0.1%, max consecutive overruns <=2. Include prep, solve, validation and IPC.
Field: >=3 laps each at 0.5 and 1 m/s, at 1 m/s lateral P95 <=0.10 m and max <=0.30 m, no expiry-induced stop or persistent oscillation. Higher speeds P95 <=0.15 m and max <=0.30 m. Replay, simulation and hardware proof are separate.

## Progress records

Implementation commits and reports append evidence below. Any unmet test/field gate remains explicit; candidate backends do not become the default until acceptance.
