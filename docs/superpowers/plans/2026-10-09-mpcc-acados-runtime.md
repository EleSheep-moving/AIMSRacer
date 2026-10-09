# AIMSRacer MPCC acados runtime implementation plan

Approved in the conversation on 2026-10-09. Base: `8e4f6b2698239ccd1141787bc80373077af683a0`; branch: `feat/mpcc-acados-runtime`.

Goal: retain the rear-axle kinematic model and system contracts while reducing complete control latency with an offline-generated acados solver and a C++ ROS 2 runtime. Implementation uses test-driven development, then independent specification and quality reviews.

## Tasks

- [ ] Preserve the baseline; create the isolated worktree and reproducible desktop container.
- [ ] Export a portable solver bundle from the existing acados OCP: pin acados 0.5.5, publish configuration/reference fingerprints, model/constraint C functions and a small C ABI. Compile independently on x86/ARM; require matching artifacts at startup. No runtime generation.
- [ ] Build `src/aims_mpcc_rt`: rear-axle state `[x,y,yaw,speed,progress,steering]`, controls `[acceleration,steering_command,progress_speed]`, three auxiliary previous-control states. Preserve existing RK4 transitions, objective scales, limits and tau=0.08.
- [ ] Preserve the exact periodic quintic path; prepare stage geometry outside the solver. Shift warm starts by actual elapsed time with interpolation, not a minimum whole-stage shift. Generate one consistent nonlinear candidate and validate it once. One RTI pass, one optional corrective pass within the 50 ms request budget.
- [ ] Separate latest-only 20 Hz solving and 50 Hz command publication. Use forwarded `/ackermann_cmd` history to forecast source/takeover states. Keep original source epoch and 0.8 s TTL. Discard late replies. Continue valid old plans after isolated failure; decelerate after expiry.
- [ ] Preserve odometry, TF, localization identity/health, RC authority, enable service, drive and diagnostics contracts. Support `implementation:=legacy|acados_cpp`, default legacy, and isolated shadow outputs/services. Preserve localization protocol-v1 epoch/sequence/freshness rules. Prevent dual drive owners.
- [ ] Audit gate ownership/cost; preserve first-round constraint semantics and `enforce_corridor=false`; merge duplicate rollouts/checks. Keep finite/status/nonlinear feasibility and takeover continuity checks. Record each rejection and timing.
- [ ] Desktop model/cost equivalence and independent closed-loop 0.5/1.0 m/s checks. Cover startup, braking, reversal, wrap, map changes, stale input, clock reset, manual takeover, failure/late replies and artifact mismatch. Reproduce historical failures without extending near-finish repair scope.
- [ ] Local commit/push; isolated NX pull/build. Run map-matched replay with FAST-LIO2+EKF+NDT genuinely registering, shadow controller only. Three runs >=180 s each, all attempts included. Compare matched legacy/new conditions; record power/temperature/clocks/threads/versions.
- [ ] Evaluate 40 Hz and 20x0.05 / 25x0.05 only after the baseline passes. Integrator steps <=20 ms independent of publisher period; dt-scaled objective equivalence. Default delivery stays 20 Hz, 10x0.1.
- [ ] Deliver source, builds, tests, manifests, timing report and launch/rollback instructions. Do not claim field qualification from simulated or replay evidence. Physical driving is a later operator test.

## Acceptance

20 Hz: complete request P95<=40 ms, P99<=50 ms, >50 ms fraction<=0.1%; publish interval P99<=30 ms, max<=60 ms; no queued request accumulation; candidate failure fraction<=0.1% outside injected faults and no three consecutive failures.

Tracking: new contour P95 <= old*1.1+0.01 m and heading P95 <= old*1.1+0.5 degrees, complete the same independent simulated route with no new actuator violations. Log observation age, geometry, native solve, validation, handover, first publication, CPU and memory separately.

40 Hz candidate: 25 ms request budget, P95<=20 ms, P99<=25 ms, deadline fraction<=0.1%. Failure to meet timing is reported by stage; freshness checks are not relaxed to pass.

## Reproducibility

NPU reference: `npu-ius-lab/Roboracer_China_2026`, `real-car-original`, commit `6c5012b9cd310c8fca5281c408298ffb5d4b3885`. acados: v0.5.5 commit `59d93e17d2985fdd73fc58b8a83ed8f83a024171`. NPU stock core smoke timings are not complete matched own-car latency evidence. Foreign residual/tire parameters are not imported.

Existing own-car vehicle profile and reference bundle remain authoritative. Data: NX session `2026-10-09/000919-b-1mps-a0p6-j2/bag`, map `20260928_010503/map.pcd`, saved seed from the previous optimization experiment.

## Implementation findings and adjustments

- Existing Python acados applies native stage scaling dt, unlike the legacy IPOPT stage sum. Export explicitly uses stage scaling dt/0.1 and terminal scaling 1, preserving the baseline objective at dt=0.1 and the physical-time weighting when dt changes.
- A plan arriving after its forecast cannot certify unexecuted new-plan controls as actual command history. Compare the forecast advanced through real old-plan history against observation, then construct and validate a reanchored candidate with the unchanged, not-yet-executed controls and actual applied prefix. Original source TTL remains unchanged. This distinct candidate requires validation; the native C++ activation cost is included in the complete publisher callback measurement. If that cost violates the output timing criterion, it must move to an independent activation worker without weakening acceptance semantics.
- Strict envelope is the first production profile. Experimental soft-recovery profiles remain available in the legacy implementation and fail C++ node startup until their independent recovery comparator is implemented; they must not be silently accepted under only the optimizer slack certificate.
