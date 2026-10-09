# MPCC runtime contract repair validation

Date: 2026-10-09. Branch: `feat/mpcc-acados-runtime`.
Controller source: `297fd22847741c177a6c794f7721e28d8f9de646` (startup discovery follow-up). Final replay-fixture source: `4f8488be916c0caa4499d0090f8beef1bb5163bc`; controller source/binary is unchanged by the fixture repair. Matched first-round source: `84f61c95f342aa8cb637bcd974a654c8c2144128`; control mathematics and bundles are unchanged by the follow-up.
Status at the 2026-10-09 measurement freeze: implementation, desktop tracking comparison and NX software timing/load qualification complete; the independent physical-response criterion remains unmet. On 2026-10-10 the operator deferred longitudinal response identification and authorized preparing locked-car checks and supervised low-speed field trials. The unmet synthetic criterion remains recorded; it is not a runtime gate or a prerequisite for those trials. No physical driving acceptance has been established.

## What changed

1. `rate_bounded_v2` removes **hard command jerk and steering-command acceleration** constraints from the OCP, seed, reanchor, output and braking reserve. Acceleration/braking, steering rate/angle, speed and combined operating envelope remain. The six physical states, kinematic model, steering response and objective weights remain. The soft rate-change penalty remains, with its own normalization scale.
2. An independent execution certificate reconstructs the actual held publication schedule, integrates the physical model at 2 ms, and checks the retained envelope and optional footprint. Activation uses the same current physical prediction and finish cap as publication. A packet crossing original-source TTL or horizon applies the recovery budget to its expired segment.
3. The worker deadline ends **after result delivery under the state mutex**. Compute, lock wait, delivery, activation and first publication are distinct measurements. Every submitted request has a terminal disposition or explicit cutoff.
4. Localization health atomically carries the accepted full transform, epoch, anchor sequence/stamp and map identity. New epochs invalidate state/history/plans even while disabled. Healthy same-epoch anchors can update a fresh source-covered snapshot without renewing its physical measurement age; either odometry/health arrival order works.
5. A pending plan owns its slot until activation, expiry or cancellation. More recent requests cannot repeatedly replace it. Original measurement TTL remains 0.8 s at the default N10/.1 mesh.
6. Launch verifies the artifact mesh/configuration and resolves budget/TTL consistently. Native supervision bounds process shutdown and restarts disabled. A stalled acados call is not cancelled inside its C++ thread.
7. At most two RTI passes remain. The corrective pass refreshes stage geometry from the first trajectory. Stage-zero acceleration capacity is supplied as an exact bound under the existing envelope, avoiding a misleading quadratic linearization. SDK cache loading forbids online regeneration/building.

## Artifacts and scope

- Primary evidence: `/home/elesheep/aimsracer-data/experiments/mpcc-acados-runtime/repair-final-20261009`.
- Final frozen desktop qualification: `qualification-final/frozen.json`; earlier `qualification`, `qualification-r2` and failed NX underlay/startup attempts remain archived.
- Extra regression/review evidence: `/home/elesheep/aimsracer-data/repair-20261009` and `experiments/mpcc-acados-runtime/repair-20261009`.
- NX: corresponding evidence under `/home/aims/aimsracer-data/experiments/mpcc-acados-runtime/repair-final-20261009`, isolated source worktree `/home/aims/.config/superpowers/worktrees/AIMSRacer/mpcc-acados-runtime` and `ros-install` overlay.
- Immutable bundles: `qual-{v1,v2}-{circle,route,field}-{v05,v10}-n10`. Matching pairs differ only in the declared command/constraint profile; cost, vehicle, mesh and reference match. Generated sources were rebuilt against the target's pinned acados installation. `unqualified-v2-dt05` is a rejection fixture, not a usable control profile.
- acados v0.5.5 commit `59d93e17d2985fdd73fc58b8a83ed8f83a024171`.

The actual controller, private RC selector and VESC converter close a numerical loop with an independent lagged plant. Estimator replay supplies **CPU load only**, not controller feedback or ground truth. Tests never publish physical vehicle commands. Original vehicle worktrees, installed software and old `final-*` artifacts remain preserved.

## Repair and regression evidence

| Check | Result | Evidence / boundary |
|---|---|---|
| Final desktop C++ suite | 28/28 PASS | `ctest-startup-witness.log`, real node callbacks and bundles |
| Final NX ARM C++ suite | 28/28 PASS | `ctest-startup-witness.log`; float residual tolerance retains exact zero wire stop |
| Final Python unit/interface tests | 70 PASS | `pytest-startup-final.log`, including supervisor nonce/witness and SDK checks |
| Actual map ROS protocol | 7/7 PASS, 158 checks | `pytest-protocol-startup-final`; positive driving prerequisite, atomic health, natural reordered arrivals, original drive publishers 0 |
| Localization monitor regressions | 2/2 NX CTest entries PASS | one Python stub regression plus one real C++ ROS alignment/clock-recovery integration test |
| SDK cache reuse | 6 PASS | forced reuse check failure cannot invoke generation/build; prepared hashes and loaded-library route preserved |
| Complete delivery deadline | PASS | forced 60 ms result wait rejected under 50 ms budget |
| Pending lead/rate matrix | 12/12 PASS | `qualification-final/protocol-results.json`: leads 0/20/50/100 ms × request caps 10/20/40 Hz |
| Late supervisor discovery / restart | 7/7 + 8/8 real ROS checks PASS | `runtime/supervisor-startup-witness`, new nonce/disabled state each restart |
| Worker stall | PASS within test scope | real ROS node starts RUNNING, call-entry stall; publication remains responsive; source TTL recovery disables; bounded TERM/KILL reaps process |
| Stage-zero exact ordered replay | 28/28 feasible requests PASS | no extra RTI beyond max2; original failures 17–28 retained |
| TTL-crossing sampler/certificate | RED then GREEN | production repair and independent 18-case full-hold review; unsafe physical prefixes remain rejected |

The blocked-worker probe is a call-entry fault injection, not a reproduced internal acados deadlock. It observed 105 commands and 22 statuses while blocked; max command gap 20.60 ms, TTL recovery at 0.813374 s for the 0.8 s TTL, and forced bounded shutdown/reap in 0.5081 s. Whole-process unavailability is a separate case handled by supervision and existing downstream authority timeout.

## Default desktop timing

30 s per case, 20 Hz request cap, 20 ms forecast lead, 50 ms delivery budget, original-source TTL 0.8 s. Final defaults ran serially after the protocol matrix; matched comparison has a separate concurrency label.

| Cruise | Requests | Solver / handover failures | Complete delivery P95 / P99 | Output gap max | Contour P95 | Heading P95 |
|---|---:|---:|---:|---:|---:|---:|
| 0.5 m/s | 599 | 0 / 0 | 0.409 / 0.540 ms | 20.196 ms | 0.98 mm | 0.408° |
| 1.0 m/s | 599 | 0 / 0 | 0.362 / 0.547 ms | 20.249 ms | 0.84 mm | 0.312° |

Both tracking and timing checks pass; all requests are accounted for, with no late/inflight request. These are desktop measurements, not NX timing or actual vehicle tracking claims.

## Matched output comparison

Final controller `297fd22`: **24/24 tracking runs and 12/12 relative comparisons PASS**. Each run is 90 s: circle/saved route × 0.5/1.0 m/s × v1/v2 × plant lag multipliers 0.75/1/1.25. All 24 complete at least one geometric lap (minimum 1.316 laps). Four shape/speed groups run concurrently in separate ROS domains; serial matched profiles within each group. The 714 source/binary/bundle inputs are checked before and after each case. Wall time: 552.06 s.

V2 contour and heading P95 are each lower in 11/12 pairs; route 0.5 m/s with lag 1.25 is slightly worse but within allowance. Limits: contour P95 <= baseline*1.1+0.01 m and heading P95 <= baseline*1.1+0.5°. All baselines complete, so all relative comparisons are defined.

| Metric across cases | v1 | v2 |
|---|---:|---:|
| Contour P95 range | 0.262–28.818 mm | 0.025–19.042 mm |
| Heading P95 range | 0.518–77.841 mrad | 0.032–50.533 mrad |
| Command-direction reversals | 1–86 | 1–92 |
| Logged continuous limiter samples | 2,759 | 0 |
| Requested/emitted acceleration metadata mean absolute error | 0.003405 m/s² | 0.002212 m/s² |
| Maximum acceleration metadata error | 0.1 m/s² | 0.497494 m/s² |

Zero v2 limiter flags do not mean zero wire/model mismatch: the motor floor is a separate mapping. Steering reversals refer to command increments >0.001 rad, not measured physical steering. The comparison does not promise less oscillation in every scenario.

**The independent physical-envelope criterion remains unsatisfied in all 24 runs**: 419/104030 samples exceed the bound; peak utilization 7.1111. This is distinct from the tool's tracking PASS. Evidence: `qualification-final/matched-results.json`, `matched-tracking-summary.csv/.json/.md`, `matched-plant-envelope.csv` and 24 per-case manifests. Earlier `qualification-r2` comparison is preserved separately.

## NX joint load

Corrected replay stage: **3/3 × 180 s software timing/load qualification PASS**, private domain 224, N10/.1, 20 Hz request cap, 50 ms complete delivery budget, 20 ms lead and 0.8 s original-source TTL. The controller closes its own numerical loop while the recorded raw sensors run actual FAST-LIO2 + EKF + NDT on the same NX.

| Run | Requests | Complete delivery P95 / P99 / max | Output gap P99 / max | First publication P99 | NDT committed updates in window |
|---|---:|---:|---:|---:|---:|
| 1 | 3,594 | 0.830 / 1.003 / 2.150 ms | 20.340 / 25.838 ms | 40.099 ms | 1,800 |
| 2 | 3,595 | 0.843 / 1.035 / 1.778 ms | 20.420 / 26.097 ms | 40.145 ms | 1,800 |
| 3 | 3,595 | 0.842 / 1.023 / 3.022 ms | 20.331 / 27.127 ms | 40.097 ms | 1,800 |

All **10,784 requests** are accounted for. No solver/validation failure, late delivery, handover reject or unfinished request in these measurement windows. The largest full publication callback was 4.468 ms. Complete delivery includes preparation, native work, validation and state-mutex result installation; **it does not include waiting for forecast takeover and first publication**. The separately conditional first-publication column includes those waits. Percentiles are per run, never pooled or averaged.

The load covers each entire 180 s window: 1,800 native registrations and 1,800 accepted anchors per window; maximum native update gap across the three windows is 181.24 ms. The recorded power mode is **MAXN_SUPER**, sampled CPU frequency 1984 MHz. Temperature statistics cover warmup/measurement/drain rather than only the controller window; raw `tegrastats`, `nvpmodel` and `jetson_clocks --show` are archived. This is the recording's mostly stationary initial segment, not a high-speed moving-estimator or physical tracking acceptance.

The separate **30 s, 100 ms lead** NX saved-route tests pass at 0.5 and 1.0 m/s: 200 submitted/activated requests each, 399/400 owned-pending skips respectively, no failure/late/handover reject. Their output gap maxima are 20.353/20.357 ms. These tests show eventual plan ownership, not a nominal 20 Hz submission rate at 100 ms lead.

### Preserve earlier failures

- `qualification-r2`: stale NDT installation lacked required anchor/timing protocol; trusted-load startup did not complete. Initialization itself reached active NDT. This is excluded from controller timing qualification.
- `qualification-r3`: correct NDT selected through a whole overlay, but FAST-LIO provider also changed. Its passing run is supplemental evidence.
- `qualification-r4`: latest FAST-LIO selected, but late supervisor discovery misclassified legitimate RUNNING as automatic startup; corrected with the process startup protocol.
- `qualification-final`: controller follow-up version, original rosbag QoS. Results remain **PASS / PASS / INVALID_LOAD**. The third FAST-LIO exited with `IMU source-time gap exceeds imu_max_gap_sec`; only 20 body clouds and 18 anchors remained, and no native registration covered the controller window. Its fast controller timing cannot count as joint load.

### Replay QoS investigation and repair

In that invalid-load run, consecutive received IMU source timestamps jump **215.909 ms**, although the bag contains the intervening **42 IMU samples** (maximum bag gap in the interval 6.78 ms). Receive-time silence is about 342 ms. This is not evidence that those samples are absent from the recording, and the existing 30 ms continuity guard correctly refuses the broken sequence.

Controlled private experiments retain the actual FAST-LIO executable, reader depth4096 and guard30ms:

| Input / paused reader | Default writer history | Explicit IMU writer history4096 |
|---|---|---|
| IMU-only, approximately 350 ms pause | All 550/550 received, no failure | All 550/550 received, no failure |
| IMU + LiDAR, approximately 350 ms pause | 255.747 ms source gap, continuity FATAL, exit1 | 770/770 received, max gap13.263 ms, exit0 |

The multi-topic A/B demonstrates a transport loss mechanism and its mitigation under a controlled pause. **The reason for the original natural pause is not fully established.** Do not label it an internal worker deadlock or a fully explained hardware failure.

The minimal repair is confined to replay: installed `params/replay_sensor_qos.yaml` explicitly sets `/livox/imu` reliable/volatile/keep-last4096. Other topics retain recorded reliability/durability adaptation. Raw IMU units, filtering, guard, FAST-LIO source and vehicle launch remain unchanged. The fixture supports an explicit QoS path and verifies source/installed/selected hashes; seven regression tests pass, and NX CTest passes3/3: the new replay fixture test, a Python monitor stub regression, and a real C++ ROS alignment/clock-recovery integration test.

The three new repeats use `qualification-replay-qos` and frozen source/installed QoS SHA `acc26976a5533a7a5945d149cda668535a43d6ce110f54e9b14f9da799ae2776`. Original attempts are not renumbered or omitted; these few trials do not establish a population failure probability.

### Provenance and evidence

The corrected package-only environment keeps:

- FAST-LIO: `/home/aims/AIMSRacer/install/fastlio2`.
- NDT/ndt-OMP: `/home/aims/AIMSRacer-fastlio-ndt/log/fastlio-ndt/install/{lidar_localization_ros2,ndt_omp_ros2}`.
- Updated controller/monitor: isolated `repair-final-20261009/ros-install`.

Controller ELF SHA is `cab07302bc2a010a9547200b6a4410e985fa98455e53b525cf20642403cf8404` and stays identical across the two recorded-QoS stages. NDT component SHA is `c55cb413931c4dfd77eef8f97026a80983672ce8b65ae249d0f29de87af42197`; its existing provider source has local changes, so binary and source hashes, not only its Git HEAD, define the tested version. Executable, shared-library, acados-provider, source, map, bag metadata and selected fixture hashes are preserved.

Host evidence: `nx-results/previous-3`, `nx-results/replay-qos-3`, `nx-results/summary/nx-run-summary.{json,csv,md}`, `runtime/nx-fastlio-third-round-investigation.md`, both transport A/B directories and `nx-evidence.tgz`. Remote original records remain in place. The pre-repair native installation retains SHA `b7cfa98a12ab18f0de6260ce09600a4dcf589b0be0f0c3721e38bfb944394c56`.

## Earlier counterexamples and their repairs

- The original default short numerical run completed tracking but had two small takeover-envelope rejects. Bounded transport now creates a distinct fully validated candidate; original violation groups remain in the CSV. It does not relabel the original candidate accepted or extend source TTL.
- Ordered long-lead requests 17–28 exceeded the exact stage-zero envelope because of a quadratic constraint linearization around near-zero acceleration. Exact input capacity fixes this; refreshing geometry alone did not.
- A finish-cap override mixed stale physical feedback with command speed and could exceed the envelope. Current decision-time prediction, shared cap and conservative per-hold braking budget fix the feasible counterexample; truly infeasible prefixes still fail.
- A normal accepted same-epoch anchor arriving just before the next odometry packet caused a false localization freshness stop. Atomic same-epoch adoption repairs this ordering; a new epoch still requires new state/history.
- An Orin run with the correct latest FAST-LIO passed startup/explicit enable but the supervisor received RUNNING first and stopped it as `startup_not_disabled`. A per-child nonce and persistent disabled-birth/enable witnesses repair actual discovery ordering; no artificial harness wait was introduced. The old real ROS reproducer still fails, while the new real late-observer and restart cases pass.
- The first 12-case frozen matrix passed 11/12: 40 Hz/100 ms lead exposed a packet straddling TTL whose expired segment bypassed the recovery envelope. `84f61c9` fixes this; the newly frozen matrix passes 12/12.

Original failed evidence remains inspectable. No failing candidate is retroactively relabeled valid.

## Model limits and physical release

The ideal acceleration model does not identify the VESC speed PID. The minimum motor speed setpoint is 0.2 m/s; an independent 0.2 s speed-lag plant accelerates at about 1 m/s² on its first positive packet, above the internal 0.5 m/s² bound. Physical plant envelope violations are therefore reported **without** asserting that tracking PASS proves the plan's physical-model criterion. ±25% lag sensitivity is uncertainty exploration, not measured vehicle identification.

The reference retains its chord-distance spline parameter. Wrap/off-path/nearby branches and sparse/dense samples have independent regression coverage; virtual progress is not exact physical arc length. Frozen stage geometry with a corrective refresh is still an RTI approximation to a converged full-quintic IPOPT objective.

The prospective certificate assumes the declared held output schedule and ideal longitudinal model. It does not prove every future jitter sequence, changing cap, traction disturbance or actual motor acceleration. The native v2 50 ms stage mesh is explicitly rejected because stage-local 20/20/10 ms holds do not match the global 20 ms schedule.

Software timing/load results are recorded. Following the operator's 2026-10-10 direction, vehicle startup uses standard interfaces with explicit enable; the shadow mode is removed. Longitudinal response identification is deferred to the planned acceleration-control interface work. Start with locked-car checks and a supervised 0.5 m/s field trial. Passing synthetic timing/tracking does not establish racing acceptance. Historical measurements above retain their original source/binary provenance; they are not measurements of the interface-removal revision.

## Launch / rollback

Use the updated `aims_racer_system` C++ monitor and the matching v2 configuration/reference/artifact together. Offline artifact generation and ARM rebuild commands are in the [runtime README](../../src/aims_mpcc_rt/README.md); localization protocol requirements are in [operations](../operations/known-map-mpcc.md).

```bash
# Source ROS + vehicle + localization underlays, then the isolated tested overlay.
source /opt/ros/humble/setup.bash
source /home/aims/AIMSRacer/install/setup.bash
source /home/aims/AIMSRacer-fastlio-ndt/log/fastlio-ndt/install/ndt_omp_ros2/share/ndt_omp_ros2/package.bash
source /home/aims/AIMSRacer-fastlio-ndt/log/fastlio-ndt/install/lidar_localization_ros2/share/lidar_localization_ros2/package.bash
source /home/aims/aimsracer-data/experiments/mpcc-acados-runtime/repair-final-20261009/ros-install/local_setup.bash
ros2 launch aims_mpcc mpcc.launch.py implementation:=acados_cpp \
  cpp_solve_frequency:=20.0 vehicle_config:=/absolute/path/to/matching-v2.yaml \
  path_directory:=/absolute/path/to/verified-reference \
  artifact_directory:=/absolute/path/to/matching-arm-bundle \
  log_directory:=/absolute/path/to/new-log-directory
```

The tested circle/route bundles are qualification fixtures; do not substitute them for a verified physical map route. Rollback withdraws authority first, stops the native runtime, and selects `implementation:=legacy` with the original v1 config/artifact. Do not run two enabled drive owners or reinterpret a v1 bundle as v2.
