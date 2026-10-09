# MPCC native migration: whole-chain contract audit

Date: 2026-10-09. Runtime audited: `199d168f63803a1c0ea284f614a1bf071d8ed232`.
Legacy baseline: `8e4f6b2`. Legacy ROS node, supervisor, solver and execution
validator sources remain unchanged; the shared `backend_models.py` interval
helper gained fractional-mesh support while retaining the .1 s mesh.
This follow-up adds audit tools and corrects documentation. It does **not**
repair or replace the production runtime binary.

## Decision

**Hold physical closed-loop release of the native prototype.** Keep it in
shadow while the confirmed gaps below are repaired and independently verified.
The three NX replay/independent-plant runs establish useful computation and
synthetic tracking measurements. They do not establish complete delivery
deadlines, executed-envelope equivalence, or real-vehicle acceptance.

The earlier claim that all remaining checks were preserved and only duplicate
rollouts were merged was too broad. The missing held-output certificate checks
a different trajectory from the optimizer candidate; removing it changes
coverage. Likewise `complete_s` was incorrectly described as complete request
delivery time: it is captured before result delivery reacquires state ownership.

## Coverage matrix

| Domain | Source/test comparison | Conclusion |
|---|---|---|
| State and vehicle model | Rear-axle Cartesian six-state bicycle, steering lag, understeer correction, previous-control memory | Model formula matches at 100 ms; three auxiliary states are bookkeeping |
| Discretization | Legacy RK4 and generated interval, five held 20 ms steering increments | Source match at .1 s; .05 s mesh is separate evaluation, not legacy parity |
| Cost and normalization | Actual legacy Opti expression versus compiled native cost functions | Same weights/formula when geometry is frozen identically; full objectives differ |
| Geometry/projection | Actual quintic versus frozen tangent/heading/curvature, wrap/progress and speed preview | Exact reference export retained; frozen optimization is intentional approximation; off-path/branch coverage incomplete |
| Warm start and RTI | Elapsed fractional interpolation, failed-update cache age, one/conditional second RTI | Intentional algorithm changes; `acados_rti_steps` config semantics not preserved |
| Input/actual history | Source/receipt timestamps, rear-axle transform, selector metadata/deadzone, 5 ms bridge | Main formulas match; source coverage check missing and clock-reset recovery regresses |
| Localization/TF | Map identity and protocol-v1 health, alignment snapshots and epochs | Health sequencing retained; retired-epoch snapshot can remain usable |
| Worker/result scheduling | Latest-only request, pending activation, generation, mutex and deadline | Pending starvation and excluded delivery wait confirmed |
| Actual output execution | Macro controls versus actual 20 ms bounded sampler | Strict-envelope continuation can pass macro validation but fail executed validation |
| Fault/recovery/finish | Old-plan continuation, stopping, finish cap, manual takeover | Recovery onset changes from two solve periods to source TTL; worker has no process restart; near-finish semantics require separate boundary coverage |
| ROS/launch/diagnostics | Topics/services/frames/poses, runtime selection, effective parameters | Core names/frames retained; native horizon/timeout and diagnostic contracts differ |
| Artifacts/evidence | Source/native fingerprints, target provider hashes, immutable inputs, release assertions | Archived runtime source provenance matches; existing tests have coverage and oracle weaknesses |

## Confirmed blocking findings

### A. Macro candidate and actual output are not the same certificate

Legacy `src/controller/aims_mpcc/execution.py:233` simulates the actual bounded
20 ms execution schedule. Native `src/aims_mpcc_rt/src/node.cpp:313` validates
the reanchored macro candidate, then `:367` feeds its controls into the sampler.
There is no corresponding complete prospective executed-envelope certificate.
Hard output rate/acceleration clamps alone do not establish the combined
longitudinal/lateral envelope.

Offline reproduction uses the actual compiled `Core::reanchor` and
`OutputSampler`, the measured strict N10/.1 profile, plus the independent legacy
`validate_candidate` / `validate_execution` implementations:

```text
Physical initial speed             0.79 m/s
Physical and commanded steering    0.4 rad
Applied acceleration               0.4 m/s^2
Candidate accelerations            0.3, 0.2, 0.1, then 0
Macro candidate                    accepted, violation ~2.22e-16
First five emitted accelerations   0.38, 0.36, 0.34, 0.32, 0.30
First emitted utilization          1.1148289
Maximum executed utilization       1.1358023 (limit 1)
Legacy macro checker               accepted
Legacy executed checker            rejected
```

The applied prefix is already outside the strict envelope. This is **not** a
claim that an initially safe prefix becomes unsafe. The counterexample proves
that a macro-valid continuation assumes acceleration can decrease across the
100 ms interval, while the real 20 ms smoother must decrease gradually and
retains/peaks at a violation. The first five emitted acceleration samples match
the independent legacy schedule to 1e-12. Spatial integration, expiry and finish
are excluded to isolate this nominal ideal-acceleration certificate; physical
motor response is not identified by this test.

Disposition: align candidate/control semantics with actual execution or provide
an independent native executed-trace certificate. Do not describe this as
removing a redundant check or weaken limits to accept the counterexample.

### B. Deadline and timing omit result-delivery mutex wait

`node.cpp:261` captures `completed`/`complete`; `:263` then waits for the state
mutex and `:268` installs the pending result. The 50 ms decision uses the
earlier timestamp, so a result can be delivered late but accepted as on time.

The audit probe uses the actual worker with a supported startup frequency of
10 Hz to separate consecutive requests. Under the acquired state mutex it
requires that request 1 has submitted **and has not delivered** (`last_` is not
1 and pending is absent) before holding ownership for 60 ms. It then observes
accepted pending result 1. This is a controlled contention reproduction, not a
new performance qualification. See `audit-20261009/repro-final/report.json` for
reported computation **0.7194 ms** versus observed delivery lower bound
**60.0897 ms**, with a 50 ms budget and accepted pending result.

Disposition: include result visibility/installation in the deadline and log
worker computation, delivery wait and first publication separately. Existing
1.02–1.04 ms NX P95 values remain computation-stage evidence. Published-plan
first-publication P95 ~39 ms remains independently measured; unpublished
results do not have a first-publication sample.

### C. Epoch change does not invalidate the old alignment snapshot

Native health callback `node.cpp:113` faults on an epoch change, but
`fault_locked` does not invalidate `snapshot_`. Enable at `:127` can use its
old alignment if the new health epoch is ready before a replacement odometry
callback. Legacy `node.py:153` clears `map_alignment`, and enable at `:274`
requires a replacement.

This is a source-confirmed regression, not yet a full ROS epoch/TF interleaving
reproduction. Repair must also define which new TF/state snapshot belongs to
the ready epoch; a continuously refreshed TF stamp is not anchor provenance.

### D. Future pending results can be replaced indefinitely

Native `node.cpp:230,268` allows continued submission/replacement while a future
pending result awaits activation. Legacy `node.py:449` suppresses submission
while a pending plan exists.

An isolated actual ROS/RC/converter numerical run at 20 Hz, 50 ms budget and
**100 ms lead** (valid startup parameters) reproduced:

```text
20 requests / 20 returned results
0 worker failures, 0 late replies
0 activation attempts / 0 accepted plans
RECOVERING: Plan expired
```

The test was scheduled for 5 s but stopped after 1.003 s when nominal RUNNING
was lost. It did not start any public actuator publishers. The previously
reported 40 Hz/20 ms lead case (724 activations/2392 requests) is a less extreme
instance of the scheduling issue. This does not claim starvation occurred in
the successful default 20 Hz/20 ms baseline.

Disposition: define pending ownership/activation eligibility when replacing
future candidates, and test lead/cadence/publication phase combinations.

## Additional functional and interface gaps

| Priority / applicability | Finding and evidence | Required follow-up |
|---|---|---|
| P1 conditional on corridor enabled | Native enable/odometry omit measured and starting footprint checks; legacy `node.py:244,277` has them | Preserve measured/start behavior when `enforce_corridor=true`, or refuse that unsupported profile. Current production bundles use false |
| P2, clock reset/replay | `node.cpp:202` clears history and returns while retaining old ROS source epoch; each lower-epoch message repeats it | Offline real callback probe seeded an old source 100 s ahead: two valid current messages both clear history and leave it unchanged. Full `/clock` session recovery still needs testing |
| P2, incomplete source history | `fresh_locked` at `node.cpp:174` checks newest history, not coverage at original measurement source | Probe shows freshness true with a 50 ms-old source and first history record now; lookup at source fails. This is not an actual enable-service response test |
| Design difference requiring explicit acceptance | Legacy `runtime.py:416` recovers after two missed solve periods; native at `node.cpp:368` waits for source TTL/horizon expiry | At 20 Hz, approximately 100 ms versus potentially 800 ms. The approved plan's hold-until-expiry wording supports the new policy, but it is not unchanged legacy recovery; test and document it |
| P2, native launch parameters | Launch horizon does not select/validate bundle horizon, TTL still assumes .1 s, native timeout is hardcoded .05 | N25/.05 + `horizon:=25` derives TTL2s and refuses startup; another horizon can silently run bundle N10. Resolve effective parameters and test actual launch |
| P2, solver option semantics | Native ignores `acados_rti_steps` and conditionally runs one/two passes | Reject/document the irrelevant option or map it to an explicit native policy; do not claim whole config behavior preserved |
| P2, diagnostics | Authority, live state/plan ages, cross-track/corridor and effective timing fields are missing | Restore operator-visible meanings; `observation_age_s` is last-request age, not live state age |
| Availability difference | Native thread has no legacy process restart/preemption after hung solve, and destruction joins worker | A measured budget does not interrupt a stuck native call. Verify bounded failure/shutdown separately |

## Deliberate algorithm changes and inherited issues

The six-state model and normalization coefficients were retained. That does
not make the full optimizer equivalent:

- IPOPT evaluates actual quintic geometry at the variable progress. Native
  freezes position tangent, heading and curvature at seed progress; RTI pass 2
  reuses those parameters. This is the selected NPU-style approximation.
- Fractional warm-start interpolation replaces legacy's minimum whole-stage
  shift. Gauss–Newton/RTI and converged IPOPT produce different controls.
- The native acados envelope reserve can be 0.01 while legacy IPOPT's default
  optimization margin is zero; physical post-check limits remain unchanged.
- The virtual progress parameter uses chord-distance spline parameterization,
  not exact arc length. Both versions compare its speed to physical m/s targets
  in the progress cost. Tangent-corrected preview/seed do not eliminate this
  inherited unit approximation. Dense-path behavior and sparse references must
  be distinguished.
- Longitudinal ideal acceleration and the lumped steering response remain
  inherited approximations, not identified speed-PID/pure-delay dynamics.

`audit_objective.py` evaluates **actual legacy `MPCCSolver.op.f`** and compiled
native stage/terminal functions with matching weights and controls. For an R2
reference, seed progress .8 and candidate progress 1.3 exactly on the actual
quintic, legacy cost is approximately zero, frozen cost is 1615.548; frozen
contour error is .0621893 m and heading difference .250029 rad. At no progress
departure both costs agree. These arbitrary evaluation points are not feasible
trajectories or evidence of observed tracking degradation. They establish that
the two objective functions differ and quantify the approximation.

## Test and evidence weaknesses

1. Export parity data calls the same symbolic model/constraint generator later
   compiled into C. This verifies generated-code/ABI consistency, not an
   independent legacy oracle. Existing independent numerical loops are useful
   but do not replace fixed-input actual-execution comparisons.
2. `test_core.cpp:10` uses `abs(a-b)>tolerance`; NaN passes that test helper.
   Runtime finite guards are separate and remain present. The helper must
   explicitly reject nonfinite comparisons.
3. Model parity has only two example states/controls and zero alignment;
   centerline projection cases omit off-path/nearest-branch boundary cases.
4. Nonnominal acceptance at `tools/acceptance.py:185` requires injection and a
   final zero converter target, without required prior positive RUNNING,
   specific FAULT/reason or response deadline. It can pass vacuously. Separate
   protocol probes are stronger but use synthetic selector echoes.
5. Clock probe uses a tiny backward stamp followed by restored wall stamps;
   it does not cover a true reset and reenable. Health probes continuously
   stream odometry and miss epoch-change-before-next-odom enable.
6. Existing launch tests inspect declarations/selection rather than resolved
   native parameters. Startup READY proves artifact loading, not execution
   contract coverage or low-speed closed-loop readiness.
7. The matched NX legacy tracking comparator still has no completed run. The
   failed 5 Hz comparison additionally has an incomplete replay drain audit.
   Its recovery-bound failure cannot be attributed solely to slow optimization.

## Reproduction and evidence

Desktop container: `aimsracer-mpcc-acados-runtime`; production sources and
native ROS binary remain unchanged. No NX builds/tests or physical driving
were performed in this audit. All ROS processes use localhost-only isolated
domains/topics. Audit native callbacks use the existing test friend and no
executor spins; the pending starvation run uses real private RC/converter nodes.

Inside that container at `/workspace`, source its existing ROS overlay, then:

```bash
export ROS_DOMAIN_ID=228 ROS_LOCALHOST_ONLY=1
python3 src/aims_mpcc_rt/tools/audit_contracts.py \
  --build-dir build/aims_mpcc_rt \
  --synthetic-bundle /absolute/path/to/final-r2-v10 \
  --measured-bundle /absolute/path/to/final-field-n10 \
  --output /absolute/path/to/new-audit-directory
python3 src/aims_mpcc_rt/tools/audit_objective.py \
  --bundle /absolute/path/to/final-r2-v10 \
  --output /absolute/path/to/objective.json
```

Both tools explicitly restrict the comparison to strict N10/.1 bundles.
For a container with an unmounted worktree gitdir, supply the host-verified
`--runtime-commit`; the report labels it as supplied and separately hashes the
actual sources, linked Core library, audit executable and bundle metadata.
Exit 0 from these tools means the counterexamples reproduced, **not** that
the runtime passed acceptance.

Evidence root:
`/home/elesheep/aimsracer-data/experiments/mpcc-acados-runtime/audit-20261009`.

- `repro-final/report.json`, `native.log`, `build.log`: final native/legacy
  comparison, bundle configuration/provider metadata and source/library hashes.
- `objective.json`: actual objective evaluations.
- `pending-lead100/report.json`, controller CSV and private routing logs:
  pending starvation reproduction.
Evidence archive: `audit-20261009-evidence.tgz`, SHA256
`b2b6743f76a551833fc2ffad6df7edfa3bbff982a94468dd58c91cc8d44318db`.

- Earlier `repro-v1/v2/v3` are investigation attempts. The early timing probe
  lacked a pre-hold delivery assertion; only `repro-final` is final contention
  evidence. Earlier cadence isolation also bypassed startup validation.

## Repair and revalidation order

1. Establish the actual executed trace certificate/control semantics first;
   retain independent oracle cases, including nonzero applied prefixes.
2. Include delivery wait in deadline decisions and stage metrics; check the
   deterministic contention case and all disposition accounting.
3. Repair epoch/clock/history recovery and pending activation ownership; test
   enable and failure interleavings rather than only final zeros.
4. Resolve conditional corridor support, launch parameters, diagnostic contracts
   and explicit recovery/solver policy differences.
5. Run independent model/objective/output boundary tests and then repeat the
   frozen-version NX joint-load matrix with corrected instrumentation. Physical
   field release follows those checks; the 0.5 m/s production bundle
   preparation remains outstanding.
