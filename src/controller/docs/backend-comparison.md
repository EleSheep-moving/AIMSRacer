# Experimental native solver backends

`ipopt` remains the default. Explicit `acados` and `qp` selections produce
numerical candidates through the same `reset()` / `solve()` interface. Every
result reports `execution_authorized: false`. A successful optimizer result is
checked by `validation.validate_candidate`, then the parent validates the actual
handover state and current command epoch before accepting a plan.

| Property | IPOPT | acados | QP |
|---|---|---|---|
| Numerical core | CasADi/IPOPT/MUMPS | Generated C, SQP_RTI, HPIPM | OSQP native C extension |
| Physical prediction | Six state nonlinear rear axle model | Same six state discrete model plus three previous input states; returned states reconstructed from unchanged native controls | Four state linear lateral/speed prediction; six state nonlinear output rollout |
| Reference | Periodic spline inside the nonlinear program | Frozen local spline geometry per interval, refreshed each request | Frozen speed/curvature per interval, refreshed each request |
| Steering actuator | 20 ms endpoint ramp, first order lag | Same ramp/lag and RK4 discretization | Same ramp/lag in the linear predictor; complete nonlinear independent output rollout |
| Longitudinal/lateral coupling | Nonlinear | Nonlinear | Coupling approximated using seeded speed; exact candidate checked afterward |
| Hard limits | Speed, acceleration, steering endpoint, jerk, steering rate/acceleration | Same | Same, with conservative actual steering bounds in predictor |
| Corridor | Nonlinear footprint at integration samples | Frozen tangent footprint at start/middle/end; current spline checked independently | Linear footprint approximation; complete independent rollout checks current spline |
| Strict envelope | Nonlinear ellipse | Nonlinear ellipse at 20 ms nodes with explicit RTI reserve | Inscribed 16-sided conservative ellipse approximation plus exact independent checks |
| Soft envelope | Optional bounded recovery slack | Same finite utilization cap and recovery deadline; independent recovery comparator | Explicit `unsupported` result; no slack approximation is silently substituted |
| Warm start | Last successful candidate | Shifted last successful controls, re-integrated from new measured state | Explicit elapsed shift of last successful controls; all primal nodes projected through current alignment; duals reset |

The acados cost retains the baseline normalized contour, lag, periodic heading,
speed, progress, steering feedforward, acceleration, steering rate, steering
acceleration and terminal weights. By default one SQP_RTI call is made per request. An RTI
step can return native status zero while nonlinear constraints remain violated;
`success` additionally requires the returned candidate to obey the physical
bounds with violation below `1e-4`. The stage zero nonlinear rows are supplied
explicitly, since acados does not inherit interior path rows at stage zero.
A single RTI step can leave a nonlinear dynamics defect in its optimized states;
returned states are reconstructed through the backend's own nonlinear RK4
transition using the unchanged native controls. Raw optimizer defects and
constraints remain in diagnostics, and every physical inequality is checked again
on that reconstruction. The separate midpoint validator remains independent. Rejected candidates do not overwrite the last successful seed; successful warm
state caches contain the forward reconstruction.

### Bounded second RTI pass experiment

`VehicleConfig.acados_rti_steps` defaults to `1` and accepts only the integers
`1` and `2`. The explicit value `2` runs exactly two complete preparation and
feedback calls on the same OCP parameters and native iterate, without shifting
or re-seeding between them. This is an experiment with an extra SQP step per
request. The [official acados RTI loop example](https://github.com/acados/acados/blob/7e1d1152c1babd6ea04af1c9d73444fe8381057b/examples/acados_python/pendulum_on_cart/ocp/example_sqp_rti_loop.py)
demonstrates repeated full RTI steps for an unchanged OCP.

The final native status and final physical candidate determine backend success;
both passes execute even if the first returns a nonzero status. The independent
candidate validator and handover validator still apply. No native controls,
physical bounds, acceptance tolerances, envelope slack caps or reserve margins
are changed by this option. Its value participates in the artifact fingerprint,
so changing it requires offline preparation. Diagnostics retain each pass's
status, native time, wall time, SQP iteration count and recomputed nonlinear
residuals; combined native time and iterations include both passes. A rejected
backend candidate also retains a rare `failure_snapshot` containing native
states and controls, projected states, frozen parameters and physical violations.
The worker writes that snapshot to disk and omits it from the parent reply.

Recorded request replay from `forecast-internal-acados-n10-v10` on source
`38dae97` reproduced all 292 original one-pass results exactly. One pass accepted
268 candidates and rejected 24; two full passes accepted all 292 under the same
physical bounds and independent validator. At request 240 the one-pass peak
utilization was 1.0068368 at the second interval's starting acceleration change;
the second pass removed this excess. Request 239's initial utilization decreased
from 0.9982138 to 0.9900207, restoring the configured reserve. These results came
from an experiment wrapper before this option was introduced; its full artifact
is `acados-fixed-pass-request-replay-38dae97.json` in the external experiment
directory. The checked-in request-240 fixture tests the unchanged failure and
the explicit two-pass option with the real native solver.

This replay holds measured states and request references fixed. It does not
establish handover acceptance, closed-loop completion, NX timing or hardware
readiness. The original controller log lacks the rejected rebase trajectory and
detailed rebase validation verdict, so the precise handover failure at request
239 still requires a fresh diagnostic closed-loop run.

The QP uses separate lateral and speed objectives. Progress is derived from
bounded longitudinal speed and the local tangent norm; it has no independent lag
objective or progress decision. Asymmetric longitudinal envelope axes use their
minimum for a conservative convex approximation. The ellipse linearization uses
seeded speed and steering, so passing OSQP alone cannot establish physical
feasibility. Exact actuator bounds, actual nonlinear utilization, and actual
footprint are checked before the backend reports success. The shared independent
validator checks these conditions again.

## Preparing acados

Install acados and `acados_template` outside the runtime worker and set
`ACADOS_SOURCE_DIR` to the source installation. The implementation recognizes
an `install-x86/{include,lib}` installation or standard source `{include,lib}`
paths. The loader must be able to resolve `libacados`, `libhpipm`, and `libblasfeo`;
set `LD_LIBRARY_PATH` to the installation's `lib` directory when required.

Call `create_solver('acados', path, config, prepare=True,
artifact_directory=directory)` during offline preparation. Generated C and its
shared library live under `directory/acados/<fingerprint>/`. The optional
`AIMS_MPCC_SOLVER_DIR` supplies the root when no explicit directory is provided.
The fingerprint contains backend source hashes, vehicle configuration, horizon,
discretization, path geometry and map identity, platform/Python ABI, CasADi/NumPy
versions, acados source revision, and native dependency library hashes. The
manifest records SHA-256 hashes of both the generated library and `ocp.json`.
Loading also checks the JSON's controlled library routes, model name, state/control
dimensions, horizon, discretization and solver types against the requested artifact.
The native constructor receives a private copy of the verified JSON bytes, so it
does not reopen mutable cache metadata after verification.

Online construction uses `generate=False, build=False`. Missing, changed, or
corrupt artifacts fail with an instruction to prepare offline. Online construction
never triggers compilation. Changing configuration or the reference requires
preparation of the new fingerprint. No ROS node or hardware publisher is needed
to prepare or test a backend.

## Verified numerical contract

`tests/test_backend_contract.py` exercises both real native cores on a synthetic
3 m circle, independently checks their returned trajectories, checks a rotated
map reference against an equivalent odom reference, and checks that disabling
the corridor reaches both optimizer and independent validation. It also checks
acados's missing artifact/configuration invalidation behavior, finite soft cap
and deadline semantics, and QP's explicit unsupported soft recovery result.
Two tamper regression variants reject changed acados JSON before native loading or compilation, including changed loader routes with an updated JSON digest. Two further QP tests verify that failed request age accumulates in the native primal shift and that a new map alignment reprojects every native node. Acados tests skip only if `acados_template` is unavailable; installing it without
its required native libraries produces a failure.

The measured dependency snapshot was acados v0.5.3 commit
`7e1d1152`, CasADi 3.7.2 and OSQP 1.0.4 in
`aimsracer-mpcc-optimization` on x86_64. This establishes numerical candidates in
that environment; it does not establish Jetson NX performance, ROS worker timing,
tracking quality, closed loop acceptance, or vehicle readiness.

## Synchronous timing sample

Twenty repeated requests used a synthetic 3 m circle, speed 0.5 m/s, N=10,
dt=0.1 s and an enabled corridor. Both backends returned 20 candidates accepted
by the independent validator. Preparation/compilation was excluded from online
timing. Assembly, native calls, extraction and diagnostics were included.

| Median elapsed time (ms) | acados | QP |
|---|---:|---:|
| Full numerical solver call | 12.32 | 21.01 |
| Input, geometry and assembly | 4.97 | 8.60 |
| Native optimization call | 0.19 | 0.17 |
| Extraction and numerical diagnostics | 7.16 | 12.25 |
| Additional shared independent validation | 9.17 | 9.09 |
| Full solver p95 | 12.73 | 21.24 |

Native optimization time alone omits most request work. These are synchronous
x86_64 calls; IPC, parent validation and actual handover checks add work. The raw
sample and scope are saved at
`/evidence/experiments/mpcc-nx-optimization/backend-contract-timing.json` in the
test container's evidence mount. They are not a comparison against IPOPT or proof
of a 20 Hz closed loop.


## Prepared independent native rollout measurement

A separate offline prepared C kernel now accelerates the independent 2 ms
midpoint rollout. It does not use the optimizer's 20 ms RK4 transition. Online
loading never compiles this kernel, and the Python numerical reference remains
available for equivalence tests. QP now explicitly initializes the native primal
every request: retained successful controls shift by their cumulative actual
request age, nonlinear seed states start from the new measured odom state, and
all lateral/heading nodes project through the current alignment. Native duals
reset to zero because changed geometry and initial bound rows do not have a
reliable multiplier mapping. Failed requests retain the last successful seed and
add to its age.

The same twenty-request circle sample was rerun after preparing the independent
kernel, applying the QP warm-start repair, and updating the shared validator to
its current implementation. Both backends again returned 20/20
independently accepted candidates. The initial Python measurements above remain
in their original file. The following table reports the second complete call
measurement, rather than inferring total performance from native core time.

| Median elapsed time (ms) | acados with native rollout | QP with native rollout |
|---|---:|---:|
| Full numerical solver call | 6.44 | 13.86 |
| Input, geometry and assembly | 3.28 | 7.72 |
| Native optimization call | 0.18 | 0.17 |
| Extraction and numerical diagnostics | 2.96 | 5.93 |
| Additional shared independent validation | 1.94 | 1.96 |
| Full solver p95 | 6.98 | 14.37 |

Across these two x86_64 samples, median full solver calls measured 6.44–12.32 ms
for acados and 13.86–21.01 ms for QP. Additional shared validation measured
1.94–9.17 ms and 1.96–9.09 ms respectively. The second QP measurement includes
its new explicit primal initialization and the current shared validator, so
these samples do not isolate the rollout kernel's contribution alone. Raw samples,
source and kernel SHA-256 hashes, and acados artifact fingerprint are saved separately in
`/evidence/experiments/mpcc-nx-optimization/backend-contract-timing-native-rollout.json`.
Both samples exclude ROS, IPC and actual handover work, and establish no NX or
20 Hz closed loop result.


## First interval constraints and RTI envelope reserve

The generated OCP explicitly sets `con_h_expr_0`, `lh_0`, and `uh_0`. Without
these fields, acados v0.5.3 leaves `nh_0=0`, so the first input would omit jerk,
steering slew/acceleration and the nonlinear sample constraints. The artifact
loader also verifies stage zero/interior/terminal nonlinear dimensions.

`VehicleConfig.acados_envelope_margin` defaults to **0.01**, is dimensionless,
and permits `0 <= margin < 1`. Acados uses `max(acados_envelope_margin, optimization_envelope_margin)` and
optimizes `E - slack <= 1 - effective_margin` to
reserve room for one RTI step's ellipse linearization error. Its physical
candidate gate and the independent validator still enforce the original
`E - slack <= 1`. Soft slack cap, recovery deadline and recovery comparison retain
their original values. IPOPT and QP do not use the acados-specific reserve. The margin is
included in configuration and the artifact fingerprint and reported in diagnostics.

Cold acceleration regressions use N=10/15/20 at a 0.5 m/s reference and N=10
at a 1.0 m/s reference. They verify the actual native first input obeys the
previous input's jerk and steering acceleration constraints, and that returned
forward states pass independent physical validation. A native output corruption
test verifies that forward reconstruction does not repair or authorize unsafe
controls. Corridor rejection and native failure remain separate hard gates.


`optimization_envelope_margin` is an optional common optimization reserve,
defaulting to **0** to preserve the original IPOPT/QP profiles. It has the same
finite range `0 <= margin < 1`. IPOPT optimizes `E - slack <= 1 - margin`, including
the last integration node. QP contracts its conservative polygon by
`sqrt(1-margin)`. Acados uses the maximum of this common reserve and its own
reserve. Configuration, diagnostics and fingerprints record the selected values.
The physical validator still uses `E <= 1` and the original recovery contract.
A reserve cannot guarantee that an arbitrary RTI step is physically feasible:
an abrupt moving 1.0 to 1.5 m/s request with a 0.02 reserve produced an envelope
violation and was rejected by the unchanged gate.

`python3 -m aims_mpcc.worker_benchmark` measures submission, IPC, worker solve and
validation, delivery, and caller validation with every failed/late reply retained.
Its separate `--prepare-only` mode records native warmup status and compiles only
offline. `--sample-lap` generates nominal reference-following states using the
spline parameter derivative; it is not recorded motion or closed loop evidence.
The default minimal polling interval does not reproduce the ROS node's 50 Hz
callback quantization or Supervisor handover. CPU counters use two process reads
outside request timing, exclude startup, and report unknown counters explicitly.
