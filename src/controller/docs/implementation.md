# MPCC implementation guide

Read [system architecture](../../../docs/architecture.md) for frame and command
conventions, and [usage](usage.md) for running the package. The current reference
is a closed lap loaded from disk, not a streamed local trajectory. The
[current runtime timing table](usage.md#runtime-timing-parameters) defines active
defaults, units and time origins. Measurements below keep their historical
experiment settings; those settings are not current launch recommendations.

## Read one control cycle

| File | Responsibility |
| --- | --- |
| [node.py](../aims_mpcc/node.py) | ROS inputs, timers, service and telemetry |
| [runtime.py](../aims_mpcc/runtime.py) | Supervisor, state/plan freshness, command selection |
| [history.py](../aims_mpcc/history.py) | Forwarded inputs, steering estimate and state prediction to takeover |
| [worker.py](../aims_mpcc/worker.py) | Separate solver process, readiness, requests and deadlines |
| [solver.py](../aims_mpcc/solver.py) | Optimization problem, initial guesses and solve results |
| [path.py](../aims_mpcc/path.py) | Reference loading, projection and curve evaluation |
| [frames.py](../aims_mpcc/frames.py) | Planar map-to-odom alignment for map references |
| [config.py](../aims_mpcc/config.py) | Vehicle parameters and validation |

Start with the node's subscriptions and request construction, follow a request
through the worker into `MPCCSolver.solve()`, then follow the result back through
the supervisor. A plan contains future controls; one proposed command is
published each tick and the RC selector decides whether to forward it. Command publication and fresh optimization have
different rates.

## Optimization variables

Each predicted state has six components:

```text
X = [x, y, yaw, speed, progress, actual steering]
U = [longitudinal acceleration, steering command, virtual progress speed]
```

Default horizon is 10 intervals of 0.1 s. The startup `horizon` parameter reaches
the worker, state/control dimensions and the supervisor's speed-reference array;
`prepare_solver --horizon` must match it. `/mpcc/status` reports the active value.
It is read-only after launch because changing it requires rebuilding the solver.
`_dynamics()` uses RK4 substeps at
20 ms with a kinematic model, steering lag and an understeer correction.
`x`, `y` and `yaw` always describe the vehicle in `odom`, including for a
persistently stored `map` reference. A fresh map-to-odom alignment is a solver
parameter; `geometry()` applies it to predicted states solely when evaluating
the map spline, contouring/lag errors and footprint corridor.
`_build()` creates `X` and `U`, constrains transitions and initial state, and adds
speed, acceleration, steering, jerk, steering-rate/acceleration, acceleration
utilization and footprint corridor constraints.
The real vehicle footprint is asymmetric about the rear-axle state: the solver
checks the four corners at +0.52 m and -0.10 m longitudinally and ±0.16 m
laterally. The current 1.0 m course model gives 0.5 m to either side of the
centered reference; this is a local corridor constraint, not an obstacle map.

The objective combines contouring/lag error, heading, reference speed, progress
speed, steering and control smoothness. Trace `geometry()` and the vendor cost
modules when studying normalization. The vehicle model and actuator parameters
include assumptions; satisfying the numerical constraints does not establish
real tire or actuator behavior.

## Reference smoothness

`ReferencePath` uses a periodic quintic interpolant through the stored path
points. Position and its first four derivatives are continuous at interior
knots and the lap seam. For a nonzero tangent, curvature therefore has two
continuous derivatives, as required by its use in the steering feedforward
cost. NumPy projection/visualization and CasADi optimization evaluate the same
piecewise polynomial coefficients; the controller does not freeze curvature
during optimization.

The earlier periodic cubic kept curvature continuous, but its curvature
derivative could jump at a knot. In an isolated 2026-10-03 replay, one such
request stalled at IPOPT's 100-iteration limit even though the final candidate
satisfied the constraints. Quintic interpolation converged in seven iterations
with the same limits and optimizer tolerances. The sampled centerline difference
on the current recording was below 0.01 mm. This addresses that numerical
failure, not vehicle tracking acceptance or a guarantee of convergence for
every request. Existing CSV bundles load through the quintic automatically;
their points and map identity are unchanged. Changing the interpolation changes
the native callbacks, so prepare the matching solver cache once before launch.

After preparing the horizon-8 `-O2` cache, ten cold-primal replays of that same
request, after native startup warm-up, all converged in seven iterations.
Complete `solve()` P50/P95/max was 31.05/32.04/32.22 ms; maximum constraint
violation was about 1.6e-13 and predicted corridor margin about 0.335 m. These
numbers describe one formerly failing request, not a whole-course latency
distribution. The package build and 16 checks passed, covering interpolation,
lap-seam continuity, curvature-cost derivatives, reference geometry and the
existing map/native-cache contracts. Diagnostic checks and artifacts are kept
outside the repository.

The final quintic runtime also completed one lap of the saved course with the
real asynchronous cached worker and an independent synthetic plant at a
0.6 m/s target. It took 60.36 s, returned 302 successful results and activated
301 plans, with no faults or handover rejections. Complete solve P50/P95/max
was 23.81/38.96/62.16 ms, parent-observed request-to-reply P95 was 59.92 ms,
and maximum simulated cross-track error was 3.47 mm. This exercised the then-current 5 Hz
scheduled-takeover runtime with the 500 ms deadline and 750 ms source-age
budget; it is a kinematic software check, not tire-model or physical-vehicle
validation. No ROS actuator commands were published by this test.

In the historical 10 Hz / 200 ms configuration, a second
0.6 m/s independent-plant run completed one lap in 60.86 s. The cached worker
returned 608 results and activated 607 plans with no faults or rejections.
Complete solve P50/P95/max was 24.88/40.87/64.14 ms; request-to-parent-reply
P95/max was 60.00/80.00 ms. Median request spacing was 100.00 ms and command
spacing 20.00 ms; maximum simulated cross-track error was 5.89 mm. Takeover
prediction then spanned 100 ms, while that experiment's original-source plan
lifetime was 750 ms. Those runtime changes reused the quintic cache and preserved the same
physical constraints and convergence checks. Physical vehicle validation is
still separate from these software measurements.

The subsequent horizon-15 comparison used the same quintic reference, 0.1 s
decision spacing, 10 Hz scheduling and 200 ms request deadline. At ten course
positions and speeds 0, 0.6 and 1.0 m/s, with two fixed-state requests per pair,
both horizons returned 60 successful results. Horizon-8 complete solve
P50/P95/max was 43.85/49.90/51.32 ms; horizon-15 was 78.02/89.16/148.83 ms,
with one solve above 100 ms and none above 200 ms. This fixed-state test includes
cold primal starts and does not describe the moving run's latency distribution.

With horizon 15, independent moving-plant runs completed one lap at both
0.6 and 1.0 m/s targets. The former returned 607 results with complete solve
P50/P95/max 38.71/69.33/93.40 ms and maximum simulated cross-track error
4.69 mm. The latter returned 404 results with 48.11/89.57/110.39 ms and
20.84 mm cross-track maximum. Neither run faulted or rejected a plan. At 1.0 m/s,
parent-observed reply P95/max was 100.00/120.00 ms and maximum handover lateness
20.00 ms; scheduled and actual takeover checks remained enabled. These software
results supported the temporary horizon-15 default, extending look-ahead to 1.5 s;
they do not guarantee every result arrives inside 100 ms or validate physical
vehicle behavior. The matching horizon-15 native cache has been prepared;
changing frequency/deadline does not require a further compilation.

That historical next-trial configuration selected horizon 10 (1.0 s), with
10 Hz optimization, 100 ms scheduled takeover, a 200 ms request deadline and
50 Hz command output. It has since been superseded by the
[current runtime timing settings](usage.md#runtime-timing-parameters).
The horizon-15 results above remain simulation evidence;
they do not establish field performance for the current horizon-10 setup.
Physical startup and localization checks follow the known-map workflow.
The matching horizon-10 cache is prepared. An isolated default worker loaded
that cache, reached READY in 4.24 s and replayed the archived request successfully
in seven iterations (31.03 ms solve, 35.89 ms parent-observed reply). This verifies
startup/configuration consistency without publishing to the vehicle; it is not
a live field test.

## Warm start

After a successful solve, the solver saves the optimized controls. On the next
request, `_warm_start()` shifts them by `round(elapsed / dt)` (clamped to at least
one step and at most the horizon), repeating the last control at the tail.
It limits acceleration/steering changes relative to the applied command,
recomputes virtual progress speed, and rolls the model forward from the new
predicted takeover state. `set_initial()` receives those rebuilt states and controls.

The previous state array is not copied as the new initial trajectory. No previous
Lagrange multipliers are supplied. Failed solves and worker generation changes
clear the stored warm start; the fallback uses reference speed and curvature.
`warm_start_init_point=yes` is configured in IPOPT, but this implementation reuses
primal variables only.

## Measurement time, computation and takeover

The runtime defaults are 5 Hz optimization and 50 Hz command publication.
Each request has three monotonic epochs, distinct from the ROS timestamp used
for measurement ordering:

```text
source_stamp          submitted_at                  stamp (takeover)
EKF measurement ---- actual-input replay ---- now ---- 200 ms bridge ---- new plan
                                                  old plan or manual input
```

`history.predict()` first replays real selector outputs from the measurement
epoch to submission. In autonomous mode a private copy of the supervisor
forecasts the old plan, including the command smoothing, until takeover at
submission + one solve period (200 ms at the default 5 Hz). In manual mode it holds the last actual speed/steering
target because future operator input is unknown. These forecasts never enter
applied history. The bridge includes kinematic pose propagation and the existing
first-order steering lag. Its longitudinal approximation follows the actual
speed target using the configured acceleration/deceleration bounds; those
bounds are not an identified motor transient model.

The NLP initial state and previous-input boundary are at takeover. An early
result is staged, while the original controller continues publishing the old
plan. On the control tick at takeover (with 0.5 ms tolerance for timer jitter), the latest EKF state is
advanced to that tick with actual input history and checked against the
candidate's expected state. MPCC prediction mismatch limits are 0.30 m
position, 30 degrees yaw, 0.30 m/s speed and 20 degrees estimated steering. Actual
speed and steering target changes are limited to 0.30 m/s and 20 degrees at this
boundary. Candidate rejection preserves the old plan only within its original
lifetime. A future bridge may read old controls past TTL while inside the
prediction horizon, but real command output enforces the original plan expiry.

Localization-health policy belongs to the external localization provider.
MPCC has no map-valid subscription and no TF-age or localization-correction
size gate. It checks coordinate availability and reference map identity, and
records TF age/correction changes as telemetry. Map alignment is projected to
x/y/yaw without rejecting roll/pitch tilt. Measured speed is retained without
a reverse/upper operating-range fault; the solver entry also has no negative
measured-speed rejection. The existing forward-only OCP uses a nonnegative
initial speed, while supervisor telemetry and takeover checks retain the signed
measurement. Nonfinite-state checks and command speed/steering constraints
remain enforced.

New-plan interpolation uses `now - stamp`, where `stamp` is the actual activation
tick. A late tick still starts with the first new control; it never skips
unexecuted inputs. `scheduled_stamp` preserves the predicted takeover epoch,
and `handover_lateness` reports their difference. The state check compares the
actual activation state with the predicted initial state, covering that short
timing difference within the experimental mismatch limits. The
250 ms request-to-parent-reply acceptance budget discards late results without
killing the worker or faulting the run. Early replies still wait for takeover; eligible replies
need the same latest-state/target validation and begin at actual activation.
They can be rejected if the predicted initial state no longer matches. Plan
lifetime defaults to `horizon * 0.8 * 0.1 s` from `source_stamp` (800 ms for
horizon 10; 1.2 s for horizon 15), so a future initial state cannot refresh
stale data. Both direct node startup and launch derive the default from horizon;
an explicit startup `plan_ttl` overrides it. At 5 Hz, a 200 ms preceding update
period + 250 ms acceptance budget + 20 ms source age leaves about 330 ms in
the default horizon-10 lifetime budget. A skipped
solve finishes in the existing process and its reply is drained before a fresh request;
IPOPT is independently capped at 30 iterations. The old plan continues
through its existing prediction; it never repeats the last control after
prediction coverage ends. Only one solve is in flight, so a long solve reduces
the actual update rate while command publication continues at 50 Hz.

Prediction to a future control epoch is a computation-delay compensation idea
described in [the advanced-step NMPC research](https://www.sciencedirect.com/science/article/pii/S0005109808004196).
This implementation uses a scheduled bridge and mismatch guard; it does not
implement that paper's sensitivity corrections or inherit its stability results.
Manual input, RC authority changes, unmodeled motor behavior and localization
error can all invalidate the forecast. Physical command-to-actuator delays are
separate from the computation delay; the computation budget is not folded into
`steering_tau`.

`/mpcc/status` preserves source age separately from execution phase and reports
the solver stages through `request_timing`, staged-plan countdown, latest
handover errors and rejection count. Frequency/lifetime changes affect the
runtime only, not the CasADi graph or native cache key. Horizon changes still
require a matching prepared cache.

**Historical validation configurations (2026-10-03):** the 500 ms / 750 ms
and earlier budgets below describe those tests, not current defaults.
Before that deadline adjustment, software checks covered scheduled/late takeover, changing
actual inputs, stale/missing history, original source expiry and timer jitter.
With the real cached solver and an isolated stationary input, a 20 s run returned
100 results and activated 99 plans (the final result was still staged), with no
faults; request spacing median was 200 ms and command spacing median was 20 ms.
This is not a live ROS/vehicle acceptance test. An independent synthetic plant
at 0.6 m/s ran for about 40 s before a solver deadline fault. Its exact last
request, replayed after cache warm-up, reached IPOPT's existing 100-iteration
limit in about 376 ms. That convergence problem is distinct from plan age and
does not disappear with a 200 ms deadline. Earlier successful simulations used
a different takeover rule and do not validate the final runtime. Physical
constraints and optimizer options were not weakened; moving-car operation
remains unvalidated.

With the subsequent historical 500 ms deadline and 750 ms source-age budget, 26 software
checks passed, including accepting a 375 ms reply, enforcing the 500 ms timeout,
and stopping at prediction exhaustion. A real cached-worker replay of the same
failed request returned in 445.6 ms (432.6 ms inside `solve()`), without killing
the worker. It still reported `Maximum_Iterations_Exceeded`; increasing the
time budget does not make an unconverged result eligible for execution. Those
failure replays used the former cubic reference; see Reference smoothness above
for the subsequent quintic correction.

## Native compilation and timing

[native.py](../aims_mpcc/native.py) configures `-O2` and persistent ccache storage.
[prepare_solver.py](../aims_mpcc/prepare_solver.py) prepares callbacks offline;
workers require cache hits and still construct the symbolic problem, link and
warm up at startup. Cache commands and invalidation rules are maintained in
[usage](usage.md#prepare-the-compiled-solver).

`solve_time_s` measures the complete `solve()` call, including path projection,
parameter setup, warm-start construction, IPOPT and result extraction (including
the predicted corridor margin). It is not solely IPOPT internal execution time.
ROS delivery and actuator communication require separate end-to-end measurements.
The historical timing comparisons below predate the quintic correction;
the replay measurement in Reference smoothness uses the new callbacks.
Earlier isolated Orin comparisons measured steady solve P50/P95 of about
43.9/76.3 ms with `-O2`, versus 67/142 ms with `-O0`. These are historical
workload-specific observations, not a full moving-vehicle latency budget.
For the current 2026-09-28 map and 620 × 320 mm body (rear axle 100 mm from the
tail), a cache-hit offline profile at 0.5 m/s used 30 repeated solves after one
tracking warm-up, alternating the map alignment by 5 mm. Median/P95 stage times
were: projection 1.36/1.44 ms, parameter setup 0.32/0.35 ms, warm start
15.01/15.38 ms, initial-value setup 0.18/0.22 ms, `op.solve()` 59.94/60.92 ms,
and result extraction 13.75/14.30 ms. Predicted-margin evaluation accounts for
11.69/12.22 ms of extraction. Complete `solve_time_s` was 91.97/92.90 ms;
worker request-to-reply was 93.41/94.49 ms, with 1 ms polling in this profile.
Worker startup to readiness took about 4.6 s. These repeated fixed-state solves
do not measure moving-car tracking or the full ROS/actuator latency. The
[vehicle checklist](../../../docs/operations/vehicle-checklist.md) tracks
alignment and model work still required before faster operation.

On 2026-10-03, with V2, known-map localization and RViz running, 30 repeated
requests used the stationary car's measured starting pose (about 8 cm lateral
error), zero previous commands and 1.0 m/s speed references. Complete solve
P50/P95/max with thread settings 1, 2 and 4 were respectively
115.2/117.4/119.7, 117.5/120.7/121.7 and 116.3/119.5/120.2 ms. CPU-time P50
was 114.4, 117.1 and 115.8 ms. The bundled CasADi OpenBLAS identifies itself as
`ARMV8 SINGLE_THREADED`, reports one thread and `openblas_get_parallel() == 0`
under every setting; raising those environment variables does not parallelize
this backend. A three-second sample during the two-thread comparison showed
61.8% aggregate utilization over eight CPU cores; FAST-LIO used about 44% of
one core. Scheduling contention can still cause occasional larger latencies,
but computation alone exceeds 100 ms in this workload.

Using the same measured starting state and one thread, reducing the horizon
gave the following complete-solve timings (30 requests, native cache already
loaded; no compiler processes active):

| Intervals | Look-ahead | P50 | P95 | Maximum |
| --- | --- | --- | --- | --- |
| 15 | 1.5 s | 115.2 ms | 117.4 ms | 119.7 ms |
| 12 | 1.2 s | 84.9 ms | 87.3 ms | 90.8 ms |
| 10 | 1.0 s | 84.0 ms | 86.1 ms | 86.1 ms |

A separate comparison sampled ten positions around the saved loop, each at
0, 0.5 and 1.0 m/s with two fixed-state requests after resetting at each
position/speed pair (60 solves per horizon). Initial yaw and steering followed
the centerline tangent and curvature; previous acceleration/ramp rate were
zero. All solves succeeded. For 12 intervals, P50/P95/max were
84.0/109.2/143.7 ms; for 10 intervals, 67.6/76.9/109.1 ms. The latter maximum
occurred at approximately 0.925 m⁻¹ curvature and 1.0 m/s, with 22 IPOPT
iterations. These are optimization checks at sampled states, not a moving-car
closed-loop test or a guarantee that every solve stays below 100 ms.

The earlier low-speed evaluation used 10 intervals. The current default is
10 intervals with 5 Hz optimization; the 0.1 s decision spacing and physical
actuator constraints remain unchanged. The current complete-lap experiment
sets `enforce_corridor: false`, omitting the footprint track constraints and
measured/start boundary gates while keeping cross-track penalties and telemetry.
The online reply acceptance budget is now 250 ms;
late replies are discarded without worker termination or a timeout fault, and
plan lifetime defaults to 80% of the prediction duration from the original
measurement (800 ms with horizon 10), as detailed above.
Recorded map-TF stalls are handled by the independently
checked, bounded correction policy in the
[known-map workflow](../../../docs/operations/known-map-mpcc.md).

The former evaluation implementation mixed stationary measured speed with
hypothetical applied braking history. Exact-request replay reproduced
`Infeasible_Problem_Detected`: measured speed was about 0.00009 m/s while previous
acceleration was -0.195 m/s². With jerk limited to 1 m/s³ and a 0.1 s step,
the next acceleration could not exceed -0.095 m/s², forcing negative speed and
violating the forward-only constraint. This was an inconsistent input/history
contract, not evidence that the NLP needed weaker constraints.

The controller now has one `/drive` output and can calculate while RC manual
mode is selected. Only actual `/ackermann_cmd` messages enter applied history.
Manual commands anchor proposal smoothing to the real forwarded targets;
manual takeover does not stop calculation. Selector status must remain fresh,
but its boolean only reports execution authority. Explicit `/mpcc/enable`
activation and the initial stationary/pose checks remain required.

The warm-start rollout now reuses the existing expanded CasADi transition
instead of repeatedly evaluating Python RK4. The exact 20 ms substeps are
preserved, and no additional native compilation is needed. Minimum footprint
margin is derived from the already extracted corridor constraint values and
bounds rather than a second symbolic expression evaluation. The NLP, IPOPT
options, costs and constraints are unchanged.

A 2026-10-03 isolated Orin NX comparison used horizon 10, one thread, cached
native callbacks and identical inputs. For 34 repeated stationary solves after
the first request, timing was:

| Stage | Before P50 / P95 | After P50 / P95 |
| --- | --- | --- |
| Complete solve | 70.0 / 71.7 ms | 54.9 / 56.3 ms |
| Warm start | 10.1 / 10.3 ms | 3.1 / 3.1 ms |
| IPOPT | 48.1 / 49.0 ms | 48.1 / 49.1 ms |
| Result extraction | 9.3 / 9.8 ms | 1.7 / 1.9 ms |

A separate 60-request comparison at ten course positions and speeds 0, 0.5 and
1.0 m/s improved complete-solve P50/P95/max from 60.6/69.2/74.3 ms to
45.8/54.3/58.8 ms. All 95 paired solves (including the first stationary request)
matched predicted states/controls within 1e-6 and margins within 1e-7; every new
margin also matched an independent evaluation of the original expression within
1e-10. These are fixed-state optimizer checks, not moving-car tracking results.

A subsequent isolated stationary bag replay with RViz and the map gate running
accepted 380 predictions over 38 s of explicitly enabled manual evaluation.
Selector status stayed false, with no faults or deadline misses; sampled solve
P50/P95/max were 58.6/62.2/64.7 ms. Recorded actual speed/steering commands were
zero, while `/drive` proposals remained separate from history. This is an
indoor replay check, not a moving-car or tire-model validation.

The historical venue and indoor tests below used the explicitly stated older
budgets. They do not set the current runtime defaults.
At the venue on 2026-10-03, the then-default TF listener queue (100 messages) allowed
old corrections to accumulate in the MPCC consumer: consumed map-TF source-age
P95/max were approximately 290/345 ms, although recorder arrival age P95 was
3 ms and source broadcast gaps stayed below 80 ms. A depth-10 listener brought
consumed age P95/max to approximately 38/47 ms. The node now bounds that queue;
the 300 ms TF validity limit is unchanged.

The live stationary starting pose was about 5 cm off the reference and 9 degrees
off its tangent. Horizon 10 remained unreliable under the complete vehicle load:
a temporary CPU partition accepted 37 solves over 4 s, with P50/P95/max
92.5/113.7/119.7 ms, before plan expiry. Independent solves at that pose all
succeeded, but wall time reached 238 ms while CPU time was 69–89 ms. The 150 ms
solve deadline and 250 ms plan lifetime were retained. Indoor replay results
therefore do not establish stable live operation; a shorter-horizon comparison
was evaluated before autonomous motion.

Horizon 8 (0.8 s) was also tested with the same constraints and deadlines.
A temporary CPU partition accepted 19 predictions over 1.95 s,
P50/P95/max 83.5/99.7/102.6 ms, before upstream map-TF source age exceeded
750 ms and localization invalidated. A final low-load comparison, with light
telemetry recording and RViz stopped, accepted 10 predictions over 1.19 s,
P50/P95/max 90.3/110.2/116.1 ms, before plan expiry. No autonomous motion was
performed. Temporary CPU assignments were removed. Those measurements precede
the scheduled-takeover and quintic corrections above; the new defaults remain an
experimental low-speed configuration requiring moving-car validation.
