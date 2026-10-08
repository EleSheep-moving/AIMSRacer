# MPCC usage

ROS 2 Humble speed-mode control along a manually recorded closed lap. The package
is named `aims_mpcc`; its directory is `src/controller`. It ports the existing
F1TENTH CasADi/IPOPT controller into a standalone package without Isaac imports.

The reference describes geometry, not the driver's speed or a timed trajectory.
MPCC optimizes local progress and produces physical speed (m/s) and steering
(rad). The supplied vehicle profile targets 1.0 m/s and stops after one lap. Current/duty
control, racing-line optimization, obstacle avoidance and tire modeling are not
part of this implementation.

## Prepare the real car

Read the [remaining real-car work and deployment checklist](../../../docs/operations/vehicle-checklist.md)
before enabling motion. It lists required measurements and remaining controller and estimator work.

Complete the [system installation](../../../docs/installation.md#8-use-system-python)
and [vehicle bringup](../../../docs/operations/bringup.md) first. Run commands below
from the workspace root with the ROS, Livox and workspace overlays sourced.

Apply the single-thread limits only to MPCC commands, as shown below. Do not
export them in the shell that starts LiDAR localization or other vehicle nodes:
`OMP_NUM_THREADS=1` also limits every later OpenMP process in that shell.

Current V2/V3 odometry already refers to the rear axle; use `rear_offset: 0`.
See the [system architecture](../../../docs/architecture.md) for the frame contract.
The reference is loaded from `path_directory` at startup. A live local-trajectory
input topic is planned but is not implemented in this version.

The supplied `src/controller/config/vehicle.yaml` puts `base_link` at the rear
axle (`rear_offset: 0`) and models the 620 mm × 320 mm body as
`front_extent: 0.52`, `rear_extent: 0.10`, and `half_width: 0.16` metres. The front and rear
extents are measured from the rear axle, so this footprint is asymmetric.
Check the actual car before driving; use a run-specific copy if its geometry
differs. The older `half_length` field remains for symmetric example profiles.
Keep `profile: measured` for the real vehicle.
The supplied synthetic profile is explicitly rejected for real operation.

The real-car steering time constant (0.08 s) was confirmed by the
[2026-09-28 speed-mode bag](../../../docs/reports/2026-09-28-speed-mode-calibration.md)
at roughly 0.85–1.15 m/s using 200 Hz raw-IMU yaw rate and measured forward
speed. It absorbs command-to-yaw delay because this model has no separate
dead-time state; it is not a measured servo constant.
Acceleration limits and the yaw coefficient remain operating assumptions.
Confirm steering sign and speed units on the actual car before enabling
autonomous motion. The [low-speed bag protocol](../../../docs/operations/recording.md#mpcc-reference-recording)
captures the extra maneuvers needed alongside the reference lap.

## Record and prepare one lap

For an `odom` reference, keep localization running throughout recording and
execution: restarting changes that session's origin. The separate mapping
launch can record CSV from `/rear_axle/lio_odom`, but that CSV is still in its
mapping session's `odom`. For a map-frame path reusable after restart, follow
the [known-map workflow](../../../docs/operations/known-map-mpcc.md): match the
recorded loop to saved PGO poses, prepare with `--map-file`, and use the
NDT add-on, completing its runtime checks before driving. The controller does not itself relocalize.

```bash
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 ros2 run aims_mpcc record_path --ros-args -p output:=/data/lap.csv
# Drive one forward lap, with a little overlap; then Ctrl-C the recorder.
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 ros2 run aims_mpcc prepare_path /data/lap.csv /data/reference \
  --vehicle-config src/controller/config/vehicle.yaml --left-width 0.5 --right-width 0.5
```

**The 0.5 m widths are the specified centered 1.0 m course model, not measured
track boundaries.** Supply checked minimum free distances to the left/right of the processed rear-axle path. The
solver constrains all four footprint corners inside that corridor. A recorded
driving line does not measure free space. Inspect smoothing and the corner
clearance in the actual area, especially bends.

If the recording includes overlap, select a single lap with `--start-time` and
`--end-time`, using the numeric timestamps in the CSV. Preparation rejects bad
closure, reversed motion, discontinuities, intersections, excessive curvature,
frame mismatches and geometry mismatches. Reverse-speed rejection applies to
the selected lap; signed speed noise down to -0.05 m/s is tolerated near
standstill, consistently with the supervisor. The forward-only solver clamps
that small negative initial speed to zero; raw recordings remain unchanged. It refuses existing output directories
and preserves the original recording. Its output is `path.csv`, `metadata.json`,
and the preserved raw CSV.

The recorder publishes `/mpcc/recorded_path`. The controller publishes
`/mpcc/reference` and `/mpcc/prediction` as `nav_msgs/Path`; overlay them in RViz.
For a saved-map run the reference is in `map`, while predictions and MPCC
dynamics remain in continuous `odom`. The solver receives the available x/y/yaw-projected
`map <- odom` alignment each cycle for path error and corridor evaluation;
see the [frame rationale](../../../docs/architecture.md#persistent-reference-and-local-control-frames).

## Prepare the compiled solver

After preparing the path and choosing the vehicle configuration, run:

```bash
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 ros2 run aims_mpcc prepare_solver /data/reference --vehicle-config /data/vehicle.yaml
```

This command has no ROS node or actuator access. It builds and warms the same
native solver used by the worker. The first `-O2` build on this Orin takes several
minutes. Repeating the command reuses ccache when the generated C, compiler and
options are unchanged. Changes to the model, embedded path/parameters, compiler,
flags or CasADi version invalidate the relevant compilation. Online state inputs
do not cause recompilation. Some parameters only affect runtime values, so changing
them does not necessarily require a new native object.

References now use periodic quintic interpolation for smooth curvature-dependent
costs. Existing prepared CSV bundles load with this representation automatically;
after updating from the former cubic implementation, run `prepare_solver` once
to prepare the matching callbacks. The cache directory does not need renaming.

The prediction horizon is a startup setting: `--horizon N` for `prepare_solver`
and `horizon:=N` for `mpcc.launch.py`. Both default to 10 intervals of 0.1 s.
Use the same value for preparation and launch. Changing it creates a different
native cache entry; existing entries remain reusable without renaming the cache
directory. A shorter horizon reduces look-ahead time as well as computational
work, so check predictions and timing with the RC selector in manual before
autonomous driving.

For the current known-map course, the low-speed manual evaluation uses
10 intervals (1.0 s), with 0.1 s decision spacing unchanged. Optimization defaults
to 5 Hz; command output remains 50 Hz:

```bash
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 ros2 run aims_mpcc prepare_solver \
  src/controller/recordings/current --vehicle-config src/controller/config/vehicle.yaml --horizon 10
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 ros2 launch aims_mpcc mpcc.launch.py \
  path_directory:=/home/aims/AIMSRacer/src/controller/recordings/current \
  horizon:=10 solve_frequency:=5.0 solver_timeout:=0.25
```

The [measured timing comparison](implementation.md#native-compilation-and-timing)
uses identical inputs before and after the rollout/extraction optimization.
A 10-interval stationary workload improved from P50/P95 70.0/71.7 ms to
54.9/56.3 ms. These are isolated solve timings; moving-car validation remains
required. Increasing thread variables did not help the installed single-thread
OpenBLAS backend.

On this Orin's synthetic reference, first preparation took 322 s, repeated
preparation about 3 s, and two independent workers reached READY in about 3.4 s.
Both workers completed ten tracking requests within the existing 150 ms deadline.
These are isolated checks, not whole-vehicle timing guarantees.

Normal controller startup only accepts cache hits. On a cache miss it reports
`MPCC native cache unavailable` and asks you to run `prepare_solver`; it does not
start a long compilation. The initialization deadline is 180 s; the current 5 Hz online result
acceptance budget defaults to 250 ms at 5 Hz. A late request is skipped without terminating
the solver process. Startup still rebuilds the symbolic problem, links the cached
object and performs stationary warm-up; it does not restore an old controller state.

The persistent cache defaults to `${XDG_CACHE_HOME:-$HOME/.cache}/aims_mpcc/ccache`.
Set `AIMS_MPCC_CACHE_DIR` to an absolute directory to override it, using the same
value and user for preparation and execution. You can inspect or clear it with:

```bash
# Adjust for XDG_CACHE_HOME / AIMS_MPCC_CACHE_DIR if set.
ccache --dir "$HOME/.cache/aims_mpcc/ccache" --show-stats
ccache --dir "$HOME/.cache/aims_mpcc/ccache" --clear
```

After clearing or eviction, rerun `prepare_solver`. ccache bounds disk usage
and handles concurrent writers; each worker has its own temporary JIT directory.
If sudo is unavailable on Ubuntu 22.04, `bash src/controller/tools/install_ccache_user.sh`
installs the Ubuntu ccache/hiredis binaries under `~/.local` without a virtual
environment. The MPCC helper also discovers `~/.local/bin/ccache` if it is not on PATH.

## Manual evaluation and one-lap execution

One controller publishes proposed commands on `/drive`. The RC selector decides
whether to forward them to `/ackermann_cmd`; MPCC computation and actuator
selection are independent. There is no separate preview mode or output topic.

```bash
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 ros2 launch aims_mpcc mpcc.launch.py \
  path_directory:=/home/aims/AIMSRacer/src/controller/recordings/current \
  horizon:=10 solve_frequency:=5.0 solver_timeout:=0.25 \
  log_directory:=/home/aims/mpcc-logs/manual-evaluation
```

For manual evaluation, select **manual + speed** on the RC with calibration
disabled. Keep Nav2 stopped. `/control/autonomy_speed_enabled` may be `false`;
it must still publish fresh selector status. Check `/mpcc/status` reports
`worker_ready: true` and, for a map reference, verify matching-map localization.
Start stationary near any point of the closed reference, with heading within
30 degrees of its forward tangent. The whole
configured body footprint must fit the corridor when `enforce_corridor: true`.
The current complete-lap experiment sets it to false, omitting the OCP track
constraints and footprint stop checks. Cross-track error remains a recorded
tracking metric, with no fixed error-distance stop gate. The controller uses the
nearest path point as this run's start and counts one lap from there; crossing
the CSV's first point does not finish the run. For a map reference these checks
use the vehicle pose aligned into map, while solver dynamics remain in odom.
Then explicitly enable computation:

```bash
ros2 topic echo /control/autonomy_speed_enabled --once
ros2 topic echo /mpcc/status --once
ros2 service call /mpcc/enable std_srvs/srv/SetBool '{data: true}'
```

Inspect the green reference and red prediction in RViz. While manual is selected,
RC commands drive the vehicle and MPCC continues calculating. Steering estimation
and prediction history always use the selector's actual `/ackermann_cmd` output;
proposed commands are never recorded as executed. Acceleration/rate metadata
is reused only when an autonomous forwarded command matches MPCC's proposal.
External commands carry no known ramp metadata, so those values default to zero;
this does not mean measured vehicle acceleration is zero.

After the vehicle checks pass, select unlocked **speed + navigation**, with
calibration disabled, to let the selector forward `/drive`. The same node keeps
running; `autonomy_selected` in `/mpcc/status` becomes `true`. Switching back to
manual returns vehicle authority to the driver while computation continues.
For an `odom` reference, keep the same localization session; for a `map` reference,
relocalize against the exact saved map after restarting localization.

```bash
ros2 service call /mpcc/enable std_srvs/srv/SetBool '{data: false}'
```

Disabling in manual immediately ends computation. Disabling while autonomous
requests a normal decelerating stop. Faults still publish a zero-speed proposal;
the selector determines whether it reaches the vehicle. Solver process or pipe
failures require restarting the controller node. Late replies are discarded
while the worker continues; they do not themselves require a restart.
`simulation:=true` is solely
for isolated synthetic tests and must not be used for real operation.

## Runtime timing parameters

**Current settings, updated 2026-10-08.** This table defines the parameter units
and time origins. Older timing experiments retain their original configurations
as historical evidence; do not copy their budgets into a current launch.

| Parameter / quantity | Unit | Current default | Meaning / time origin |
| --- | --- | --- | --- |
| `horizon` | Intervals | 10 | Number of prediction intervals; requires a matching native cache |
| `dt` | Seconds per interval | 0.1, fixed by the current worker | Model decision spacing; not a ROS launch argument |
| `solve_frequency` | Hz | 5 | Scheduling target for new requests; one request in flight |
| `handover_delay` | Seconds | `1 / solve_frequency` = 0.2 | Submission to scheduled takeover; internally derived, not a launch argument |
| `solver_timeout` | Seconds | 0.25 | Request submission to parent receipt of the reply; late results are skipped |
| `plan_ttl` | Seconds | `horizon * 0.8 * dt` = 0.8 | Maximum plan age from the original EKF measurement; checked on receipt, takeover and real command output |
| `solver_max_iterations` | Iterations | 35 | IPOPT iteration cap, configured in `vehicle.yaml` |
| Command output interval | Seconds | 0.02 | 50 Hz command publication; not the model decision spacing |

For horizon 15 with the same dt, default `plan_ttl` is 1.2 s. An explicit
`plan_ttl:=...` overrides the computed default even when horizon changes.
**Remove old `plan_ttl:=0.75` overrides** from saved shell commands or launch
wrappers to use the current policy. Startup validates the budget against request
scheduling and available prediction coverage. Changing runtime frequency or age
budgets does not require rebuilding the native solver; changing horizon does.

Plan age uses `now - source_stamp`, with the original EKF epoch mapped onto the
controller's monotonic clock. Reply age uses `now - submitted_at`. Receipt,
scheduled takeover, actual activation and rejected/late results never reset
`source_stamp`. For example, on one common time axis, a measurement at 1.0 s
and TTL 0.8 s give an expiry deadline of 1.8 s even if the result arrives at
1.25 s. The 0.25 s reply budget has a separate origin at request submission.

Current takeover tolerances are 0.30 m position, 30 degrees yaw, 0.30 m/s speed
and 20 degrees estimated steering; actual speed/steering target differences use
the same 0.30 m/s and 20 degree limits. `/mpcc/status` reports effective
`horizon`, `solve_frequency`, `solver_timeout`, `plan_ttl`, `handover_delay` and
`handover_limits`; angular errors/limits in telemetry are radians. Map tilt and
measured-speed operating-range rejection gates are removed.

## Execution rules and telemetry

- Commands publish at a target 50 Hz; solving defaults to 5 Hz in a separate process.
  `solve_frequency`, `solver_timeout` and `plan_ttl` are startup-only parameters; they do not change
  the optimization graph or require native recompilation.
- Measurement source age must remain below 100 ms. Received selector status expires
  after 100 ms. A reply is eligible only within 250 ms of request submission;
  this includes worker queueing and parent delivery. Late results are discarded
  without a timeout FAULT or worker restart. Plans expire
  `horizon * 0.8 * 0.1 s` after their **original measurement epoch** by default
  (800 ms for horizon 10; 1.2 s for horizon 15), not their future takeover
  epoch. An explicit startup `plan_ttl` overrides the horizon-derived default. Timer gaps above 100 ms fault the run. Plan lifetime remains an
  independent experimental low-speed budget, not a validated tracking-error
  bound; changing the reply acceptance budget does not extend it.
- Steering state is estimated from forwarded command history; it is not a sensor
  measurement. Real speed-mode command history must cover the measurement epoch
  and have a sample within 100 ms. Each request predicts from that measurement to
  takeover at submission + one solve period (200 ms at 5 Hz). The known prefix replays `/ackermann_cmd`;
  its future part forecasts the old plan in autonomous mode or holds the latest
  actual target in manual mode. Future bridge simulation may read old-plan
  controls beyond its TTL while still inside its prediction horizon; this
  does not extend real execution validity. Only actual control time can trigger
  the plan-age stop. An early solver reply is staged until takeover. A late reply is checked against
  the latest state and actual targets before activation. Replies beyond the
  250 ms budget are discarded before takeover. Only one solve is in flight, so an
  overrun temporarily reduces the optimization update rate.
  New-plan interpolation starts at the actual activation tick, not at the old
  measurement epoch; a late tick still begins with the first new control.
  `handover_error` records position, yaw, speed, steering and actual-target
  differences at activation. MPCC retains its prediction mismatch limits:
  0.30 m position, 30 degrees yaw, 0.30 m/s speed and 20 degrees steering, with
  0.30 m/s and 20 degrees on actual-target changes. A rejected candidate does
  not extend the previous plan's lifetime.
- Map references require protocol version 1 health from `/localization/status`,
  matching map identity and an available map/odom transform. Trusted anchor age
  must remain within 0.5 s in source and monotonic time, and health heartbeat
  within 0.3 s. Epoch changes or readiness loss fault the controller, cancel
  current/pending plans and request zero speed. Three new NDT commits can restore
  localization readiness; MPCC still requires manual enable after a fault.
  Ordinary committed corrections preserve the running plan. NDT owns registration
  gates; independent scan/map consistency is diagnostic. TF timer republication
  cannot renew trusted anchor age. Map x/y/yaw are projected for the planar model.
  Finite-state checks, stationary-start checks and actuator target limits remain.
- IPOPT is capped by `solver_max_iterations` (35).
  `Maximum_Iterations_Exceeded` results are discarded and counted in
  `iteration_limit_skips`; the previous valid plan continues until its original
  expiry while the next scheduled request retries. Other solver failures still
  fault. `solver_status` and `solver_iterations` identify the last returned result.
  Failed solves preserve the last valid trajectory seed, shifted by total elapsed
  request time. See [solver diagnostics](solver-diagnostics.md) for residuals,
  constraint groups, warm-start age and failed-request snapshots.
- Faults request zero speed immediately; emergency commands supersede ordinary
  acceleration/jerk limits. A zero-speed command is not evidence of instantaneous
  physical stopping. Existing VESC watchdogs remain enabled.
- `/drive` must have only this autonomous publisher. `/mpcc/enable` is explicit;
  command publishing alone never arms the run.
- `/mpcc/status` reports state, reason, `autonomy_selected`, progress, cross-track error, state/plan
  source age, plan phase since activation, `handover_lateness`, pending takeover time, handover errors,
  rejection count, command values, measured speed and solve latency.
  `late_result_skips` counts requests beyond the acceptance budget;
  `deadline_misses` is retained as an alias for that count and does not imply a FAULT.
  `solver_busy` remains true while a skipped solve finishes. Its reply is drained
  before the next request, preventing concurrent solves and a stale-input backlog.
  The startup parameter `solver_timeout` now means the result acceptance budget,
  rather than a process-kill timeout. `plan_ttl` must cover
  one solve period plus the larger of the result budget and scheduled handover
  delay, plus a 20 ms command tick. It cannot exceed the available predicted
  control coverage; an exhausted prediction faults instead of repeating its
  last control indefinitely. Source age and jitter consume the remaining margin.
  `request_timing` separates source-to-submission, worker queue, request-to-reply,
  result delivery and time remaining before takeover (seconds). Optional
  JSONL logs contain the same values. Follow the [bag recording guide](../../../docs/operations/recording.md)
  to capture sensor inputs, commands and MPCC telemetry together.
