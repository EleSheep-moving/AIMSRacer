# MPCC usage

ROS 2 Humble speed-mode control along a manually recorded closed lap. The package
is named `aims_mpcc`; its directory is `src/controller`. It ports the existing
F1TENTH CasADi/IPOPT controller into a standalone package without Isaac imports.

The reference describes geometry, not the driver's speed or a timed trajectory.
MPCC optimizes local progress and produces physical speed (m/s) and steering
(rad). It starts at 0.5 m/s target speed and stops after one lap. Current/duty
control, racing-line optimization, obstacle avoidance and tire modeling are not
part of this implementation.

## Prepare the real car

Read the [remaining real-car work and deployment checklist](../../../docs/operations/vehicle-checklist.md)
before enabling motion. It lists required measurements and remaining controller and estimator work.

Complete the [system installation](../../../docs/installation.md#8-use-system-python)
and [vehicle bringup](../../../docs/operations/bringup.md) first. Run commands below
from the workspace root with the ROS, Livox and workspace overlays sourced.

```bash
export OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1
```

Current V2/V3 odometry already refers to the rear axle; use `rear_offset: 0`.
See the [system architecture](../../../docs/architecture.md) for the frame contract.
The reference is loaded from `path_directory` at startup. A live local-trajectory
input topic is planned but is not implemented in this version.

Copy `src/controller/config/vehicle.yaml` to a run-specific file. Enter the measured longitudinal
distance **from the rear axle to the odometry base_link origin** as `rear_offset`.
The footprint is a rectangle centered on that same base_link origin; specify
conservative `half_length` and `half_width` enclosing the car. Keep `profile:
measured`; set `geometry_verified: true` only after checking these values.
The supplied synthetic profile is explicitly rejected for real drive mode.

The steering-lag assumption (0.115 s), acceleration limits and yaw coefficient are
configuration assumptions, not identified vehicle dynamics. Confirm steering
sign and speed units on the actual car before enabling autonomous motion.

## Record and prepare one lap

Keep localization running throughout recording and execution. The default frames
are `odom` and `base_link`; localization restarting requires a new recording or a
separately established alignment. This version does not perform relocalization.

```bash
ros2 run aims_mpcc record_path --ros-args -p output:=/data/lap.csv
# Drive one forward lap, with a little overlap; then Ctrl-C the recorder.
ros2 run aims_mpcc prepare_path /data/lap.csv /data/reference \
  --vehicle-config /data/vehicle.yaml --left-width 0.9 --right-width 0.9
```

**The widths above are examples, not measured track boundaries.** Supply checked
minimum free distances to the left/right of the processed rear-axle path. The
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

## Prepare the compiled solver

After preparing the path and choosing the vehicle configuration, run:

```bash
ros2 run aims_mpcc prepare_solver /data/reference --vehicle-config /data/vehicle.yaml
```

This command has no ROS node or actuator access. It builds and warms the same
native solver used by the worker. The first `-O2` build on this Orin takes several
minutes. Repeating the command reuses ccache when the generated C, compiler and
options are unchanged. Changes to the model, embedded path/parameters, compiler,
flags or CasADi version invalidate the relevant compilation. Online state inputs
do not cause recompilation. Some parameters only affect runtime values, so changing
them does not necessarily require a new native object.

On this Orin's synthetic reference, first preparation took 322 s, repeated
preparation about 3 s, and two independent workers reached READY in about 3.4 s.
Both workers completed ten tracking requests within the existing 150 ms deadline.
These are isolated checks, not whole-vehicle timing guarantees.

Normal controller startup only accepts cache hits. On a cache miss it reports
`MPCC native cache unavailable` and asks you to run `prepare_solver`; it does not
start a long compilation. The 180 s initialization and 150 ms solve deadlines
remain unchanged. Startup still rebuilds the symbolic problem, links the cached
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

## Shadow and one-lap execution

First inspect shadow predictions. Shadow mode never creates a `/drive` publisher:

```bash
ros2 launch aims_mpcc mpcc.launch.py path_directory:=/data/reference \
  vehicle_config:=/data/vehicle.yaml output_mode:=shadow log_directory:=/data/shadow
```

Select unlocked, **speed + navigation** mode on the RC, with calibration disabled.
Check `/control/autonomy_speed_enabled` is true and `/mpcc/status` says the worker
is ready. Keep Nav2 stopped. Start only stationary, within 0.2 m of the recorded
start, with heading within 15 degrees and lateral error within 0.15 m.

```bash
ros2 service call /mpcc/enable std_srvs/srv/SetBool '{data: true}'
```

Shadow mode evaluates predictions against incoming vehicle state; it does not
move the car or establish closed-loop tracking. It currently requires autonomous
RC selection and uses hypothetical steering while active, so it is not yet a
manual-driving shadow validation tool. Stop that node before launching
drive mode. Keep the same localization session and start pose:

```bash
ros2 launch aims_mpcc mpcc.launch.py path_directory:=/data/reference \
  vehicle_config:=/data/vehicle.yaml output_mode:=drive log_directory:=/data/run
ros2 service call /mpcc/enable std_srvs/srv/SetBool '{data: true}'
# Request a normal decelerating stop:
ros2 service call /mpcc/enable std_srvs/srv/SetBool '{data: false}'
```

Manual takeover remains available through RC selection. It cancels the run and
requires explicit re-enabling after returning to the start. Solver crashes or
timeouts require restarting the controller node. `simulation:=true` is solely
for isolated synthetic tests and must not be used for real operation.

## Execution rules and telemetry

- Commands publish at a target 50 Hz; solving targets 10 Hz in a separate process.
- Measurement source age must remain below 100 ms. Received mode status expires
  after 100 ms. A solve has a 150 ms execution deadline; plans expire 250 ms after
  their state epoch. Timer gaps above 100 ms fault the run.
- Steering state is estimated from forwarded command history; it is not a sensor
  measurement. Prediction state and applied-control history use the same epoch.
- Faults request zero speed immediately; emergency commands supersede ordinary
  acceleration/jerk limits. A zero-speed command is not evidence of instantaneous
  physical stopping. Existing VESC watchdogs remain enabled.
- `/drive` must have only this autonomous publisher. `/mpcc/enable` is explicit;
  command publishing alone never arms the run.
- `/mpcc/status` reports state, reason, progress, cross-track error, state/plan
  age, command values, measured speed, solve latency and deadline misses. Optional
  JSONL logs contain the same values. Follow the [bag recording guide](../../../docs/operations/recording.md)
  to capture sensor inputs, commands and MPCC telemetry together.
