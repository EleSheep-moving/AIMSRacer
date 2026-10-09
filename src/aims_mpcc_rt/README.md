# AIMSRacer native acados runtime

This package retains the current rear-axle kinematic model, six physical states,
three controls, steering time constant, exact periodic quintic reference and
vehicle configuration. Three auxiliary previous-control states retain command
history and cost bookkeeping. `rate_bounded_v2` removes additional hard jerk
and steering-command acceleration limits; `legacy_bounded_v1` retains them. It uses generated acados SQP-RTI,
Gauss-Newton and partial-condensing HPIPM through C++.

Defaults: N=10, dt=0.1 s, 20 Hz request cap with one owned pending plan, 50 Hz command output,
50 ms requested budget including result delivery mutex wait, 20 ms forecast lead, 0.8 s original-source TTL.
The launch defaults to the legacy controller. Select `implementation:=acados_cpp`
for the native runtime, which uses the standard command/status/service names
and starts disabled. Hardware driving acceptance is a separate step.

> The audit repairs pass desktop tracking and NX software timing/load
> qualification. On 2026-10-10 the operator deferred longitudinal response
> identification and selected locked-car checks followed by supervised low-speed
> trials. Synthetic motor-response violations remain recorded observations;
> they are not a runtime stop gate or a prerequisite for those trials.
> See [repair validation](../../docs/reports/2026-10-09-mpcc-runtime-repair-validation.md).
> The [standard-interface follow-up](../../docs/reports/2026-10-10-mpcc-standard-interface.md)
> records removal of the runtime mode and the desktop/NX interface verification.

For the actual vehicle trial, follow the [NX field test runbook](../../docs/operations/mpcc-acados-field-test.md).

## Offline bundle and build

Use an isolated checkout, ROS Humble, yaml-cpp/OpenSSL development headers,
CasADi, NumPy, SciPy, and acados_template from the pinned acados 0.5.5 commit
`59d93e17d2985fdd73fc58b8a83ed8f83a024171`. Initialize its BLASFEO/HPIPM
submodules and build acados on the target CPU. Set ACADOS_SOURCE_DIR to that
checkout and LD_LIBRARY_PATH to its installed lib directory.

```bash
export PYTHONPATH="$PWD/src/controller:$PYTHONPATH"
python3 src/aims_mpcc_rt/scripts/export_bundle.py \
  --config src/controller/config/native_rate_bounded.yaml \
  --reference /absolute/path/to/reference --output /absolute/path/to/bundle
source /opt/ros/humble/setup.bash
colcon build --packages-select aims_mpcc aims_mpcc_rt \
  --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

To transfer generated C from x86 to NX, copy the complete bundle directory,
then rebuild its immutable sources against the target's pinned acados install:

```bash
python3 src/aims_mpcc_rt/scripts/build_bundle.py \
  --bundle /absolute/path/to/bundle \
  --acados-install /absolute/path/to/acados/install-runtime
```

Source and native manifests bind the exact config, recorded reference,
SpeedPlanner profile, generator source, native ABI/platform and actual loaded
dependency hashes. Online startup refuses missing or incompatible artifacts;
it never generates or compiles a solver. Changing config/reference requires
a new offline bundle. Custom runtime parameters are fixed until restart.

The optional `--horizon 20 --dt .05` and `--horizon 25 --dt .05` exports
use RK4 steps of 20 + 20 + 10 ms per stage. The default 100 ms stage retains
five 20 ms steps. Stage costs scale by dt/0.1; terminal cost retains its
original weight. These are evaluation profiles; the default remains N10/0.1.
The v2 runtime rejects 50 ms prediction stages because their 20/20/10 ms
stage-local holds do not match the global 20 ms output schedule. Export does
not establish runtime qualification. The Python backend retains its original
dt validation.

## Select and roll back

```bash
ros2 launch aims_mpcc mpcc.launch.py implementation:=acados_cpp \
  cpp_solve_frequency:=20.0 \
  path_directory:=/absolute/path/to/reference \
  vehicle_config:=/absolute/path/to/vehicle.yaml \
  artifact_directory:=/absolute/path/to/bundle \
  log_directory:=/absolute/path/to/new-log-directory
```

The native runtime consumes `/odometry/filtered`, forwarded `/ackermann_cmd`,
RC authority and map localization health. It publishes `/drive`, `/mpcc/status`,
`/mpcc/reference` and `/mpcc/prediction`, with `/mpcc/enable` as its service.
There is no shadow mode or parameter. Startup and supervised restarts remain
disabled; an explicit successful enable is required before driving.
The isolated acceptance harness uses explicit ROS remaps for **all** input/output
topics and synthetic feedback, independently of vehicle runtime behavior.
On NX, source the tested vehicle/localization/monitor underlays followed by
`/home/aims/aimsracer-data/experiments/mpcc-acados-runtime/remove-shadow-20261010/ros-install/local_setup.bash`.
The preserved `repair-final-20261009/ros-install` controller still has the old
interface switch; the new overlay supplies the standard-interface revision.

Physical driving validation is under operator control; the current evidence
does not establish physical closed-loop acceptance. It requires
a measured, verified vehicle profile and a verified closed reference.
Start from a stationary vehicle within 30 degrees of the reference heading,
with matching map hash, fresh protocol-v1 trusted anchor carrying the same
qualified transform/epoch/sequence/stamp, and sole drive owner. Deploy the
updated C++ localization monitor with this runtime.
Enable via `/mpcc/enable`. Stop/withdraw authority before switching runtimes.
Rollback selects `implementation:=legacy`; its existing backend, frequency and
deadline arguments remain unchanged. The launch selects exactly one runtime.

## Ownership and validation

| Check | Owner / timing | Purpose |
|---|---|---|
| odom frames, clock, finite/unit quaternion, age/discontinuity | receive callback | correct and fresh rear-axle state |
| map identity and epoch/anchor/heartbeat protocol | health callback/output | trusted global constraint; TF timestamp alone is insufficient |
| forwarded wire commands | actual history | predict source-to-takeover motion without treating proposals as applied commands |
| status/finite/OCP constraints and true quintic footprint | solver candidate, once per RTI | reject nonlinear infeasible candidates |
| old-history continuity and reanchored candidate | activation | account for lateness without executing skipped new controls |
| source TTL, sole publisher, RC selection | output callback | preserve command ownership and original information age |
| acceleration/braking, steering rate and physical envelope | command sampler/certificate | v2 preserves these bounds without extra hard jerk/angular acceleration |
| stop/recovery/finish | output supervisor | bounded stopping; stationary recovery requires explicit reenable |

Reanchoring constructs a distinct candidate from the actual state and applied
prefix, so it needs its own validation. A feasible original sequence stays
unchanged. If the new prefix makes it infeasible, a bounded endpoint/acceleration
transport may form an alternate candidate, with the same full nonlinear
certificate. It neither skips unexecuted controls nor adds optimizer passes or
extends source TTL. Both nonlinear macro and prospective actual held-output certificates must
pass. The activation certificate uses the decision-time physical prediction
and current finish cap; later changing caps use a per-hold physical budget.
This does not certify unidentified motor response or all future disturbances.
Certificate cost is included in the complete publisher callback. This is measured independently of the native optimizer.
The strict vehicle profile is supported; experimental soft-envelope profiles
are refused at ROS startup until their independent recovery comparator is
ported. They remain available through the legacy implementation.

`runtime.csv` records every worker disposition, activation acceptance/reason,
reanchor time and every publication, including zero output. Logger I/O runs
on a bounded asynchronous queue; dropped entries are reported in diagnostics.
`compute_s`, `delivery_wait_s` and `delivery_s` distinguish worker work from
result visibility. Pending ownership skips are recorded separately from solver
failures. A source TTL never renews on a newer solver result.

The launch uses `runtime_supervisor.py` for bounded process shutdown/restart
when the native worker is unavailable. A restart remains disabled and needs
explicit reenable. Startup status carries the child's per-spawn nonce, persistent
disabled-birth witness and count of successful explicit enable requests, so
late DDS discovery does not mistake a legitimate RUNNING status for automatic
startup. Previous-process status cannot refresh the new child. It does not
interrupt a running acados C++ call in place.

## Validation commands

Numeric CTest must explicitly select a prepared bundle:

```bash
colcon build --packages-select aims_mpcc_rt --cmake-args \
  -DAIMS_MPCC_RT_TEST_BUNDLE=/absolute/path/to/synthetic-bundle
ctest --test-dir build/aims_mpcc_rt --output-on-failure
python3 -m pytest src/aims_mpcc_rt/tests/test_node_startup.py \
  src/aims_mpcc_rt/tests/test_benchmark_accounting.py
```

The opt-in ROS probe requires a synthetic map fixture:
`AIMS_MPCC_PROTOCOL_BUNDLE=... AIMS_MPCC_PROTOCOL_OUTPUT=... python3 -m pytest
src/aims_mpcc_rt/tests/test_protocol_ros.py`. It injects map/health/clock/state/
manual/publisher faults. Its selector echo is synthetic.

`tools/acceptance.py` uses actual RC selector and VESC conversion, all actuator
topics isolated, with an independent 2 ms midpoint bicycle plant with speed/
steering lag. `tools/nx_joint_load.py` adds genuine FAST-LIO2/EKF/NDT bag replay
and power/temperature telemetry; replay is CPU load, **not controller feedback**.
Qualification requires sustained motion, request/publication coverage, all
failed submissions/rejections, timing bounds and continuous native registration
through the same measurement window. Neither experiment proves field tracking.


## Fixed-input failure reproduction

```bash
ros2 run aims_mpcc_rt replay_requests /absolute/path/to/strict-bundle \
  src/controller/tests/fixtures/nx_failure_requests.json /absolute/path/to/result.json
```

This is an offline executable with no ROS node or actuator publisher. It
selects recorded cases matching horizon, dt and every recorded config key;
requires a strict-envelope bundle; and cold-starts each request independently.
It does not reconstruct the original warm-start history. Nonfinite diagnostic
numbers are JSON null; no matching case returns a nonzero exit code. Native
solver stdout is separate from the JSON output file.

A state with initial minimum utilization above one is already infeasible under
the strict acceleration envelope. Faster optimization cannot authorize it.
V2 recovery checks the current physical state separately from its conservative
hold proxy. When the current state is feasible, it clips braking to the hold
budget, including zero capacity; this also applies when source TTL or horizon
ends inside an upcoming publication packet. An already infeasible state retains
bounded deceleration and remains uncertified. STOPPING retains acceleration/braking bounds; only v1 retains command jerk.
An initially infeasible state cannot be made retrospectively certified by
a fallback stop. Recovery requests target zero speed; stationary
recovery requires explicit reenable.

See [implementation and validation report](../../docs/reports/2026-10-09-mpcc-acados-runtime.md)
for measured results, version boundaries and field limitations.


For forensic takeover evidence, `AIMS_MPCC_CAPTURE_TAKEOVER=1` records exact
source/actual state, applied prefixes and controls as JSON in `takeover_snapshot`
CSV rows. It is disabled by default and uses the same bounded asynchronous logger.
The output decision clock is sampled after acquiring state ownership; callback
entry is retained separately for full callback timing.

Exact ordered warm-start replay is available through `replay_runtime`: pass
a bundle, recorded ordered request JSON and an output JSON path. Optional
`frozen`/`refresh` selects second-pass geometry policy. Forensic request capture
is opt-in (`AIMS_MPCC_CAPTURE_REQUEST=1`) and excluded from timing qualification.
