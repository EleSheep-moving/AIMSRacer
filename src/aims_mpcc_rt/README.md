# AIMSRacer native acados runtime

This package retains the current rear-axle kinematic model, six physical states,
three controls, steering time constant, exact periodic quintic reference and
vehicle configuration. Three auxiliary previous-control states implement jerk
and steering acceleration constraints. It uses generated acados SQP-RTI,
Gauss-Newton and partial-condensing HPIPM through C++.

Defaults: N=10, dt=0.1 s, 20 Hz latest-only solver, 50 Hz command output,
50 ms complete request budget, 20 ms forecast lead, 0.8 s original-source TTL.
The launch defaults to the legacy controller. The new runtime defaults to
shadow outputs; hardware driving acceptance is a separate step.

## Offline bundle and build

Use an isolated checkout, ROS Humble, yaml-cpp/OpenSSL development headers,
CasADi, NumPy, SciPy, and acados_template from the pinned acados 0.5.5 commit
`59d93e17d2985fdd73fc58b8a83ed8f83a024171`. Initialize its BLASFEO/HPIPM
submodules and build acados on the target CPU. Set ACADOS_SOURCE_DIR to that
checkout and LD_LIBRARY_PATH to its installed lib directory.

```bash
export PYTHONPATH="$PWD/src/controller:$PYTHONPATH"
python3 src/aims_mpcc_rt/scripts/export_bundle.py \
  --config src/controller/config/vehicle.yaml \
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

## Select and roll back

```bash
ros2 launch aims_mpcc mpcc.launch.py implementation:=acados_cpp \
  shadow:=true cpp_solve_frequency:=20.0 \
  path_directory:=/absolute/path/to/reference \
  vehicle_config:=/absolute/path/to/vehicle.yaml \
  artifact_directory:=/absolute/path/to/bundle \
  log_directory:=/absolute/path/to/new-log-directory
```

Shadow mode still consumes real `/odometry/filtered`, forwarded
`/ackermann_cmd`, RC authority and map localization health. It publishes only
`/mpcc_rt_shadow/drive`, status/reference/prediction and its enable service.
The isolated acceptance harness additionally remaps **all** input/output
topics and uses synthetic feedback. A shadow controller must not run beside
an enabled real controller in an acceptance domain.

Physical control uses `shadow:=false`, after operator field acceptance; it
requires a measured, verified vehicle profile and a verified closed reference.
Start from a stationary vehicle within 30 degrees of the reference heading,
with matching map hash, fresh protocol-v1 trusted anchor and sole drive owner.
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
| bounded acceleration/jerk/steering rate | command sampler | preserve existing actuator command semantics |
| stop/recovery/finish | output supervisor | bounded stopping; stationary recovery requires explicit reenable |

Reanchoring constructs a distinct candidate from the actual state and applied
prefix, so it needs its own validation. Its cost is included in the complete
publisher callback. This is measured independently of the native optimizer.
The strict vehicle profile is supported; experimental soft-envelope profiles
are refused at ROS startup until their independent recovery comparator is
ported. They remain available through the legacy implementation.

`runtime.csv` records every worker disposition, activation acceptance/reason,
reanchor time and every publication, including zero output. Logger I/O runs
on a bounded asynchronous queue; dropped entries are reported in diagnostics.

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
