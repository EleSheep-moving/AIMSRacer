# MPCC implementation guide

Read [system architecture](../../../docs/architecture.md) for frame and command
conventions, and [usage](usage.md) for running the package. The current reference
is a closed lap loaded from disk, not a streamed local trajectory.

## Read one control cycle

| File | Responsibility |
| --- | --- |
| [node.py](../aims_mpcc/node.py) | ROS inputs, timers, service and telemetry |
| [runtime.py](../aims_mpcc/runtime.py) | Supervisor, state/plan freshness, command selection |
| [history.py](../aims_mpcc/history.py) | Forwarded command history and steering estimate |
| [worker.py](../aims_mpcc/worker.py) | Separate solver process, readiness, requests and deadlines |
| [solver.py](../aims_mpcc/solver.py) | Optimization problem, initial guesses and solve results |
| [path.py](../aims_mpcc/path.py) | Reference loading, projection and curve evaluation |
| [config.py](../aims_mpcc/config.py) | Vehicle parameters and validation |

Start with the node's subscriptions and request construction, follow a request
through the worker into `MPCCSolver.solve()`, then follow the result back through
the supervisor. A plan contains future controls; only the selected command is
forwarded at each command tick. Command publication and fresh optimization have
different rates.

## Optimization variables

Each predicted state has six components:

```text
X = [x, y, yaw, speed, progress, actual steering]
U = [longitudinal acceleration, steering command, virtual progress speed]
```

Default horizon is 15 intervals of 0.1 s. `_dynamics()` uses RK4 substeps at
20 ms with a kinematic model, steering lag and an understeer correction.
`_build()` creates `X` and `U`, constrains transitions and initial state, and adds
speed, acceleration, steering, jerk, steering-rate/acceleration, acceleration
utilization and footprint corridor constraints.

The objective combines contouring/lag error, heading, reference speed, progress
speed, steering and control smoothness. Trace `geometry()` and the vendor cost
modules when studying normalization. The vehicle model and actuator parameters
include assumptions; satisfying the numerical constraints does not establish
real tire or actuator behavior.

## Warm start

After a successful solve, the solver saves the optimized controls. On the next
request, `_warm_start()` shifts them by `round(elapsed / dt)` (clamped to at least
one step and at most the horizon), repeating the last control at the tail.
It limits acceleration/steering changes relative to the applied command,
recomputes virtual progress speed, and rolls the model forward from the new
measured state. `set_initial()` receives those rebuilt states and controls.

The previous state array is not copied as the new initial trajectory. No previous
Lagrange multipliers are supplied. Failed solves and worker generation changes
clear the stored warm start; the fallback uses reference speed and curvature.
`warm_start_init_point=yes` is configured in IPOPT, but this implementation reuses
primal variables only.

## Native compilation and timing

[native.py](../aims_mpcc/native.py) configures `-O2` and persistent ccache storage.
[prepare_solver.py](../aims_mpcc/prepare_solver.py) prepares callbacks offline;
workers require cache hits and still construct the symbolic problem, link and
warm up at startup. Cache commands and invalidation rules are maintained in
[usage](usage.md#prepare-the-compiled-solver).

`solve_time_s` measures the `solve()` call, including parameter setup and warm-start
construction; it is not solely IPOPT internal execution time. ROS delivery,
worker IPC and actuator communication require separate end-to-end measurements.
Earlier isolated Orin comparisons measured steady solve P50/P95 of about
43.9/76.3 ms with `-O2`, versus 67/142 ms with `-O0`. These are historical
workload-specific observations, not a full moving-vehicle latency budget. The [vehicle checklist](../../../docs/operations/vehicle-checklist.md)
tracks alignment and model work still required before faster operation.
