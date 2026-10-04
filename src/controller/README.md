# AIMSRacer MPCC

`aims_mpcc` provides ROS 2 Humble speed-mode control along a recorded closed lap,
using CasADi/IPOPT. Its source directory is `src/controller`.

The current controller loads a reference from `path_directory`, targets 1.0 m/s
and stops after one lap. Live local-trajectory topic input is not implemented.
It can start stationary near any point of the closed path (heading error at
most 30 degrees; with `enforce_corridor: true`, the full body must fit the corridor). One
lap is counted from the actual activation point, rather than the CSV seam.
The current defaults are horizon 10 (1.0 s), 5 Hz optimization and 50 Hz
command publication, with state prediction to a scheduled takeover time.
The objective weights in `config/vehicle.yaml` are runtime solver parameters:
changing only these weights reuses the prepared native cache after restart.
`contour_weight` sets lateral correction, `heading_weight` sets tangent
alignment, and `steering_rate_weight` / `steering_acceleration_weight` penalize
command changes. The corresponding physical rate limits remain hard constraints.
The operating target is 1.0 m/s cruise, a 1.5 m/s speed ceiling, and absolute
lateral error within 0.20 m. This error target is evaluated from recorded data;
it does not stop the car at 0.20 m. The complete-lap experiment sets
`enforce_corridor: false`: both the OCP footprint constraints and measured/start
footprint checks are disabled. The reference still describes a centered 1 m
corridor; cross-track error is still measured and penalized. RC authority,
actuator limits, estimator validity and plan expiry remain active. Set the flag
to true to restore track constraints and prepare the matching native cache.
`solver_max_iterations: 30` caps IPOPT iterations. A result with status
`Maximum_Iterations_Exceeded` is skipped and the next request is attempted;
the last accepted plan retains its original expiry. This does not accept an
unconverged trajectory. Replies received more than 250 ms after submission
are also discarded, without killing the solver or faulting the run. The
previous plan's original source epoch is not refreshed by a skipped result.
Plan age defaults to `horizon * 0.8 * dt` (0.8 s for horizon 10 and dt = 0.1 s),
counted from the original EKF measurement. `solver_timeout` separately limits
submission-to-reply time to 0.25 s. See the
[runtime timing parameters](docs/usage.md#runtime-timing-parameters); omit a
`plan_ttl` override to derive its value from horizon. Only one request is in flight;
after a late solve finishes, its reply is drained and a fresh request can run.
The real-car configuration sets `minimum_drive_speed: 0.2` m/s from the
operator's observed motor dead zone. A positive running proposal below this
threshold is sent as 0.2 m/s; disabled/faulted commands remain zero, and
stopping commands below the threshold become zero. Physical speed and OCP
states may still pass below 0.2 m/s during acceleration or braking. Diagnostics
separate the continuous `model_speed_command` from the published `speed_command`.
The command-history bridge models subthreshold setpoints as zero and uses the
same actuator mapping when forecasting takeover. The finish stop boundary
includes braking/latency distance at the minimum effective speed, computed
from the effective `plan_ttl`, brake and jerk limits, so lap completion
tolerates this early stop plus 0.05 m.
This threshold is operator-supplied, not a new motor-response calibration.
References use a quintic periodic spline (C4 continuity), including at the
lap seam, to keep curvature-dependent steering costs smooth for IPOPT.

## Documentation

- [Usage: prepare a lap, cache the solver, evaluate manually and select autonomous control](docs/usage.md)
- [Implementation and code reading guide](docs/implementation.md)
- [System architecture](../../docs/architecture.md)
- [Vehicle checklist](../../docs/operations/vehicle-checklist.md)
- [Dependencies and installation](../../docs/installation.md#8-use-system-python)

## Interfaces

| Direction | Interface | Purpose |
| --- | --- | --- |
| Input | `/odometry/filtered` | Rear-axle state; default odometry topic |
| Input | `/ackermann_cmd` | Forwarded command history |
| Input | `/control/autonomy_speed_enabled` | RC authority status |
| Parameter | `path_directory` | Prepared reference directory |
| Output | `/drive` | Proposed speed/steering commands; RC selects execution |
| Output | `/mpcc/status`, `/mpcc/reference`, `/mpcc/prediction` | Status and visualization |
| Service | `/mpcc/enable` | Explicit enable/disable (`std_srvs/srv/SetBool`) |

## Build and tests

From the workspace root, after sourcing the installed dependencies:

```bash
/usr/bin/python3 -m colcon build --symlink-install --packages-select aims_mpcc
source install/setup.bash
/usr/bin/python3 -m pytest -q src/controller/tests
```

The tracked tests exercise native cache behavior; they do not establish vehicle
tracking performance. See [third-party notices](NOTICE.md) and
[LICENSE](LICENSE) for attribution and licensing.
