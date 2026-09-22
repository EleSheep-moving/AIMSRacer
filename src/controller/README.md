# AIMSRacer MPCC

`aims_mpcc` provides ROS 2 Humble speed-mode control along a recorded closed lap,
using CasADi/IPOPT. Its source directory is `src/controller`.

The current controller loads a reference from `path_directory`, targets 1.2 m/s
and stops after one lap. Live local-trajectory topic input is not implemented.

## Documentation

- [Usage: prepare a lap, cache the solver, run shadow or drive mode](docs/usage.md)
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
| Output | `/drive` | Speed/steering commands in drive mode |
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
tracking performance. See [third-party notices](THIRD_PARTY_NOTICES.md) and
[LICENSE](LICENSE) for attribution and licensing.
