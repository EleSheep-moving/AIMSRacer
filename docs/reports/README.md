# Experiment reports

These are dated evidence, not current operating instructions. Keep their sample
conditions, source assumptions and limitations with the results. Current behavior
is maintained in [architecture](../architecture.md) and outstanding vehicle work
in the [checklist](../operations/vehicle-checklist.md).

| Report | Scope |
| --- | --- |
| [2026-09-20 engineering review](2026-09-20-engineering-review.md) | Synthetic estimator diagnostics and controller limitations |
| [2026-09-21 frame check](2026-09-21-frame-check.md) | Stationary powered-vehicle frame/latency observations with the earlier local FAST-LIO modification |
| [2026-09-28 vehicle response](2026-09-28-vehicle-response.md) | Bag-derived speed onset, steering command path, IMU yaw response and MPCC single-lag fit |
| [2026-09-28 speed-mode calibration](2026-09-28-speed-mode-calibration.md) | New speed-only bag: speed scale, command-to-motion timing, steering lag and LIO age |
| [2026-09-28 map reference](2026-09-28-map-reference.md) | Closed-lap extraction and matching to saved PGO map poses |
| [2026-10-04 premature plan expiry](2026-10-04-plan-expiry.md) | Historical 750 ms / 10 cm configuration; future forecast versus actual plan expiry |
| [2026-10-04 LIO delay and EKF yaw](2026-10-04-lio-delay.md) | Incident latency, two-second EKF replay, rotation Jacobian and matching-cost evidence; cumulative backlog remains unexplained |
| [2026-10-05 LIO worker and catch-up](2026-10-05-fastlio-thread.md) | Joined worker, preserved IMU history, deskew boundaries and isolated desktop replay acceptance |
| [2026-10-06 NX workspace synchronization](2026-10-06-nx-workspace-sync.md) | Preserved NX regression sources, current C++ test interfaces, branch-specific synthetic startup and desktop/NX/GitHub synchronization |

Future reports should identify source/configuration versions, hardware, workload,
measurement definitions, sample count and evidence location. Generated local
artifacts must be identified as unavailable from a fresh checkout when applicable.
