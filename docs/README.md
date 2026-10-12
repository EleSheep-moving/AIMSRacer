# Documentation

**English** · [简体中文](README.zh-CN.md) · [Project overview](../README.md)

Guides for the current **FAST-LIO2 + rear-axle EKF + NDT + native MPCC** stack. Current guides listed here have full English and Simplified Chinese versions, with English as the default and a language switch on each page. Historical reports and retired design notes retain their original language.

## Suggested reading order

1. **New deployment:** [system dependencies](deployment/README.md) → [target platform](deployment/orin.md) → [vehicle launch](operations/bringup.md).
2. **First known-map run:** [frames and topics](architecture.md) → [map/reference preparation](../src/controller/docs/usage.md) → [initialization and enable](operations/known-map-mpcc.md).
3. **Controller development:** [implementation](../src/controller/docs/implementation.md) → [native runtime](../src/aims_mpcc_rt/README.md) → [regressions and replay](../verification/README.md).
4. **Analyze a run:** [recording](operations/recording.md) → [field review](reports/2026-10-10-mpcc-field-review.md) → [dated reports](reports/README.md).

## Installation and hardware

| Guide | What it covers | Language |
| --- | --- | --- |
| [Deployment](deployment/README.md) | System packages, pinned sources, SDK preparation, bounded builds, bundle data | English / 简体中文 |
| [Orin NX](deployment/orin.md) | Vehicle devices, MID360 network, native deployment | English / 简体中文 |
| [x86 / NUC](deployment/nuc.md) | Same source stack on x86; target-specific bundle compilation | English / 简体中文 |
| [Device rules](../rules/README.md) | Stable device names and serial-port ownership | English / 简体中文 |
| [Dependency manifest](../dependencies/manifest.json) | Exact upstream commits, patches and acados architecture settings | JSON |

## Vehicle operation and data

| Guide | What it covers | Language |
| --- | --- | --- |
| [Launch guide](operations/bringup.md) | Choose vehicle, mapping, race or controller-only launch | English / 简体中文 |
| [Known-map MPCC](operations/known-map-mpcc.md) | Map and bundle identity, rear-axle initial pose, enable, stop and repeated laps | English / 简体中文 |
| [Recording](operations/recording.md) | Launch-managed bags, runtime logs, QoS, session layout and timing definitions | English / 简体中文 |
| [Longitudinal acceleration](operations/longitudinal-acceleration.md) | Optional wheel + IMU output, standalone launch, diagnostics and field recording | English / 简体中文 |
| [Vehicle checklist](operations/vehicle-checklist.md) | Device, localization, authority and run checks | English / 简体中文 |
| [Reference and bundle workflow](../src/controller/docs/usage.md) | Map-frame reference, speed planning, export and target-CPU build | English / 简体中文 |
| [Map/reference tools](../tools/reference/README.md) | Save a PGO map and recover a matching closed reference | English / 简体中文 |
| [Calibration context](../src/aims_racer_system/docs/calibration.md) | Existing response evidence, current workflows and archived collection guide | English / 简体中文 |

Maps, reference inputs, generated bundles and raw bags are external data. A repository clone supplies source and reports, not all recorded data or a runnable vehicle bundle.

## Architecture, control and diagnostics

| Guide | What it covers | Language |
| --- | --- | --- |
| [System architecture](architecture.md) | TF ownership, rear-axle frames, topics, timestamps, estimation and control flow | English / 简体中文 |
| [Vehicle response](vehicle-response.md) | Steering lag, speed PID, measured response, model assumptions and bundle changes | English / 简体中文 |
| [MPCC implementation](../src/controller/docs/implementation.md) | Vehicle model, costs, geometry, speed profile and execution history | English / 简体中文 |
| [Native controller](../src/aims_mpcc_rt/README.md) | C++ runtime, immutable bundles, launch parameters and diagnostics | English / 简体中文 |
| [Offline reference tools](../src/controller/README.md) | `aims_mpcc` package and its offline responsibilities | English / 简体中文 |
| [Localization monitor](localization_monitor.md) | Trusted anchors, freshness, epoch handling and background quality diagnostics | English / 简体中文 |
| [RC selector](../src/ackermann_mux/README.md) | RC channels, manual/autonomous arbitration, command bounds and timeouts | English / 简体中文 |
| [Vehicle package](../src/aims_racer_system/README.md) | Launch composition and sensor/estimator adapters | English / 简体中文 |
| [Installed system helpers](../src/aims_racer_system/scripts/README.md) | Gyro correction, rear-axle conversion and NDT initialization | English / 简体中文 |

## Verification and measured evidence

| Reference | What it establishes | Language |
| --- | --- | --- |
| [Development verification](../verification/README.md) | Current regression/replay tools, test locations and required bundle inputs | English / 简体中文 |
| [Acceleration verification](../verification/acceleration/README.md) | Offline comparison tools, shared observer and separate ROS/DDS runtime replay | English / 简体中文 |
| [Main-stack integration](reports/2026-10-10-main-field-stack-integration.md) | Field-default parity, NX builds, installed providers and software checks | Chinese |
| [Latest field review](reports/2026-10-10-mpcc-field-review.md) | Run-by-run motion, speed, tracking, solver timing and joint CPU load | Chinese |
| [All dated reports](reports/README.md) | Earlier LIO, EKF, gyro bias, vehicle response, control and localization evidence | English / 简体中文 |

Field measurements retain their vehicle, map, configuration and timing definitions. Software checks and field driving are reported separately.

## Historical design notes

The following documents describe earlier branches and retired runtime designs:

- [Solver diagnostics and warm starts](../src/controller/docs/solver-diagnostics.md): earlier Python/IPOPT controller.
- [Backend comparison](../src/controller/docs/backend-comparison.md): earlier experimental IPOPT/acados/QP interfaces.
- [NX optimization](../src/controller/docs/nx-optimization.md): earlier Python-runtime optimization work.

Archived operating guides are indexed through [dated reports](reports/README.md). Use the installation and operation sections above for the current main stack.

## Attribution

[Model NOTICE](../src/controller/NOTICE.md) · [Native runtime NOTICE](../src/aims_mpcc_rt/NOTICE.md) · [Pinned dependency sources](../dependencies/manifest.json)

[Back to the project overview](../README.md)
