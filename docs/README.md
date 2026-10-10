# Documentation

**English** · [简体中文](README.zh-CN.md) · [Project overview](../README.md)

Guides for the current **FAST-LIO2 + rear-axle EKF + NDT + native MPCC** stack. The overview and this index are bilingual; detailed guides retain the languages listed below.

## Suggested reading order

1. **New deployment:** [system dependencies](deployment/README.md) → [target platform](deployment/orin.md) → [vehicle launch](operations/bringup.md).
2. **First known-map run:** [frames and topics](architecture.md) → [map/reference preparation](../src/controller/docs/usage.md) → [initialization and enable](operations/known-map-mpcc.md).
3. **Controller development:** [implementation](../src/controller/docs/implementation.md) → [native runtime](../src/aims_mpcc_rt/README.md) → [regressions and replay](../verification/README.md).
4. **Analyze a run:** [recording](operations/recording.md) → [field review](reports/2026-10-10-mpcc-field-review.md) → [dated reports](reports/README.md).

## Installation and hardware

| Guide | What it covers | Language |
| --- | --- | --- |
| [Deployment](deployment/README.md) | System packages, pinned sources, SDK preparation, bounded builds, bundle data | Chinese |
| [Orin NX](deployment/orin.md) | Vehicle devices, MID360 network, native deployment | Chinese |
| [x86 / NUC](deployment/nuc.md) | Same source stack on x86; target-specific bundle compilation | Chinese |
| [Device rules](../rules/README.md) | Stable device names and serial-port ownership | English |
| [Dependency manifest](../dependencies/manifest.json) | Exact upstream commits, patches and acados architecture settings | JSON |

## Vehicle operation and data

| Guide | What it covers | Language |
| --- | --- | --- |
| [Launch guide](operations/bringup.md) | Choose vehicle, mapping, race or controller-only launch | Chinese |
| [Known-map MPCC](operations/known-map-mpcc.md) | Map and bundle identity, rear-axle initial pose, enable, stop and repeated laps | Chinese |
| [Recording](operations/recording.md) | Launch-managed bags, runtime logs, QoS, session layout and timing definitions | Chinese |
| [Vehicle checklist](operations/vehicle-checklist.md) | Device, localization, authority and run checks | Chinese |
| [Reference and bundle workflow](../src/controller/docs/usage.md) | Map-frame reference, speed planning, export and target-CPU build | Chinese |
| [Map/reference tools](../tools/reference/README.md) | Save a PGO map and recover a matching closed reference | Chinese |
| [Calibration context](../src/aims_racer_system/docs/calibration.md) · [English entry](../src/aims_racer_system/docs/calibration.en.md) | Existing response evidence, current workflows and archived collection guide | Chinese / English |

Maps, reference inputs, generated bundles and raw bags are external data. A repository clone supplies source and reports, not all recorded data or a runnable vehicle bundle.

## Architecture, control and diagnostics

| Guide | What it covers | Language |
| --- | --- | --- |
| [System architecture](architecture.md) | TF ownership, rear-axle frames, topics, timestamps, estimation and control flow | Chinese |
| [MPCC implementation](../src/controller/docs/implementation.md) | Vehicle model, costs, geometry, speed profile and execution history | Chinese |
| [Native controller](../src/aims_mpcc_rt/README.md) | C++ runtime, immutable bundles, launch parameters and diagnostics | Chinese |
| [Offline reference tools](../src/controller/README.md) | `aims_mpcc` package and its offline responsibilities | Chinese |
| [Localization monitor](localization_monitor.md) | Trusted anchors, freshness, epoch handling and background quality diagnostics | Chinese |
| [RC selector](../src/ackermann_mux/README.md) | RC channels, manual/autonomous arbitration, command bounds and timeouts | English |
| [Vehicle package](../src/aims_racer_system/README.md) | Launch composition and sensor/estimator adapters | Chinese |
| [Installed system helpers](../src/aims_racer_system/scripts/README.md) | Gyro correction, rear-axle conversion and NDT initialization | Chinese |

## Verification and measured evidence

| Reference | What it establishes | Language |
| --- | --- | --- |
| [Development verification](../verification/README.md) | Current regression/replay tools, test locations and required bundle inputs | Chinese |
| [Main-stack integration](reports/2026-10-10-main-field-stack-integration.md) | Field-default parity, NX builds, installed providers and software checks | Chinese |
| [Latest field review](reports/2026-10-10-mpcc-field-review.md) | Run-by-run motion, speed, tracking, solver timing and joint CPU load | Chinese |
| [All dated reports](reports/README.md) | Earlier LIO, EKF, gyro bias, vehicle response, control and localization evidence | Mixed |

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
