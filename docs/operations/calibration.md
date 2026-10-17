# Current and lateral-acceleration calibration

[简体中文](calibration.zh-CN.md) · [Project overview](../../README.md#calibration) · [Evidence index](../reports/assets/README.zh-CN.md)

Use this entry for current-response calibration, lateral-acceleration validation and lateral-grip history. Dated NX deployments are distinguished from the source committed on main. This documentation update adds no vehicle-control workflow or parameter changes.

| Task | Instructions | Evidence and status |
|---|---|---|
| Stage A: current needed to hold speed | [A/B/C environment and full-step protocol (Chinese)](full-step-current.zh-CN.md) | [Steady Iq/voltage study](../reports/2026-10-13-steady-current-voltage-model.md); real collection available |
| Stages B/C: acceleration and deceleration intervals | [Full-step acquisition (Chinese)](full-step-current.zh-CN.md) | [Selected models (Chinese)](../reports/2026-10-17-current-affine-selection.zh-CN.md), [collection coverage](../reports/2026-10-15-current-response-identification.md) |
| AY units, reference point and dynamic accuracy | [Lateral-acceleration calibration and validation](lateral-acceleration-calibration.md) | Rear/CG outputs and offline tools available; independent dynamic accuracy pending |
| Lateral grip, slip onset and maximum AY | [Lateral-grip status](lateral-acceleration-calibration.md#lateral-grip) | Fixed-radius stepped-speed procedure is historical; its old automatic script is not a current NX entry |
| Motion-capture state/actuator comparison | [Acquisition and analysis (Chinese)](mocap-validation.zh-CN.md) | [Delivery evidence (Chinese)](../reports/2026-10-18-mocap-validation.zh-CN.md); site clock semantics and marker extrinsics remain to be established |

<a id="current-calibration"></a>
## Current calibration

The deployed NX calibration runtime uses the separate `calibration-full-step` installation. Use its shared environment, frozen Stage-A baseline and map/reference, following the [full protocol](full-step-current.zh-CN.md). The deployed session helper is `~/run_calibration_full_step.sh B`; C uses a separate session. `inspect`, `start` and `stop` run in the same calibration environment. Launch and physical enable are separate, with `auto_start=false`. No commands were executed by this documentation update. [Native deployment evidence](../reports/2026-10-16-native-calibration.zh-CN.md) records the actual installation.

Use signed measured `avg_iq`. The selected B/C models describe interval-average acceleration as `a_bar=b+k·mean(avg_iq)`; do not subtract basic current again. B/C identify acquisition stages, not simply current signs. Coefficients, screening and held-out validation are in the [model-selection report](../reports/2026-10-17-current-affine-selection.zh-CN.md).

Holding-current characterization, wheel-response fitting, independent body-acceleration accuracy and a deployed acceleration controller are separate achievements. Small interval fit errors do not validate instantaneous body acceleration or high-current slip conditions.

<a id="lateral-calibration"></a>
## Lateral-acceleration calibration

Read the [lateral guide](lateral-acceleration-calibration.md) before selecting signals or interpreting results. Establish reference points, mounting and time semantics, then use the [motion-capture tools](mocap-validation.zh-CN.md). Computable AY is not proof of sensor calibration, independent dynamic accuracy or a measured tire-grip limit.

The [Chinese](../reports/2026-10-10-calibration-guide-archive.md) and [English](../reports/2026-10-10-calibration-guide-en-archive.md) historical tutorials retain their original workflows. The 2026-10-18 inspection found no usable `lateral_grip_calib` entry in the current NX installation. Do not interpret those retired commands as the current automatic lateral procedure.

[Selected experiments](../reports/latest-results.md) · [State outputs (Chinese)](state-observer.zh-CN.md) · [Historical AY quality](../reports/2026-10-15-planar-acceleration-quality.md) · [In-repository evidence](../reports/assets/README.zh-CN.md)
