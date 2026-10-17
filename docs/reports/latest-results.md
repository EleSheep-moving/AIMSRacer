# Selected experiment results — through 2026-10-18

[Current calibration entry](../operations/calibration.md) · [Lateral-acceleration calibration / AY validation](../operations/lateral-acceleration-calibration.md): procedures, results and retired grip-entry status.

[Evidence index](assets/README.zh-CN.md)：所有已上传附件均可在仓库内浏览；来源记录中的原始路径不作为附件链接。

[简体中文：完整精选与结论](latest-results.zh-CN.md) · [Report archive](README.md)

This publication collects reports, figures and small evidence files from the host and NX. It changes documentation only. New estimator, motion-capture, calibration and localization implementations still present in the local workspaces are not included in this documentation commit. Deployment claims refer to the dated NX installation snapshots; a fresh clone does not necessarily include those implementations.

Read these groups first (recent detailed reports are in Chinese):

- **Independent state outputs:** [VY quality and real-bag plots](2026-10-18-vy-quality.zh-CN.md), [NX deployment](2026-10-18-state-observer-nx-deployment.zh-CN.md), [calculation and warning-first policy](2026-10-17-state-observer-latency-policy.zh-CN.md). Rear-axle and CG outputs are published; independent dynamic accuracy remains unvalidated. MPCC still consumes navigation VX.
- **Motion capture:** [delivery and verification](2026-10-18-mocap-validation.zh-CN.md), [field instructions](../operations/mocap-validation.zh-CN.md), [explicitly synthetic demonstration](assets/2026-10-18-mocap-validation/demo/report.zh-CN.md). Site time semantics, clock mapping and marker extrinsics must be established before independent acceptance.
- **Current calibration:** [selected affine B/C interval models](2026-10-17-current-affine-selection.zh-CN.md), [collection and wheel-response evidence](2026-10-15-current-response-identification.md), [native deployed calibration chain](2026-10-16-native-calibration.zh-CN.md). Interval fit errors are not instantaneous body-acceleration accuracy or controller acceptance.
- **Control and localization:** [15-lap field review](2026-10-15-mpcc-multilap-review.md), [steering model](2026-10-14-imu-steering-model.md), [EKF component response](2026-10-14-ekf-component-latency.md), [FastLIO scale replay](2026-10-17-fastlio-accel-scale.zh-CN.md), [NX CPU diagnosis](2026-10-17-localization-cpu.zh-CN.md), [5 Hz NDT deployment](2026-10-17-ndt-5hz.zh-CN.md).

Source/build tests, replay agreement, waveform alignment and independent physical validation remain separate. Raw bags, full replay arrays, maps, bundles and build environments remain in external `aimsracer-data`. Repository figures and small JSON/CSV receipts can be inspected without those datasets.
