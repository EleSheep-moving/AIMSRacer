# Current code documentation

[English](README.md) · [简体中文](README.zh-CN.md) · [AIMSRacer](../README.md)

Navigation follows packages and entrypoints in the current source tree. Superseded designs and Ood files are retained without current-navigation links. Dated evidence does not override source defaults.

| Task | Current entry |
|---|---|
| Installation | [Deployment](deployment/README.md) · [NX](deployment/orin.md) · [x86](deployment/nuc.md) |
| Vehicle operation | [Bringup](operations/bringup.md) · [Known-map MPCC](operations/known-map-mpcc.md) · [Recording](operations/recording.md) |
| System interfaces | [Architecture](architecture.md) · [Localization](localization_monitor.md) · [VESC](vesc-command.md) |
| 后轴／质心五量 | [VX/VY/AX/AY/yaw rate](operations/state-observer.zh-CN.md) · [现场脚本](../src/aims_racer_system/field/README.zh-CN.md) |
| 纵向电流 | [A/B/C](operations/calibration.md#current-calibration) · [完整协议](operations/full-step-current.zh-CN.md) · [拟合／查询](../src/calibration/longitudinal_current/docs/wheel-current-model.zh-CN.md) |
| 横向附着／AY | [Current lateral tool](../src/calibration/lateral_grip/README.md) · [AY validation](operations/lateral-acceleration-calibration.md) |
| 动捕校对 | [配置与采集](operations/mocap-validation.zh-CN.md) · [源码](../src/calibration/mocap_validation/README.zh-CN.md) |
| 地图／参考编辑 | [Reference tools](../tools/reference/README.md) · [Editor](../tools/reference/editor/README.md) · [Bundle workflow](../src/controller/docs/usage.md) |
| 控制器与验证 | [Native MPCC](../src/aims_mpcc_rt/README.md) · [Implementation](../src/controller/docs/implementation.md) · [Verification](../verification/README.md) |
| 当前证据 | [Current results](reports/latest-results.md) · [相关报告](reports/README.md) · [仓库内附件](reports/assets/README.zh-CN.md) |

[本次源码归并与发布范围](reports/2026-10-18-code-consolidation-audit.zh-CN.md)：离线电流模型、参考编辑器、估计器／Vicon 现场脚本及必要依赖；CPU 采集工具不纳入仓库。
