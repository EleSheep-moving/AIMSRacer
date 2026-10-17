# 当前代码文档

[English](README.md) · [简体中文](README.zh-CN.md) · [AIMSRacer](../README.zh-CN.md)

这里按当前仓库可构建的包与操作入口导航。已替代设计和 Ood 资料保留原文件，不再列入当前索引。日期化报告只作为当前功能的证据，不能替代源码默认值。

| 任务 | 当前入口 |
|---|---|
| 安装与平台 | [Deployment](deployment/README.zh-CN.md) · [NX](deployment/orin.zh-CN.md) · [x86](deployment/nuc.zh-CN.md) |
| 车辆操作 | [Bringup](operations/bringup.zh-CN.md) · [Known-map MPCC](operations/known-map-mpcc.zh-CN.md) · [Recording](operations/recording.zh-CN.md) |
| 系统接口 | [Architecture](architecture.zh-CN.md) · [Localization](localization_monitor.zh-CN.md) · [VESC](vesc-command.zh-CN.md) |
| 后轴／质心五量 | [VX/VY/AX/AY/yaw rate](operations/state-observer.zh-CN.md) · [现场脚本](../src/aims_racer_system/field/README.zh-CN.md) |
| 纵向电流 | [A/B/C](operations/calibration.zh-CN.md#current-calibration) · [完整协议](operations/full-step-current.zh-CN.md) · [拟合／查询](../src/calibration/longitudinal_current/docs/wheel-current-model.zh-CN.md) |
| 横向附着／AY | [Current lateral tool](../src/calibration/lateral_grip/README.zh-CN.md) · [AY validation](operations/lateral-acceleration-calibration.zh-CN.md) |
| 动捕校对 | [配置与采集](operations/mocap-validation.zh-CN.md) · [源码](../src/calibration/mocap_validation/README.zh-CN.md) |
| 地图／参考编辑 | [Reference tools](../tools/reference/README.zh-CN.md) · [Editor](../tools/reference/editor/README.zh-CN.md) · [Bundle workflow](../src/controller/docs/usage.zh-CN.md) |
| 控制器与验证 | [Native MPCC](../src/aims_mpcc_rt/README.zh-CN.md) · [Implementation](../src/controller/docs/implementation.zh-CN.md) · [Verification](../verification/README.zh-CN.md) |
| 当前证据 | [Current results](reports/latest-results.zh-CN.md) · [相关报告](reports/README.zh-CN.md) · [仓库内附件](reports/assets/README.zh-CN.md) |

[本次源码归并与发布范围](reports/2026-10-18-code-consolidation-audit.zh-CN.md)：离线电流模型、参考编辑器、估计器／Vicon 现场脚本及必要依赖；CPU 采集工具不纳入仓库。
