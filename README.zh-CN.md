# AIMSRacer

![AIMSRacer](docs/assets/aimsracer-banner.svg)

[English](README.md) · [简体中文](README.zh-CN.md)

**PolyU AIMS Lab** 的 ROS 2 无人赛车栈：FAST-LIO2、后轴 EKF、NDT 已知地图定位与原生 acados/C++ MPCC。

Orin NX 16 GB · Ubuntu 22.04 · ROS 2 Humble · KKPIT ZQR 1/7 · Livox MID360 · VESC · ELRS

<a id="navigation"></a>
## 当前任务入口

| 任务 | 当前文档／源码 |
|---|---|
| 安装与构建 | [Deployment](docs/deployment/README.zh-CN.md) · [NX](docs/deployment/orin.zh-CN.md) |
| 驾驶、建图、MPCC | [Bringup](docs/operations/bringup.zh-CN.md) · [Known-map operation](docs/operations/known-map-mpcc.zh-CN.md) |
| 后轴／质心 VX、VY、AX、AY、yaw rate | [计算、话题与启动](docs/operations/state-observer.zh-CN.md) · [现场脚本](src/aims_racer_system/field/README.zh-CN.md) |
| 纵向电流 A/B/C | [操作指南](docs/operations/calibration.zh-CN.md#current-calibration) · [longitudinal_current](src/calibration/longitudinal_current/README.zh-CN.md) · [离线拟合／查询](src/calibration/longitudinal_current/docs/wheel-current-model.zh-CN.md) |
| 自动横向附着标定 | [lateral_grip](src/calibration/lateral_grip/README.zh-CN.md) · [AY 校对](docs/operations/lateral-acceleration-calibration.zh-CN.md) |
| 动捕校对：状态／FastLIO／执行器 | [mocap_validation](src/calibration/mocap_validation/README.zh-CN.md) · [现场配置](docs/operations/mocap-validation.zh-CN.md) |
| 地图与参考轨迹编辑 | [Reference tools](tools/reference/README.zh-CN.md) · [Editor](tools/reference/editor/README.zh-CN.md) |
| 录制与软件验证 | [Recording](docs/operations/recording.zh-CN.md) · [Verification](verification/README.zh-CN.md) |
| 全部当前指南 | [Docs](docs/README.zh-CN.md) |

<a id="quick-start"></a>
## 构建与启动

先按部署指南安装系统依赖，准备固定源码与 acados。每个终端加载同一工作区：

```bash
source /opt/ros/humble/setup.bash
export MAKEFLAGS="-j2 -l2"
colcon build --packages-up-to aims_racer_system aims_mpcc_rt --executor sequential \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DROS_EDITION=ROS2 -DDISTRO_ROS=humble
src/calibration/build.sh
source install/setup.bash
```

车辆启动还需实际地图、参考／bundle、设备配置；源码仓库不包含这些现场输入。具体命令见任务入口。

<a id="calibration"></a>
## 标定与动捕

三个目录统一在 [src/calibration](src/calibration/README.zh-CN.md)：`longitudinal_current`、`lateral_grip`、`mocap_validation`。在线标定与估计持续节点为 C++；启动配置和离线分析使用 Python。横纵向共用标定指令通道，不同时运行；默认不自动启用。离线轮速电流模型不会自动替换当前 B/C 模型。

<a id="architecture"></a>
## 接口与职责

`base_link` 位于后轴中点；TF 为 `map → odom → base_link → livox_frame`。独立五量估计器只读取 corrected IMU 与 raw LIO 位姿，发布后轴和质心两套输出。Robot Localization 与 MPCC 现有输入接线保持原样，MPCC 仍读取导航 VX。详见[架构](docs/architecture.zh-CN.md)及[原生控制器](src/aims_mpcc_rt/README.zh-CN.md)。

<a id="defaults"></a>
实际默认值以 [vehicle.yaml](src/controller/config/vehicle.yaml)、[系统参数](src/aims_racer_system/params/)和当前构建的 launch 为准；NX 的场地配置与已生成 bundle 需要单独核对。

<a id="results"></a>
[当前相关结果与待验收项](docs/reports/latest-results.zh-CN.md)：五量动态精度和实际摩擦上限仍待独立动捕／实车验证。旧运行时和已替代教程不再作为操作索引。

<a id="credits"></a>
## 致谢与许可证

第三方归属和许可证保存在[模型 NOTICE](src/controller/NOTICE.zh-CN.md)、[原生运行时 NOTICE](src/aims_mpcc_rt/NOTICE.zh-CN.md)及各包中。依赖版本与补丁见[固定清单](dependencies/manifest.json)。

本项目基于以下开源贡献：

- [ForzaETH Race Stack](https://github.com/ForzaETH/race_stack)
- [QUTMS Driverless](https://github.com/QUT-Motorsport/QUTMS_Driverless)
- [Original upstream system package](https://github.com/f1tenth/f1tenth_system)
- [ros2_crsf_receiver](https://github.com/AndreyTulyakov/ros2_crsf_receiver.git)
- [ackermann_mux](https://github.com/z1047941150/ackermann_mux.git)
- [Veddar VESC Interface](https://github.com/f1tenth/vesc)
- [FAST-LIO2_ROS2 maintained fork](https://github.com/EleSheep-moving/FASTLIO2_ROS2.git)

硬件与基础软件由 PolyU AIMS Lab 开发。目前在 PolyU AIMS Lab 攻读 MPhil，项目持续开发中。
