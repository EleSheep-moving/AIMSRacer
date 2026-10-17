# 独立五量估计器：NX 部署记录

2026-10-18 已部署并验证。在线节点为 ARM aarch64 C++ `state_observer`，同时提供后轴和质心的 VX、VY、AX、AY、车体 omega_z；Vicon 采集／分析程序也已同步。部署没有启动整车或发送驾驶指令，回放进程已经退出。

同日后续更新：按用户要求退役额外测试用 HI13 IMU，已移除 NX 主工作区及实际安装中的可选启动、参数入口和录包接入；主机原本已无此接入。12 项启动测试及实际安装检查通过，Livox IMU、五量估计器和 MPCC 速度参数透传保留，历史测试数据未删除。[移除核验回执](assets/2026-10-18-state-observer-nx/hi13-retirement-verification.json)。

## 现场使用

实际主工作区是 `/home/aims/AIMSRacer`，安装前缀是 `/home/aims/AIMSRacer/install/aims_racer_system`。已有 corrected IMU 和 raw LIO 传感器链运行时，只启动估计器：

```bash
~/run_state_observer.sh
```

前台 Ctrl-C 停止。若通过整车入口启动，则加 `state_observer:=true`，不再另启一份观测器：

```bash
~/run_mpcc_imu_fit.sh state_observer:=true
```

保留原地图、bundle、速度参数和 `auto_start:=false`。上述整车入口未在本次部署过程中执行；MPCC 继续使用原有状态输入。

另一个终端查看两套速度及状态：

```bash
source ~/state_observer_env.bash
ros2 topic echo /state_estimation/rear_axle/velocity --once
ros2 topic echo /state_estimation/center_of_mass/velocity --once
ros2 topic echo /state_estimation/center_of_mass/status --once
```

同一前缀下还有 `acceleration` 和 `yaw_rate`。数值字段、单位和计算链见[操作说明](../operations/state-observer.zh-CN.md)。警告优先策略已启用；IMU→质心偏移为 `[-0.09,-0.03,-0.03] m`，高度仍标记为估计值，外参及动态精度未标记为已验证。

## 验证结果

| 检查 | 结果 |
|---|---|
| NX 原生 Release 编译 | 成功，ELF 为 ARM aarch64 |
| NX CTest | 30/30 组通过 |
| NX Vicon 工具测试 | 19/19 通过，包含 DDS→rosbag→提取 |
| 从主安装启动的完整 1× 实包回放 | 221.929989576 s，通过接口、源时间和参考点恒等式检查 |
| 每个参考点速度消息 | 42,255 条，包含 VX/VY |
| 每个参考点加速度消息 | 42,293 条，包含 AX/AY |
| 每个参考点 yaw rate 消息 | 42,313 条 |
| 两点速度刚体换算最大残差 | 8.64×10⁻¹⁶ m/s |
| 两点 yaw rate 最大差异 | 0 rad/s |
| 受保护运行文件 | 174 个部署前后哈希一致 |

回放输入是 2026-10-15 `022604_194643_race` 实车录包的 42,315 条 corrected IMU 和 2,229 条 raw LIO。使用 `ROS_DOMAIN_ID=204`、`ROS_LOCALHOST_ONLY=1`，仅启动被测观测器和录包输入播放器。ROS 通信图检查无 TF／控制命令发布，也不订阅导航 EKF 或轮速。

通过 `/proc/<pid>/exe` 确认实际进程来自主安装，实际 ELF SHA256 为：

```text
5d4564ad02a99afaa680a9beae16a6e4a506559f4d17951f33358b94d89eeb3d
```

该哈希与验证构建一致；动态链接库全部可解析，没有依赖暂存工作区的库。实际参数 dump 确认 `warn_only_quality_gates=true`、`use_sim_time=false` 和上述几何配置。

首次部署时合并保留了 NX 现场的 HI13 驱动／录制入口和 MPCC `target_speed` 透传，并通过测试；随后按上述更新移除了 HI13，继续保留 `target_speed`。导航 EKF 参数、FastLIO、MPCC、VESC 驱动和原有整车脚本保持校验一致；原安装中的 12 份参数文件与部署前备份逐字节一致。独立标定工作区的安装未在此次部署中切换。

## Vicon 准备

配置副本已经建立：

```text
/home/aims/aimsracer-data/sessions/2026-10-19/vicon-state-validation/config.capture.json
```

目前保持未填写模板，接入话题／类型／跟踪语义需现场确定；分析还需独立核对时钟与 marker 外参。填写接入配置后：

```bash
source ~/state_observer_env.bash
~/vicon_state.sh inspect --config "$AIMS_VICON_CONFIG"
~/vicon_state.sh record --config "$AIMS_VICON_CONFIG" \
  --output /home/aims/aimsracer-data/sessions/2026-10-19/vicon-state-validation/run01
```

录包目录必须不存在，Ctrl-C 正常落盘。使用此专用录包时，原整车入口追加 `record:=false`。完整工具说明见 Vicon README（记录中的工作区路径：`verification/vicon_state/README.zh-CN.md`）。

## 证据位置及边界

- [部署回执](assets/2026-10-18-state-observer-nx/completion.json)、[安装路径及运行文件核验](assets/2026-10-18-state-observer-nx/deployed-verification.json)。
- [NX CTest 日志](assets/2026-10-18-state-observer-nx/nx-ctest.log)、[Vicon 工具测试日志](assets/2026-10-18-state-observer-nx/nx-vicon-tests.log)、[完整回放摘要及实际参数／二进制哈希](assets/2026-10-18-state-observer-nx/full-replay-summary.json)。
- NX 完整证据：`/home/aims/aimsracer-data/experiments/state-observer-deployment-20261018/`，包含旧源码／安装备份 `main-before-deploy.tgz`、源文件清单、部署差异和完整输出 CSV。
- 本机证据镜像：`/home/elesheep/aimsracer-data/maintenance/2026-10-18-state-observer-nx/`。
- NX 简短现场说明：`/home/aims/aimsracer-data/code/state-observer-20261018/field/README.zh-CN.md`。

这些结果证明当前 ARM 安装及软件输出链可用。此次回放没有整车联合负载，也没有独立 Vicon 真值，不构成动态精度、物理延迟或闭环控制验收。
