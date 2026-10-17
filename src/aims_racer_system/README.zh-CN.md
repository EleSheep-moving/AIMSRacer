<a id="aimsracer-system-integration"></a>
# AIMSRacer 系统集成

**简体中文** · [English](README.md)

本包维护车辆 launch、后轴数据适配、gyro bias 校正、EKF/定位配置与已知地图初始化。当前生产入口使用一个 workspace：

| 入口 | 用途 |
| --- | --- |
| `vehicle.launch.py` | MID360、FAST-LIO2、后轴适配、EKF、RC/VESC |
| `mapping.launch.py` | 手动驾驶建图、PGO 与可选录包 |
| `race.launch.py` | vehicle、已知地图 NDT/monitor、native MPCC 与可选录包 |
| `longitudinal_acceleration.launch.py` | 在已有车辆图上单独运行轮速＋IMU 加速度观测器 |
| `state_observer.launch.py` | 同时发布后轴和质心两套独立 VX/VY、AX/AY、yaw rate |

[后轴与质心五量](../../docs/operations/state-observer.zh-CN.md)通过 `state_observer.launch.py` 同时输出两点的 VX/VY、AX/AY、yaw rate 和分量有效性。Vehicle/race 使用 `state_observer:=true` 显式启用（默认 false）；旧话题保留。质心高度 −0.03 m 是用户估计，几何与动态精度尚未独立验收。

[启动](../../docs/operations/bringup.zh-CN.md)描述源码准备和构建，[架构](../../docs/architecture.zh-CN.md)统一 topics、TF、时间与命令所有权。

<a id="known-map-operation"></a>
## 已知地图运行

```bash
ros2 launch aims_racer_system race.launch.py \
  map_file:="$HOME/maps/20260928_010503/map.pcd" \
  artifact_directory:="$HOME/aimsracer-data/bundles/field-v35-20261010" \
  record:=true
```

`map_file`、`artifact_directory` 必填；`initial_pose` 可选，格式为 map 中 **base_link** 的 `x y z roll pitch yaw`，角度 rad。省略时通过工具或 RViz 初始化：

```bash
ros2 run aims_racer_system relocalize_known_map.py \
  "$HOME/maps/20260928_010503/map.pcd" --pose-frame base_link \
  --x 0 --y 0 --z 0 --roll 0 --pitch 0 --yaw 0
```

示例值须替换为实际后轴摆放。[已知地图指南](../../docs/operations/known-map-mpcc.zh-CN.md)解释 epoch、map identity 和资格。`auto_start=false` 默认手动 enable；true 由 native 节点就绪后启用一次，显式 stop 取消等待。`repeat_laps=false` 默认单圈停车，true 连续圈直到 stop。完成/故障不自动重启或再次启用。


## 录制与估计

`record=true` 自动记录 raw/corrected IMU、LIO、wheel/EKF、TF、NDT/MPCC 状态及已发布的可选加速度估计/状态，race 同时保存 runtime。`session_directory` 留空自动生成日期会话，指定时须新目录；完整列表及 reliable/depth=4096 原始 IMU QoS 见[录制](../../docs/operations/recording.zh-CN.md)。

`base_link` 是后轴中点。C++ `imu_to_rear_axle` 保留 IMU source stamp 和不可用信息，转换单位并做杠杆补偿；`lio_to_rear_axle` 提供同一点的 odometry。EKF 融合 LIO、wheel vx 与 IMU yaw rate，保持 2 s 延迟历史。gyro 校正保留独立原始与修正 topic。`livox_gyro_bias`、`lio_to_rear_axle` 和可选 `longitudinal_acceleration` 现均为 C++，见[2026-10-15 迁移及 NX 验证](../../docs/reports/2026-10-15-native-state-nodes.zh-CN.md)。

`/rear_axle/lio_odom` 保留 LIO scan-end 时间戳，用源时刻匹配或插值的 gyro；`/rear_axle/imu` 随最新 IMU 到达发布，不依赖 LIO 延迟，输出比力 `f = a − g`，姿态标为不可用。z 轴向上的世界中 `g = [0,0,−9.80665] m/s²`；恢复惯性加速度需在同一坐标系使用 `a = f + g`。没有姿态外推，也不新增对齐 IMU 话题。见[时间戳修正报告](../../docs/reports/2026-10-11-rear-imu-source-time.zh-CN.md)。

body cloud 与 raw LIO odometry 保留相同 scan-end stamp，供 NDT/PGO 使用；world cloud/path 为可选显示输出。race 中 EKF 拥有 `odom → base_link`，NDT 拥有 `map → odom`；mapping 中由后轴 LIO 与 PGO 分别拥有这两条边。

## 可选纵向加速度

场地验证时给 vehicle/race 添加 `longitudinal_acceleration:=true`，默认 `false`。车辆图已运行时，可只启动观测器：

```bash
ros2 launch aims_racer_system longitudinal_acceleration.launch.py
```

选择集成或独立启动，只运行一个实例。节点输入轮速 vx 和后轴 IMU 比力，发布 `/state_estimation/longitudinal_acceleration` 与 `/state_estimation/longitudinal_acceleration/status`。输出 linear.x 是 `d(body_vx)/dt`，协方差是模型不确定性，尚未用场地真值标定。没有 LIO 位姿预测、命令或 TF 发布，导航 EKF 不融合该输出。源年龄告警只用于诊断。初始化、时间戳及两圈录制见[操作指南](../../docs/operations/longitudinal-acceleration.zh-CN.md)。

<a id="build-and-evidence"></a>
## 构建与证据

```bash
source /opt/ros/humble/setup.bash
export MAKEFLAGS="-j2 -l2"
colcon build --packages-up-to aims_racer_system aims_mpcc_rt --executor sequential \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DROS_EDITION=ROS2 -DDISTRO_ROS=humble
source install/setup.bash
```

固定依赖准备见[根 README](../../README.zh-CN.md)。软件验证放在 `verification/`；本包的[校准说明](docs/calibration.zh-CN.md)和日期化[外场报告](../../docs/reports/2026-10-10-mpcc-field-review.md)保留具体条件。当前默认 v35 bundle 的规划峰值约 2.99 m/s、实车峰值约 2.89 m/s；source、构建、软件检查与实跑证据分别解释。

平面 G 可选入口：`planar_acceleration:=true`（默认 false）；[操作与已知质量局限](../../docs/operations/planar-acceleration.zh-CN.md)。

## 电流与横向加速度标定导航

[电流标定 A/B/C 与当前结果](../../docs/operations/calibration.zh-CN.md#current-calibration) · [横向加速度 AY 校对及附着标定状态](../../docs/operations/lateral-acceleration-calibration.zh-CN.md)。源码统一在 `src/calibration/`，横向使用当前 C++ 入口；历史 Python 命令仅供溯源。

[当前标定包](../calibration/README.zh-CN.md) · [独立状态／Vicon 现场入口](field/README.zh-CN.md)
