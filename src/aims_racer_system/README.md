# AIMSRacer 系统集成

本包维护车辆 launch、后轴数据适配、gyro bias 校正、EKF/定位配置与已知地图初始化。当前生产入口使用一个 workspace：

| 入口 | 用途 |
| --- | --- |
| `vehicle.launch.py` | MID360、FAST-LIO2、后轴适配、EKF、RC/VESC |
| `mapping.launch.py` | 手动驾驶建图、PGO 与可选录包 |
| `race.launch.py` | vehicle、已知地图 NDT/monitor、native MPCC 与可选录包 |

[启动](../../docs/operations/bringup.md)描述源码准备和构建，[架构](../../docs/architecture.md)统一 topics、TF、时间与命令所有权。

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

示例值须替换为实际后轴摆放。[已知地图指南](../../docs/operations/known-map-mpcc.md)解释 epoch、map identity 和资格。`auto_start=false` 默认手动 enable；true 由 native 节点就绪后启用一次，显式 stop 取消等待。`repeat_laps=false` 默认单圈停车，true 连续圈直到 stop。完成/故障不自动重启或再次启用。

## 录制与估计

`record=true` 自动记录 raw/corrected IMU、LIO、wheel/EKF、TF、NDT/MPCC 状态，race 同时保存 runtime。`session_directory` 留空自动生成日期会话，指定时须新目录；完整列表及 reliable/depth=4096 原始 IMU QoS 见[录制](../../docs/operations/recording.md)。

`base_link` 是后轴中点。C++ `imu_to_rear_axle` 保留 IMU source stamp 和不可用信息，转换单位并做杠杆补偿；`lio_to_rear_axle.py` 提供同一点的 odometry。EKF 融合 LIO、wheel vx 与 IMU yaw rate，保持 2 s 延迟历史。gyro 校正保留独立原始与修正 topic。

body cloud 与 raw LIO odometry 保留相同 scan-end stamp，供 NDT/PGO 使用；world cloud/path 为可选显示输出。race 中 EKF 拥有 `odom → base_link`，NDT 拥有 `map → odom`；mapping 中由后轴 LIO 与 PGO 分别拥有这两条边。

## 构建与证据

```bash
source /opt/ros/humble/setup.bash
export MAKEFLAGS="-j2 -l2"
colcon build --packages-up-to aims_racer_system aims_mpcc_rt --executor sequential \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DROS_EDITION=ROS2 -DDISTRO_ROS=humble
source install/setup.bash
```

固定依赖准备见[根 README](../../README.md)。软件验证放在 `verification/`；本包的[校准说明](docs/calibration.md)和日期化[外场报告](../../docs/reports/2026-10-10-mpcc-field-review.md)保留具体条件。当前默认 v35 bundle 的规划峰值约 2.99 m/s、实车峰值约 2.89 m/s；source、构建、软件检查与实跑证据分别解释。
