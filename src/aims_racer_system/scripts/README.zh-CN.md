<a id="system-scripts"></a>
# 系统脚本

**简体中文** · [English](README.md)

当前车辆通过[单 workspace 启动入口](../../../docs/operations/bringup.zh-CN.md)运行。下表列出安装的辅助入口。持续运行的 gyro 修正、后轴里程计及加速度观测器已于 2026-10-15 迁移为 C++；`imu_to_rear_axle` 和 `localization_monitor` 原本就是 C++。见[迁移与验证](../../../docs/reports/2026-10-15-native-state-nodes.zh-CN.md)。保留的 Python 参考包装不再安装为运行节点。

| 脚本 | 用途 |
| --- | --- |
| `livox_gyro_bias` (C++) | 输出 gyro bias 修正 IMU 与诊断，原始 IMU 保留 |
| `lio_to_rear_axle` (C++) | raw LIO 转为后轴 odometry；mapping 时拥有 odom/base_link TF |
| `longitudinal_acceleration` (C++) | 可选轮速＋IMU 加速度/状态发布器，原生观测器保留 Python 数值参考 |
| `activate_ndt.py` | 配置并激活 NDT lifecycle |
| `relocalize_known_map.py` | 用明确 map/base_link 六维初值初始化并等待可信新 epoch |

加速度验证时给 vehicle/race 添加 `longitudinal_acceleration:=true`（默认 `false`），或在已有车辆图上运行 `ros2 launch aims_racer_system longitudinal_acceleration.launch.py`。独立 launch 只启动该观测器，只运行一个实例；不发布命令或 TF，导航 EKF 不融合其输出。见[加速度指南](../../../docs/operations/longitudinal-acceleration.zh-CN.md)与[离线/运行期验证](../../../verification/acceleration/README.zh-CN.md)。

已知地图初始化使用安装入口：

```bash
ros2 run aims_racer_system relocalize_known_map.py \
  "$HOME/maps/20260928_010503/map.pcd" --pose-frame base_link \
  --x 0 --y 0 --z 0 --roll 0 --pitch 0 --yaw 0
```

数值须替换为实际后轴位姿，角度为 rad。也可用 RViz；完整说明见[已知地图](../../../docs/operations/known-map-mpcc.zh-CN.md)。定位 replay 与开发审计位于根 [verification](../../../verification/README.zh-CN.md)，不安装为生产入口。

地图保存使用 [tools/save_map.sh](../../../tools/save_map.sh)；闭环 CSV 与保存 PGO 地图的关联使用 [tools/reference](../../../tools/reference/README.zh-CN.md)。历史校准采集和响应证据见[校准资料](../docs/calibration.zh-CN.md)；该页链接归档和当前操作指南。

## 电流与横向加速度标定导航

[电流标定 A/B/C 与当前结果](../../../docs/operations/calibration.zh-CN.md#current-calibration) · [横向加速度 AY 校对及附着标定状态](../../../docs/operations/lateral-acceleration-calibration.zh-CN.md)。源码统一在 `src/calibration/`，横向使用当前 C++ 入口；历史 Python 命令仅供溯源。
