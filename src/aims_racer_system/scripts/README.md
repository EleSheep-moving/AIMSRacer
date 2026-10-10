# 系统脚本

当前车辆通过[单 workspace 启动入口](../../../docs/operations/bringup.md)运行。Python 脚本由包显式安装，下面列出主流程使用的脚本；C++ `imu_to_rear_axle` 和 `localization_monitor` 属于本包编译的节点。

| 脚本 | 用途 |
| --- | --- |
| `livox_gyro_bias.py` | 输出 gyro bias 修正 IMU 与诊断，原始 IMU 保留 |
| `lio_to_rear_axle.py` | raw LIO 转为后轴 odometry；mapping 时拥有 odom/base_link TF |
| `activate_ndt.py` | 配置并激活 NDT lifecycle |
| `relocalize_known_map.py` | 用明确 map/base_link 六维初值初始化并等待可信新 epoch |

已知地图初始化使用安装入口：

```bash
ros2 run aims_racer_system relocalize_known_map.py \
  "$HOME/maps/20260928_010503/map.pcd" --pose-frame base_link \
  --x 0 --y 0 --z 0 --roll 0 --pitch 0 --yaw 0
```

数值须替换为实际后轴位姿，角度为 rad。也可用 RViz；完整说明见[已知地图](../../../docs/operations/known-map-mpcc.md)。定位 replay 与开发审计位于根 [verification](../../../verification/README.md)，不安装为生产入口。

地图保存使用 [tools/save_map.sh](../../../tools/save_map.sh)；闭环 CSV 与保存 PGO 地图的关联使用 [tools/reference](../../../tools/reference/README.md)。历史校准采集和响应证据见[校准资料](../docs/calibration.md)；该页链接归档和当前操作指南。
