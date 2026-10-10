# AIMSRacer 参考与离线模型工具

ROS 包名 `aims_mpcc`，源码目录为 `src/controller`。它提供 `record_path`、`prepare_path`，并保存 native acados bundle 生成所需的车辆配置、周期参考、模型和速度规划工具。在线控制由 [aims_mpcc_rt](../aims_mpcc_rt/README.md) 的 C++ 节点承担；整车使用 [race.launch.py](../aims_racer_system/launch/race.launch.py)。

当前默认 [vehicle.yaml](config/vehicle.yaml)与 2026-10-10 后期实跑一致：巡航 3.5 m/s、硬上限 4 m/s、加速/制动 1 m/s²、规划 ay=1 m/s²、RTI 至多两次；bundle 使用 N15/dt=.1。车体参考为后轴 `base_link`，footprint 前/后/半宽为 0.52/0.10/0.16 m。`enforce_corridor=false`、组合加速度椭圆关闭；保存参考宽度不是实际赛道边界。

实跑规划峰值约 2.99 m/s、实测约 2.89 m/s，参见[外场复盘](../../docs/reports/2026-10-10-mpcc-field-review.md)。

## 使用

- [准备参考、导出 bundle 和运行](docs/usage.md)
- [模型、成本与 native 执行实现](docs/implementation.md)
- [系统坐标、topics 和命令归属](../../docs/architecture.md)
- [已知地图初始化](../../docs/operations/known-map-mpcc.md)

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run aims_mpcc record_path --ros-args -p output:="$HOME/aimsracer-data/lap.csv"
ros2 run aims_mpcc prepare_path "$HOME/aimsracer-data/lap.csv" \
  "$HOME/aimsracer-data/reference-new" \
  --vehicle-config src/controller/config/vehicle.yaml --left-width 0.5 --right-width 0.5
```

示例 0.5 m 左右宽度须按实际可用空间检查；`record_path` 输出 `odom/base_link`，绑定该次定位原点。持久地图参考须先得到真正的 map-frame CSV，并用 `--map-file` 绑定 PCD；此参数不转换 CSV 坐标。生成和目标平台构建使用 `src/aims_mpcc_rt/scripts/export_bundle.py`、`build_bundle.py`，详细命令见[使用指南](docs/usage.md#离线生成与目标平台构建)。

## 构建与来源

从 workspace 根目录构建整个主流程：

```bash
export MAKEFLAGS="-j2 -l2"
colcon build --packages-up-to aims_racer_system aims_mpcc_rt --executor sequential \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DROS_EDITION=ROS2 -DDISTRO_ROS=humble
```

软件验证位于 `verification/`，不属于车辆启动流程。第三方来源与许可保存在 [NOTICE.md](NOTICE.md)、[LICENSE](LICENSE) 和 [vendor](aims_mpcc/vendor/)；参考与模型继续保留相应归属。
