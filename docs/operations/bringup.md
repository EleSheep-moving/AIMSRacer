# 车辆启动

当前流程使用一个 workspace。先按[部署指南](../deployment/README.md)准备 ROS 和硬件，再在根目录准备固定源码与构建：

```bash
git submodule update --init src/FASTLIO2_ROS2
python3 tools/setup_dependencies.py --workspace .
bash tools/setup_acados.sh --workspace . --jobs 2
source /opt/ros/humble/setup.bash
export MAKEFLAGS="-j2 -l2"
colcon build --packages-up-to aims_racer_system aims_mpcc_rt --executor sequential \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DROS_EDITION=ROS2 -DDISTRO_ROS=humble
source install/setup.bash
export LD_LIBRARY_PATH="$PWD/dependencies/work/acados/install/lib:${LD_LIBRARY_PATH:-}"
```

以后每个操作终端只需 source `/opt/ros/humble/setup.bash` 和当前根目录的 `install/setup.bash`，设置当前 acados `install/lib` 路径。zsh 使用对应 `setup.zsh`。同一车辆图的所有终端使用相同 `ROS_DOMAIN_ID` 与 DDS 配置。Livox Ethernet UDP 与 ROS DDS 是两条独立链路；网络和串口规则见[部署](../deployment/README.md)及 [rules](../../rules/README.md)。

## 选择一个入口

| 入口 | 内容 | 用途 |
| --- | --- | --- |
| `vehicle.launch.py` | MID360、FAST-LIO2、后轴适配、EKF、RC、VESC | 手动驾驶和传感器检查 |
| `mapping.launch.py` | 车辆传感器、PGO、建图 TF、可选录包 | 手动驾驶建图 |
| `race.launch.py` | vehicle、NDT、定位 monitor、native MPCC、可选录包 | 已知地图闭环单圈或连续圈 |

```bash
ros2 launch aims_racer_system vehicle.launch.py
```

建图使用以下独立入口；PGO 与已知地图 NDT 的 `map → odom` 归属不能同时启用：

```bash
ros2 launch aims_racer_system mapping.launch.py record:=true
```

建图完成并确认 PGO 已接收数据后，用保留的[地图保存工具](../../tools/save_map.sh)写新目录：

```bash
bash tools/save_map.sh "$HOME/maps/my-new-map"
```

该脚本检查服务成功及 `map.pcd`、`poses.txt` 文件；参考恢复见 [tools/reference](../../tools/reference/README.md)。

已知地图运行：

```bash
ros2 launch aims_racer_system race.launch.py \
  map_file:="$HOME/maps/20260928_010503/map.pcd" \
  artifact_directory:="$HOME/aimsracer-data/bundles/field-v35-20261010" \
  record:=true
```

`race` 已包含车辆和定位，无需额外启动。它要求现有地图和完整 native bundle，默认 `auto_start=false`、`repeat_laps=false`、`record=false`。地图初始化、手动启用与一次自动启动见[已知地图指南](known-map-mpcc.md)，参数详见 [MPCC 使用](../../src/controller/docs/usage.md#在线启动参数)。

## 运行与停止

启动时保持 RC 锁定，确认定位地图与实际摆放一致、车辆静止、转向方向正确。检查项目见[车辆检查](vehicle-checklist.md)。选定自主 speed 模式后，手动启用：

```bash
ros2 service call /mpcc/enable std_srvs/srv/SetBool '{data: true}'
```

停止及取消尚未发生的自动启动：

```bash
ros2 service call /mpcc/enable std_srvs/srv/SetBool '{data: false}'
```

现场紧急停止依靠 RC/物理急停；停稳并撤销权限后再 Ctrl-C。保存录包、runtime 和地图/bundle 身份，详见[录制](recording.md)。同一次运行只允许一个 `/drive` 发布者。
