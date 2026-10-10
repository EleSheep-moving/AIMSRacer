# Orin NX 当前部署

Orin NX 是现有实跑车辆电脑，直接连接 MID360、ELRS receiver 和 VESC。使用车辆现有 Ubuntu 22.04 / JetPack 原生环境与 ROS 2 Humble，按[当前 main 部署](README.md)准备系统依赖、固定源码、acados 和标准 install。

## 设备与网络

安装实际 Orin 设备的规则：

```bash
sudo install -m 0644 rules/rulesForOrin/*.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules
sudo udevadm trigger
sudo usermod -aG dialout "$USER"
```

新组权限在重新登录后生效。检查 `/dev/ttyELRS`、`/dev/ttyVESC`，核对 [MID360 配置](../../src/aims_racer_system/params/MID360_config.json)与 Ethernet 地址。端点被占用时先查所有者，不直接停止未知进程：

```bash
sudo fuser -v /dev/ttyELRS /dev/ttyVESC
```

## 构建与运行

在 workspace 根目录：

```bash
git submodule update --init src/FASTLIO2_ROS2
python3 tools/setup_dependencies.py --workspace .
bash tools/setup_acados.sh --workspace . --jobs 2
source /opt/ros/humble/setup.bash
colcon build --packages-up-to aims_racer_system aims_mpcc_rt \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DROS_EDITION=ROS2 -DDISTRO_ROS=humble
source install/setup.bash
export LD_LIBRARY_PATH="$PWD/dependencies/work/acados/install/lib:${LD_LIBRARY_PATH:-}"
ros2 launch aims_racer_system race.launch.py \
  map_file:="$HOME/maps/20260928_010503/map.pcd" \
  artifact_directory:="$HOME/aimsracer-data/bundles/field-v35-20261010" \
  record:=true
```

地图和目标平台 bundle 须实际部署。当前 launcher 从 bundle 读取参考/config，初始化与权限检查见[已知地图](../operations/known-map-mpcc.md)。默认手动 enable、单圈停车；`auto_start=true` 只执行一次就绪启动，显式 stop 取消等待。`record=true` 自动保存完整现场会话。

默认 v35 参数来自 2026-10-10 实跑；巡航标签 3.5 m/s，规划峰值约 2.99、实测约 2.89 m/s。N15/dt=.1/RTI2 和现场 source 修复保留在[复盘](../reports/2026-10-10-mpcc-field-review.md)。联合 CPU 负载、worker 时间与行驶表现分别记录，不由桌面构建结果推断。
