# x86 / NUC 当前源码构建

x86-64 / NUC 使用与 Orin 相同的 main 源码、Ubuntu 22.04 和 ROS 2 Humble。按[当前部署](README.md)准备系统依赖，并只构建同一 selected closure。现有实车闭环证据来自 Orin NX；x86 构建或软件重放不等于 NUC 实车验收。

```bash
git clone --branch main https://github.com/EleSheep-moving/AIMSRacer.git
cd AIMSRacer
git submodule update --init src/FASTLIO2_ROS2
python3 tools/setup_dependencies.py --workspace .
bash tools/setup_acados.sh --workspace . --jobs 2
source /opt/ros/humble/setup.bash
colcon build --packages-up-to aims_racer_system aims_mpcc_rt \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DROS_EDITION=ROS2 -DDISTRO_ROS=humble
source install/setup.bash
export LD_LIBRARY_PATH="$PWD/dependencies/work/acados/install/lib:${LD_LIBRARY_PATH:-}"
```

固定 acados 在 x86 使用 GENERIC BLASFEO/HPIPM；ARM 使用清单中的架构设置。generated C 可以复制到另一 CPU，但 native 库与 manifest 必须在目标架构重建。[MPCC 使用](../../src/controller/docs/usage.md#离线生成与目标平台构建)给出离线流程。

开发测试和定位 replay 见 [verification](../../verification/README.md)，使用独立 ROS domain 和新日志目录。实际连接车辆时，设备名和地址必须与当前 [vehicle 入口](../../src/aims_racer_system/launch/vehicle.launch.py)一致，使用与车辆图相同的 ROS domain；完整启动步骤见[车辆启动](../operations/bringup.md)。

[规则目录](../../rules/README.md)按连接硬件选择。NUC 的 CP2102 规则仅适用于实际接入的设备，当前主流程的 IMU 来自 MID360；规则存在不代表额外 IMU 驱动已经安装。不要凭电脑名称选择不匹配的串口规则。
