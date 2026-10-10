# 当前 main 部署

当前车辆主流程为原生 Ubuntu 22.04、ROS 2 Humble、单 workspace：FAST-LIO2、后轴 EKF、保存地图 NDT 与 native acados/C++ MPCC。实际闭环行驶证据来自 Orin NX，见[外场复盘](../reports/2026-10-10-mpcc-field-review.md)。[Orin](orin.md)说明现场设备，[x86 / NUC](nuc.md)说明同源码的目标平台构建。

## 系统依赖

先完成 ROS 2 Humble 的 apt 源与基础安装，再安装当前包闭包使用的构建依赖：

```bash
sudo apt update
sudo apt install \
  build-essential git cmake pkg-config python3-pip python3-colcon-common-extensions \
  python3-numpy python3-scipy python3-yaml python3-pytest \
  libpcl-dev libeigen3-dev libyaml-cpp-dev libssl-dev \
  libgflags-dev libgoogle-glog-dev libfmt-dev libasio-dev \
  ros-humble-ros-base ros-humble-gtsam \
  ros-humble-pcl-conversions ros-humble-pcl-ros \
  ros-humble-robot-localization ros-humble-message-filters ros-humble-angles \
  ros-humble-tf2-ros ros-humble-tf2-eigen ros-humble-ackermann-msgs \
  ros-humble-diagnostic-updater ros-humble-sensor-msgs-py ros-humble-udp-msgs \
  ros-humble-rmw-cyclonedds-cpp ros-humble-rosbag2 \
  ros-humble-ament-cmake-auto ros-humble-ament-cmake-python \
  ros-humble-ament-cmake-gtest ros-humble-ament-cmake-pytest
python3 -m pip install --user 'casadi==3.7.2' 'matplotlib<3.9' deprecated jinja2 future
```

以上面向当前 selected package closure。源码包的依赖声明与 CMake 仍是具体构建依据，固定源和补丁在[依赖清单](../../dependencies/manifest.json)。

## 源码准备

```bash
git clone --branch main https://github.com/EleSheep-moving/AIMSRacer.git
cd AIMSRacer
git submodule update --init src/FASTLIO2_ROS2
python3 tools/setup_dependencies.py --workspace .
bash tools/setup_acados.sh --workspace . --jobs 2
```

只初始化主流程需要的 FAST-LIO 子模块，并保留当前提交的固定指针。准备脚本将固定版本 NDT、ndt_omp、Livox ROS 2、serial 放在同一 `src/` 下，核验 NDT 补丁和源码身份。Livox SDK2、Sophus、CppLinuxSerial 已安装时沿用并记录现有库/配置，缺失时按清单准备。acados 与 BLASFEO/HPIPM 的固定来源构建到 `dependencies/work/acados/install`。现有不同版本/本地修改由工具明确报告，不覆盖历史目录。

脚本 CLI 与各 source pin 以 [setup_dependencies.py](../../tools/setup_dependencies.py)、[setup_acados.sh](../../tools/setup_acados.sh)及 manifest 为准。可通过 `--jobs` 限制构建并发；SDK 和系统库默认安装 prefix 为 `/usr/local`。若缺少这些库，先以普通用户准备 ROS source，再为系统库安装赋予权限，避免把 ROS clone 变成 root 所有：

```bash
python3 tools/setup_dependencies.py --workspace . --skip-sdk \
  --only lidar_localization_ros2 ndt_omp_ros2 livox_ros_driver2 serial
sudo python3 tools/setup_dependencies.py --workspace . --only Sophus CppLinuxSerial sdk
```

系统库已完整安装的 NX 使用前面的普通用户命令记录现有安装即可。

## 标准 install 构建

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-up-to aims_racer_system aims_mpcc_rt \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DROS_EDITION=ROS2 -DDISTRO_ROS=humble
source install/setup.bash
export LD_LIBRARY_PATH="$PWD/dependencies/work/acados/install/lib:${LD_LIBRARY_PATH:-}"
```

Livox 的 ROS 2 edition/distribution 是显式构建参数。该命令选择当前整车依赖闭包；其它实验包不需要随车辆构建。使用标准 install，使安装内容与同一源码构建对应；修改后重新构建，不靠符号链接让未部署源码改变现场行为。

## 数据与 bundle

保存地图和 bundle 放在源码目录外。当前 NX 部署目标为 `~/maps/20260928_010503/map.pcd` 与 `~/aimsracer-data/bundles/field-v35-20261010`；新机器需实际复制这些完整数据，源码 clone 不包含它们。

生成器额外使用固定源码的 acados_template。离线生成环境设置 `ACADOS_SOURCE_DIR=.../acados/source`、`ACADOS_INSTALL_DIR=.../acados/install`，并将 `source/interfaces/acados_template` 加入 PYTHONPATH；依赖准备工具不自动安装离线 Python 包。完整 export/build 命令见 [MPCC 使用](../../src/controller/docs/usage.md#离线生成与目标平台构建)。跨架构传递 generated C 后，在目标 CPU 构建完整 native bundle。

## ROS domain 与设备

每个车辆终端 source `/opt/ros/humble/setup.bash` 和该 workspace 的 `install/setup.bash`，使用相同 `ROS_DOMAIN_ID` 与 DDS 设置。默认未设置时为 domain 0。若使用 CycloneDDS loopback 配置，确认 XML 和 participant capacity 足够覆盖图；单独设置 `ROS_LOCALHOST_ONLY` 不替代实际 DDS interface 配置。Livox Ethernet UDP 网络单独核对。

[设备规则](../../rules/README.md)按实际连线选择；规则只提供稳定设备名，不启动驱动。确认 [MID360_config.json](../../src/aims_racer_system/params/MID360_config.json)中的设备与主机地址适用于现场。

启动命令、地图初始化与自动录制见[车辆启动](../operations/bringup.md)、[已知地图](../operations/known-map-mpcc.md)、[录制](../operations/recording.md)。回归和重放用于开发，见 [verification](../../verification/README.md)。
