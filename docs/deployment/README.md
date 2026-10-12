<a id="当前-main-部署"></a>
# Current main deployment

**English** · [简体中文](README.zh-CN.md)

The current vehicle workflow uses native Ubuntu 22.04, ROS 2 Humble, and one workspace: FAST-LIO2, rear axle EKF, saved-map NDT, and native acados/C++ MPCC. Actual closed-loop driving evidence comes from Orin NX; see the [field review](../reports/2026-10-10-mpcc-field-review.md). [Orin](orin.md) describes the field hardware; [x86 / NUC](nuc.md) describes building the same source for the target platform.

<a id="系统依赖"></a>
## System dependencies

First configure the ROS 2 Humble apt repository and complete the basic installation, then install the build dependencies used by the current package closure:

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

These dependencies target the current selected package closure. Source package dependency declarations and CMake remain the basis for individual builds; fixed sources and patches are in the [dependency manifest](../../dependencies/manifest.json).

<a id="源码准备"></a>
## Source preparation

```bash
git clone --branch main https://github.com/EleSheep-moving/AIMSRacer.git
cd AIMSRacer
git submodule update --init src/FASTLIO2_ROS2 src/vesc
python3 tools/setup_dependencies.py --workspace .
bash tools/setup_acados.sh --workspace . --jobs 2
```

Initialize the FAST-LIO and VESC submodules required by the main workflow and retain the fixed pointer in the current commit. The preparation script places fixed versions of NDT, ndt_omp, Livox ROS 2, and serial under the same `src/`, verifying NDT patches and source identity. When Livox SDK2, Sophus, and CppLinuxSerial are already installed, it reuses and records the existing libraries/configurations; otherwise, it prepares them according to the manifest. Fixed acados and BLASFEO/HPIPM sources build into `dependencies/work/acados/install`. The tools explicitly report differing existing versions/local changes without overwriting historical directories.

The script CLI and source pins are defined by [setup_dependencies.py](../../tools/setup_dependencies.py), [setup_acados.sh](../../tools/setup_acados.sh), and the manifest. Use `--jobs` to limit build concurrency; the default installation prefix for SDKs and system libraries is `/usr/local`. Run the preparation tools as a normal user so source/build/receipts remain owned by that user; only when the system installation directory is not writable do the tools use `sudo cmake --install` to install already-built libraries. An NX with complete system libraries only records the existing installation and does not call sudo. Do not run the entire preparation tool with sudo, as that may leave dependency sources and acados directories owned by root.

<a id="标准-install-构建"></a>
## Standard install build

```bash
source /opt/ros/humble/setup.bash
export MAKEFLAGS="-j2 -l2"
colcon build --packages-up-to aims_racer_system aims_mpcc_rt --executor sequential \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DROS_EDITION=ROS2 -DDISTRO_ROS=humble
source install/setup.bash
export LD_LIBRARY_PATH="$PWD/dependencies/work/acados/install/lib:${LD_LIBRARY_PATH:-}"
```

`MAKEFLAGS="-j2 -l2"` limits Make compilation concurrency within each package; `--executor sequential` limits concurrency between packages. On NX, setting only `CMAKE_BUILD_PARALLEL_LEVEL=2` may still be overridden by colcon's automatically appended `-j8`, exhausting memory during PCL/NDT compilation; do not change runtime threads or vehicle parameters in response.

Livox's ROS 2 edition/distribution are explicit build arguments. This command selects the current complete vehicle dependency closure; other experimental packages need not be built with the vehicle. Use a standard install so installed content corresponds to a build of the same source; rebuild after changes rather than using symbolic links that allow undeployed source to change field behavior.

<a id="数据与-bundle"></a>
## Data and bundle

Keep saved maps and bundles outside the source directory. Current NX deployment targets are `~/maps/20260928_010503/map.pcd` and `~/aimsracer-data/bundles/field-v35-20261010`; a new machine requires an actual copy of these complete data, which are not included in a source clone.

The generator additionally uses acados_template from the fixed source. Set `ACADOS_SOURCE_DIR=.../acados/source` and `ACADOS_INSTALL_DIR=.../acados/install` in the offline generation environment and add `source/interfaces/acados_template` to PYTHONPATH; the dependency preparation tool does not automatically install offline Python packages. Full export/build commands are in [MPCC usage](../../src/controller/docs/usage.md#离线生成与目标平台构建). After transferring generated C between architectures, build the complete native bundle on the target CPU.

<a id="ros-domain-与设备"></a>
## ROS domain and devices

In every vehicle terminal, source `/opt/ros/humble/setup.bash` and this workspace's `install/setup.bash`, using the same `ROS_DOMAIN_ID` and DDS settings. The default is domain 0 when unset. When using a CycloneDDS loopback configuration, confirm that its XML and participant capacity cover the graph; setting `ROS_LOCALHOST_ONLY` alone does not replace an actual DDS interface configuration. Check Livox Ethernet UDP networking separately.

Choose [device rules](../../rules/README.md) according to the actual wiring; rules only provide stable device names and do not start drivers. Confirm that the device and host addresses in [MID360_config.json](../../src/aims_racer_system/params/MID360_config.json) match the field setup.

Launch commands, map initialization, and automatic recording are described in [vehicle bringup](../operations/bringup.md), [known maps](../operations/known-map-mpcc.md), and [recording](../operations/recording.md). Regression checks and replay support development; see [verification](../../verification/README.md).
