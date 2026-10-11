<a id="x86--nuc-当前源码构建"></a>
# Building current source for x86 / NUC

**English** · [简体中文](nuc.zh-CN.md)

x86-64 / NUC uses the same main source, Ubuntu 22.04, and ROS 2 Humble as Orin. Prepare system dependencies according to [current deployment](README.md) and build only the same selected closure. Existing closed-loop vehicle evidence comes from Orin NX; an x86 build or software replay does not constitute NUC vehicle acceptance.

```bash
git clone --branch main https://github.com/EleSheep-moving/AIMSRacer.git
cd AIMSRacer
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

The fixed acados uses GENERIC BLASFEO/HPIPM on x86; ARM uses the architecture settings in the manifest. Generated C can be copied to another CPU, but native libraries and manifests must be rebuilt on the target architecture. [MPCC usage](../../src/controller/docs/usage.md#离线生成与目标平台构建) provides the offline workflow.

For development tests and localization replay, see [verification](../../verification/README.md), using a separate ROS domain and a new log directory. When connecting to a vehicle, device names and addresses must match the current [vehicle entry point](../../src/aims_racer_system/launch/vehicle.launch.py), using the same ROS domain as the vehicle graph; see [vehicle bringup](../operations/bringup.md) for the full procedure.

Select from the [rules directory](../../rules/README.md) according to the connected hardware. The NUC CP2102 rule applies only to a device actually connected; the current main workflow obtains its IMU from MID360. A rule's existence does not mean an additional IMU driver is installed. Do not choose incompatible serial rules based on the computer's name.
