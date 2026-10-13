<a id="orin-nx-当前部署"></a>
# Current Orin NX deployment

**English** · [简体中文](orin.zh-CN.md)

Orin NX is the computer in the existing field-run vehicle, directly connected to MID360, the ELRS receiver, and VESC. Use the vehicle's existing native Ubuntu 22.04 / JetPack environment and ROS 2 Humble; follow [current main deployment](README.md) to prepare system dependencies, fixed sources, acados, and a standard install.

<a id="设备与网络"></a>
## Devices and network

Install the rules for the actual Orin devices:

```bash
sudo install -m 0644 rules/rulesForOrin/*.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules
sudo udevadm trigger
sudo usermod -aG dialout "$USER"
```

New group permissions take effect after logging in again. Check `/dev/ttyELRS` and `/dev/ttyVESC`, and verify the [MID360 configuration](../../src/aims_racer_system/params/MID360_config.json) and Ethernet addresses. If an endpoint is occupied, identify its owner before acting; do not directly stop an unknown process:

```bash
sudo fuser -v /dev/ttyELRS /dev/ttyVESC
```

<a id="构建与运行"></a>
## Build and run

From the workspace root:

```bash
git submodule update --init src/FASTLIO2_ROS2 src/vesc
python3 tools/setup_dependencies.py --workspace .
bash tools/setup_acados.sh --workspace . --jobs 2
source /opt/ros/humble/setup.bash
export MAKEFLAGS="-j2 -l2"
colcon build --packages-up-to aims_racer_system aims_mpcc_rt --executor sequential \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DROS_EDITION=ROS2 -DDISTRO_ROS=humble
source install/setup.bash
export LD_LIBRARY_PATH="$PWD/dependencies/work/acados/install/lib:${LD_LIBRARY_PATH:-}"
ros2 launch aims_racer_system race.launch.py \
  map_file:="$HOME/maps/20260928_010503/map.pcd" \
  artifact_directory:="$HOME/aimsracer-data/bundles/field-v35-20261010" \
  record:=true
```

The map and target-platform bundle must actually be deployed. The current launcher reads the reference/config from the bundle; see [known maps](../operations/known-map-mpcc.md) for initialization and permission checks. Defaults are manual enable and stopping after one lap; `auto_start=true` performs a single start when ready, and explicit stop cancels waiting. `record=true` automatically saves the complete field session.

Default v35 parameters come from the 2026-10-10 field run; the cruise label is 3.5 m/s, the planned peak approximately 2.99, and the measured peak approximately 2.89 m/s. N15/dt=.1/RTI2 and the field source fixes are retained in the [review](../reports/2026-10-10-mpcc-field-review.md). Combined CPU load, worker timing, and driving performance are recorded separately rather than inferred from desktop build results.
