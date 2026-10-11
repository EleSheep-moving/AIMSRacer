<a id="车辆启动"></a>
# Vehicle bringup

**English** · [简体中文](bringup.zh-CN.md)

The current workflow uses one workspace. First prepare ROS and hardware according to the [deployment guide](../deployment/README.md), then prepare fixed sources and build from the root:

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

For later operation terminals, source `/opt/ros/humble/setup.bash` and `install/setup.bash` from the current root, and set the current acados `install/lib` path. Use the corresponding `setup.zsh` for zsh. All terminals in the same vehicle graph use the same `ROS_DOMAIN_ID` and DDS configuration. Livox Ethernet UDP and ROS DDS are separate links; see [deployment](../deployment/README.md) and [rules](../../rules/README.md) for networking and serial rules.

<a id="选择一个入口"></a>
## Choose one entry point

| Entry point | Contents | Purpose |
| --- | --- | --- |
| `vehicle.launch.py` | MID360, FAST-LIO2, rear axle adaptation, EKF, RC, VESC | Manual driving and sensor checks |
| `mapping.launch.py` | Vehicle sensors, PGO, mapping TF, optional recording | Mapping while driving manually |
| `race.launch.py` | vehicle, NDT, localization monitor, native MPCC, optional recording | Known-map closed-loop single or repeated laps |

```bash
ros2 launch aims_racer_system vehicle.launch.py
```

Use the following separate entry point for mapping; PGO and known-map NDT ownership of `map → odom` cannot be enabled simultaneously:

```bash
ros2 launch aims_racer_system mapping.launch.py record:=true
```

After completing mapping and confirming that PGO has received data, use the retained [map saving tool](../../tools/save_map.sh) to write a new directory:

```bash
bash tools/save_map.sh "$HOME/maps/my-new-map"
```

The script checks service success and the `map.pcd` and `poses.txt` files; see [tools/reference](../../tools/reference/README.md) for reference recovery.

Known-map operation:

```bash
ros2 launch aims_racer_system race.launch.py \
  map_file:="$HOME/maps/20260928_010503/map.pcd" \
  artifact_directory:="$HOME/aimsracer-data/bundles/field-v35-20261010" \
  record:=true
```

`race` already includes the vehicle and localization, so no additional launch is needed. It requires an existing map and a complete native bundle, with defaults `auto_start=false`, `repeat_laps=false`, and `record=false`. See the [known-map guide](known-map-mpcc.md) for map initialization, manual enable, and one-time auto start, and [MPCC usage](../../src/controller/docs/usage.md#在线启动参数) for parameters.

<a id="运行与停止"></a>
## Run and stop

Keep RC locked during startup; confirm that the localization map agrees with the vehicle's actual placement, the vehicle is stationary, and the steering direction is correct. See [vehicle checks](vehicle-checklist.md). After selecting autonomous speed mode, enable manually:

```bash
ros2 service call /mpcc/enable std_srvs/srv/SetBool '{data: true}'
```

Stop and cancel an auto start that has not yet occurred:

```bash
ros2 service call /mpcc/enable std_srvs/srv/SetBool '{data: false}'
```

Field emergency stopping relies on RC/the physical emergency stop; use Ctrl-C after the vehicle is stationary and permission has been revoked. Preserve the bag, runtime, and map/bundle identity; see [recording](recording.md). Only one `/drive` publisher is permitted in a run.
