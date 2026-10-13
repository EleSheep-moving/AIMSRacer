# 旧地图 MPCC → 扩展地图 PP：场地命令链

[English](field-mpcc-then-pp.md)

本次顺序已确认：**旧场地的旧地图＋旧轨迹先调 MPCC，随后在同一场地建立更大新地图、绘制新轨迹，再测试 PP。两类驾驶都记录 ACC。** 小电流标定单独安排，不包含在这条链中。

| 流程 | 地图 | 参考 | 控制器产物 |
| --- | --- | --- | --- |
| MPCC 基线/调参 | 旧 `20260928_010503/map.pcd` | 旧 bundle 的 `input_reference` | 原场地 bundle，或用旧参考生成的新调参 bundle |
| 建图 | 新目录 | 尚无 | PGO 保存 `map.pcd`、`poses.txt` |
| PP | 新的更大地图 | GUI 导出的新参考 | `reference.json`，不需要 MPCC bundle |

## 0. 每个 NX 终端的公共环境

以下使用 **NX 的 bash**。在每个操作终端先运行同一段；把 `AIMS_FIELD_TAG` 改为本次唯一标识，所有终端保持一致。地图、参考和导出 bundle 使用新目录，不覆盖旧数据。当前源码为功能分支 `feat/pp-calibration-reference-editor`，main 保留原场地版本。

```bash
cd /home/aims/aimsracer-data/code/pp-calibration-reference-editor
source /opt/ros/humble/setup.bash
source /home/aims/AIMSRacer/install/setup.bash
source "$PWD/install/setup.bash"
export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=0
export AIMS_WS="$PWD"
export AIMS_FIELD_TAG=20261013-field
export AIMS_FIELD_ROOT="$HOME/aimsracer-data/field-tests/$AIMS_FIELD_TAG"
export AIMS_OLD_MAP="$HOME/maps/20260928_010503/map.pcd"
export AIMS_OLD_BUNDLE="$HOME/aimsracer-data/bundles/field-v35-20261010"
export AIMS_NEW_MAP_DIR="$HOME/maps/$AIMS_FIELD_TAG-expanded"
export AIMS_EDITOR_OUTPUT="$AIMS_FIELD_ROOT/new-references"
export AIMS_NEW_PP_REF="$AIMS_EDITOR_OUTPUT/pp-route"
export ACADOS_SOURCE_DIR=/home/aims/AIMSRacer/dependencies/work/acados/source
export ACADOS_INSTALL_DIR=/home/aims/AIMSRacer/dependencies/work/acados/install
export LD_LIBRARY_PATH="$ACADOS_INSTALL_DIR/lib:${LD_LIBRARY_PATH:-}"
mkdir -p "$AIMS_FIELD_ROOT/configs"
```

公共环境中的旧地图与 bundle 已在 NX 核对存在。`ROS_DOMAIN_ID` 在整车相关终端保持一致；这些是实机命令，不使用录包回放的 `use_sim_time` 或虚拟重新启用选项。

## 1. 先跑旧地图 MPCC 基线，录 ACC

**终端 A：** 遥控锁车，启动完整车辆/定位/MPCC/录包链：

```bash
export AIMS_SESSION="$HOME/aimsracer-data/sessions/$(date +%F)/$(date +%H%M%S)-mpcc-baseline"
ros2 launch aims_racer_system race.launch.py \
  controller:=mpcc \
  map_file:="$AIMS_OLD_MAP" \
  artifact_directory:="$AIMS_OLD_BUNDLE" \
  auto_start:=false repeat_laps:=false \
  longitudinal_acceleration:=true record:=true \
  session_directory:="$AIMS_SESSION"
```

**终端 B：** 公共环境后，输入实际后轴 base_link 在旧地图中的初值。六个数分别是 x/y/z/roll/pitch/yaw；角度为 rad。不要把示例原点或 Livox 位置当成实际初值。

```bash
read -r -p 'Old-map base_link: x y z roll pitch yaw (rad): ' \
  AIMS_X AIMS_Y AIMS_Z AIMS_ROLL AIMS_PITCH AIMS_YAW
ros2 run aims_racer_system relocalize_known_map.py "$AIMS_OLD_MAP" \
  --pose-frame base_link --x "$AIMS_X" --y "$AIMS_Y" --z "$AIMS_Z" \
  --roll "$AIMS_ROLL" --pitch "$AIMS_PITCH" --yaw "$AIMS_YAW"
ros2 topic echo /localization/status --once
ros2 topic echo /mpcc/status --once
ros2 topic echo /state_estimation/longitudinal_acceleration/status --once
```

确认定位 `ready=true`、MPCC 求解器/输入可用、ACC `initialized=true`；车辆静止且方向与参考匹配。遥控切换到解锁、导航、速度模式，再执行：

```bash
ros2 service call /mpcc/enable std_srvs/srv/SetBool '{data: true}'
```

默认一圈完成停车。每套配置先跑 3–5 次，保留起步前与停车后的静止段；下一圈须满足静止、定位和方向条件，再明确启用。需要不同 bag 的圈次，停止并重启 launch 创建新 session。人工结束用：

```bash
ros2 service call /mpcc/enable std_srvs/srv/SetBool '{data: false}'
```

实际停稳并撤销权限后，在终端 A Ctrl-C；不要直接关闭运动中的 launch。终端 A 的 `AIMS_SESSION` 仍指向该次录包，按第 6 节核验。

## 2. 调 MPCC：旧地图和旧轨迹固定，生成新 bundle

先停止整车 launch，在 NX 公共环境下复制旧 bundle 的配置，一次只修改一个参数。比如 `contour_weight`、`heading_weight`、速度规划参数；不要直接改旧 bundle。

```bash
cp "$AIMS_OLD_BUNDLE/input_config.yaml" "$AIMS_FIELD_ROOT/configs/mpcc-trial-01.yaml"
nano "$AIMS_FIELD_ROOT/configs/mpcc-trial-01.yaml"
export PYTHONPATH="$AIMS_WS/src/controller:$ACADOS_SOURCE_DIR/interfaces/acados_template:${PYTHONPATH:-}"
export AIMS_TRIAL_BUNDLE="$HOME/aimsracer-data/bundles/$AIMS_FIELD_TAG-mpcc-trial-01"
python3 "$AIMS_WS/src/aims_mpcc_rt/scripts/export_bundle.py" \
  --config "$AIMS_FIELD_ROOT/configs/mpcc-trial-01.yaml" \
  --reference "$AIMS_OLD_BUNDLE/input_reference" \
  --output "$AIMS_TRIAL_BUNDLE" --horizon 15 --dt 0.1 --source-only
python3 "$AIMS_WS/src/aims_mpcc_rt/scripts/build_bundle.py" \
  --bundle "$AIMS_TRIAL_BUNDLE" --acados-install "$ACADOS_INSTALL_DIR"
```

**当前 MPCC 参数冻结在生成 C 与 bundle 中。改 YAML 后要重新 export/build，重启节点才生效。** 输出目录必须不存在；下一组使用 `trial-02` 等新名字。仅做这项离线导出与 NX 目标构建，不必重编整个 ROS 工作区。

终端 A 用新 bundle 重启，仍配旧地图：

```bash
export AIMS_SESSION="$HOME/aimsracer-data/sessions/$(date +%F)/$(date +%H%M%S)-mpcc-trial-01"
ros2 launch aims_racer_system race.launch.py \
  controller:=mpcc map_file:="$AIMS_OLD_MAP" \
  artifact_directory:="$AIMS_TRIAL_BUNDLE" \
  auto_start:=false repeat_laps:=false \
  longitudinal_acceleration:=true record:=true \
  session_directory:="$AIMS_SESSION"
```

终端 B 重复第 1 节的旧地图初始化、状态检查和启用流程。重新 source 公共环境不会设置 `AIMS_TRIAL_BUNDLE`，因此调参构建与调参启动在同一终端执行，或在启动终端明确设置它为生成目录。配置名、会话名、实际速度与干预一并保留。完成后停止整车链，再进入建图。

## 3. 在旧场地建立更大的新地图

**终端 A：** 使用建图入口，遥控前进覆盖扩展区域，加入回环并等待地图稳定：

```bash
export AIMS_SESSION="$HOME/aimsracer-data/sessions/$(date +%F)/$(date +%H%M%S)-mapping-expanded"
ros2 launch aims_racer_system mapping.launch.py \
  record:=true session_directory:="$AIMS_SESSION"
```

**终端 B：** 公共环境后，在建图 launch 仍运行时保存：

```bash
bash "$AIMS_WS/tools/save_map.sh" "$AIMS_NEW_MAP_DIR"
sha256sum "$AIMS_OLD_MAP" "$AIMS_NEW_MAP_DIR/map.pcd"
```

保存工具必须报告成功，且目录中有非空 `map.pcd`、`poses.txt`。旧图保持原样；本次只是重建更大图，不执行旧图原点保持或地图拼接。保存完成、车辆停稳后终止建图 launch，再核验建图 bag。建图 PGO 与 race 的 NDT 不同时运行。

## 4. 新地图上绘制新 PP 轨迹

**NX 终端 A：** 公共环境后启动离线编辑器：

```bash
export PYTHONPATH="$AIMS_WS/src/controller:$AIMS_WS:${PYTHONPATH:-}"
/home/aims/aimsracer-data/dependencies/reference-editor-venv/bin/python \
  -m tools.reference.editor.server \
  --map "$AIMS_NEW_MAP_DIR/map.pcd" \
  --config "$AIMS_WS/src/controller/config/vehicle.yaml" \
  --output "$AIMS_EDITOR_OUTPUT"
```

**主机终端：** 建立转发，浏览器打开 `http://127.0.0.1:8765`：

```bash
ssh -N -L 8765:127.0.0.1:8765 aims@192.168.123.107
```

在新地图上绘制后轴 XY 闭合轨迹，确认方向、起点、曲率与可通行余量。需要后续 PP 标定时再标直线区间；普通 PP 跟踪不要求直线区间。预览通过，**Export 名称输入 `pp-route`**，输出即 `$AIMS_NEW_PP_REF`，包含 `reference.json`、`path.csv`、`metadata.json`、工程和区间文件。新地图的坐标原点可能与旧地图不同，旧轨迹/旧初始化坐标不能直接复用。

保存工程与导出目录，编辑器和 SSH 隧道可 Ctrl-C 关闭，不影响已写出的文件。

## 5. 后测 PP：新地图＋新轨迹，录 ACC

**NX 终端 A：** 公共环境后，以 1 m/s 上限启动：

```bash
export AIMS_SESSION="$HOME/aimsracer-data/sessions/$(date +%F)/$(date +%H%M%S)-pp-new-map"
ros2 launch aims_racer_system race.launch.py \
  controller:=pp map_file:="$AIMS_NEW_MAP_DIR/map.pcd" \
  reference_directory:="$AIMS_NEW_PP_REF" \
  vehicle_config:="$AIMS_WS/src/controller/config/vehicle.yaml" \
  speed_limit:=1.0 auto_start:=false repeat_laps:=false \
  longitudinal_acceleration:=true record:=true \
  session_directory:="$AIMS_SESSION"
```

**终端 B：** 公共环境后，按新地图输入新的 base_link 初值：

```bash
read -r -p 'New-map base_link: x y z roll pitch yaw (rad): ' \
  AIMS_X AIMS_Y AIMS_Z AIMS_ROLL AIMS_PITCH AIMS_YAW
ros2 run aims_racer_system relocalize_known_map.py "$AIMS_NEW_MAP_DIR/map.pcd" \
  --pose-frame base_link --x "$AIMS_X" --y "$AIMS_Y" --z "$AIMS_Z" \
  --roll "$AIMS_ROLL" --pitch "$AIMS_PITCH" --yaw "$AIMS_YAW"
ros2 topic echo /localization/status --once
ros2 topic echo /pp/status --once
ros2 topic echo /state_estimation/longitudinal_acceleration/status --once
```

定位稳定、状态可用、车辆静止、转向方向正确；遥控解锁、导航、速度模式后：

```bash
ros2 service call /pp/enable std_srvs/srv/SetBool '{data: true}'
```

默认跑一圈。检查跟踪、停止和遥控接管后再调整 PP。人工结束用：

```bash
ros2 service call /pp/enable std_srvs/srv/SetBool '{data: false}'
```

停稳后关闭终端 A launch，核验 bag。调 PP 时可把参数复制到外部目录：

```bash
cp "$AIMS_WS/src/aims_pp/config/pp.yaml" "$AIMS_FIELD_ROOT/configs/pp-trial-01.yaml"
nano "$AIMS_FIELD_ROOT/configs/pp-trial-01.yaml"
```

下一次启动在第 5 节 race 命令中追加 `pp_config:="$AIMS_FIELD_ROOT/configs/pp-trial-01.yaml"`，更换 session 后重启。PP 参数改动不需 acados export/build，也不需重编整个 ROS 工作区；当前节点不支持热更新，必须重启。轨迹本身改变则 GUI 重新导出到新目录，修改 `reference_directory`，保留原版本。

## 6. 每组结束检查，回来分析

在该组 **launch 所在终端** 正常 Ctrl-C 之后：

```bash
ros2 bag info "$AIMS_SESSION/bag"
ls "$AIMS_SESSION"
```

核对 bag 含 `/rear_axle/wheel_odom`、`/rear_axle/imu`、`/state_estimation/longitudinal_acceleration`、其 `/status`、`/vesc/command`、`/sensors/core`、定位和对应控制器状态。保留实际地图/参考/bundle/config 与录包目录。起止静止、加速、稳速、弯道、减速、停稳都留在记录中；电机电流—轮速—ACC 的专门标定另行安排。

回到室内分开分析：MPCC 权重/速度配置、圈时与误差；PP 跟踪表现；ACC 静止噪声、加减速滞后、停车回零、转弯影响、源年龄和有效性。ACC 目前仅观测，不接入导航 EKF 或加速度驱动闭环。

已完成的软件/NX时序检查及其限制见[实现报告](../reports/2026-10-13-pp-calibration-reference-editor.zh-CN.md)。这份命令链未在场地自动执行；实际驾驶由操作员依次启动和授权。
