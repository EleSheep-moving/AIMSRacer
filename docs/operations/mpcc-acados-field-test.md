# NX acados MPCC 外场测试流程

更新：2026-10-10。分支：`feat/mpcc-acados-runtime`。新控制器已删除 shadow 模式。

当前具备开始人工接管下低速闭环试验的条件。桌面/NX 软件验证不代表实车跟踪验收；本次试验用于确认它。纵向速度响应辨识暂缓，独立仿真超限保留为观察项。

## 1. 本次目标与固定配置

先验证新求解器在实际 FAST-LIO2＋EKF＋NDT 输入下的轨迹跟踪、转向振荡、计划接管与停车表现。按顺序进行：锁车检查 → 0.5 m/s 短段 → 0.5 m/s 完整一圈 → 1.0 m/s。

| 项目 | 本次选择 |
|---|---|
| 地图 | `/home/aims/maps/20260928_010503/map.pcd` |
| 地图 SHA-256 | `1db8c1905dc99ed4c0897838421118b998d15eb7cf57f50cee96a8b0744870dc` |
| 控制器 | `implementation:=acados_cpp`、`rate_bounded_v2` |
| 起步测试 bundle | `qual-v2-field-v05-n10`，巡航目标 0.5 m/s |
| 后续 bundle | `qual-v2-field-v10-n10`，巡航目标 1.0 m/s |
| 模型 / 权重 | 保留当前六状态运动学模型和权重 |
| 预测 / 调度 | N10、dt=0.1 s；请求上限 20 Hz；输出 50 Hz |
| 时限 | 完整交付预算 50 ms；原始状态 TTL 0.8 s |
| 行驶循环 | 默认执行一圈后停车；不是无限重复 |

**必须回到这张地图对应的场地。** 如果换场地，要先准备匹配的新地图、轨迹和 bundle。

0.5/1.0 m/s 是巡航目标；这两套已验证配置的硬速度上限仍为 1.5 m/s。不能把巡航目标当作硬限速。当前 `enforce_corridor=false`，配置里的赛道宽度不提供越界自动停车。第一轮保留配置，出现持续超速或明显偏离时人工停止。

## 2. 所有终端使用同一环境

每个终端连接 NX 后进入 bash，再 source：

```bash
ssh aims@192.168.123.107
bash
source /home/aims/.config/superpowers/worktrees/AIMSRacer/mpcc-acados-runtime/src/aims_mpcc_rt/tools/field_env.bash
```

脚本只设置环境，不启动程序、不 enable、不发布驱动命令。它选择最新 FAST-LIO2、匹配 NDT、更新后的 C++ localization monitor 和删除 shadow 的控制器，统一 ROS domain 为 0、实时时钟。不要叠加录包回放或另一套车辆 bringup。

## 3. 建立本次记录目录（操作终端，首次做一次）

先保持遥控锁车、油门中位、标定模式关闭。

```bash
export MPCC_BUNDLE="$MPCC_BUNDLE05"
export MPCC_RUN="$MPCC_DATA_ROOT/sessions/$(date +%F)/mpcc-acados/$(date +%H%M%S)_v05_short"
mkdir -p "$MPCC_RUN"
printf 'export MPCC_RUN=%q\nexport MPCC_BUNDLE=%q\n' \
  "$MPCC_RUN" "$MPCC_BUNDLE" > "$MPCC_DATA_ROOT/sessions/mpcc-field-current.env"
cp "$MPCC_DATA_ROOT/sessions/mpcc-field-current.env" "$MPCC_RUN/run.env"
git -C "$MPCC_REPO" rev-parse HEAD > "$MPCC_RUN/source-head.txt"
git -C /home/aims/AIMSRacer status --short > "$MPCC_RUN/vehicle-source-status.txt"
sha256sum "$MPCC_MAP" "$MPCC_BUNDLE/input_config.yaml" \
  "$MPCC_INTERFACE_ROOT/ros-install/aims_mpcc_rt/lib/aims_mpcc_rt/mpcc_rt_node" \
  > "$MPCC_RUN/hashes.txt"
cp "$MPCC_BUNDLE/input_config.yaml" "$MPCC_RUN/vehicle.yaml"
cp "$MPCC_BUNDLE/input_reference/metadata.json" "$MPCC_RUN/reference-metadata.json"
cp "$MPCC_BUNDLE/native_manifest.json" "$MPCC_RUN/native-manifest.json"
printf 'v05 short; operator stop planned; speed response identification deferred\n' > "$MPCC_RUN/notes.txt"
```

随后每个其他终端，在完成第 2 步后加载同一次实验：

```bash
source /home/aims/aimsracer-data/sessions/mpcc-field-current.env
```

本次所有日志会落到同一个 `$MPCC_RUN`。下一轮必须新建目录，避免覆盖控制器的 `runtime.csv`。

## 4. 终端 A：启动车辆基础链路

```bash
ros2 launch aims_racer_system base_orin_livox_bringup_v2.launch.py \
  2>&1 | tee "$MPCC_RUN/bringup.log"
```

它启动 Livox、FAST-LIO2、后轴转换、EKF、VESC、RC 选择器。已知地图定位由下一个终端启动；该基础 launch 没有把旧 localizer 加入运行图。

确认雷达和 VESC 已正常连接，没有反复退出或 IMU 连续性错误。启动后至少保持静止 10 秒，等待陀螺零偏校准完成；期间不要手推车辆或转动车轮。操作终端确认：

```bash
ros2 topic echo /imu/gyro_bias/status --once
```

其中 `livox_gyro_bias` 的 `ready` 必须为 true。校准尚未完成时后轴 IMU/里程计链路可能尚未就绪，不能据此认定 EKF 故障。此时保持锁车。

## 5. 终端 B：启动 NDT

```bash
ros2 launch aims_racer_system known_map_localization.launch.py \
  map_file:="$MPCC_MAP" use_sim_time:=false \
  2>&1 | tee "$MPCC_RUN/localization.log"
```

等待 `Registration target warm-up` 和 `NDT configured and active`。地图 target 初始化属于启动工作，完成前不要 enable。

## 6. 终端 C：启动 MPCC（保持禁用）

```bash
ros2 launch aims_mpcc mpcc.launch.py implementation:=acados_cpp \
  simulation:=false cpp_solve_frequency:=20.0 \
  vehicle_config:="$MPCC_BUNDLE/input_config.yaml" \
  path_directory:="$MPCC_BUNDLE/input_reference" \
  artifact_directory:="$MPCC_BUNDLE" \
  log_directory:="$MPCC_RUN/controller" \
  2>&1 | tee "$MPCC_RUN/mpcc.log"
```

没有 shadow 参数。配置、轨迹和 ARM solver 全部从同一个 bundle 读取。不要把旧 `vehicle.yaml` 或 `recordings/current` 混入这次命令。

此时标准 `/drive` 已存在，但启动禁用、速度指令为零。锁车时不能 enable，未 enable 时也不会提交行驶求解请求。

## 7. 终端 D：开始录包

在初始化和 enable 之前开始，保留完整起步和停车过程：

```bash
ros2 bag record -o "$MPCC_RUN/bag" \
  /livox/lidar /livox/imu /livox/imu_bias_corrected /imu/gyro_bias/status \
  /fastlio2/lio_odom /fastlio2/body_cloud /fastlio2/tf \
  /rear_axle/lio_odom /rear_axle/wheel_odom /rear_axle/imu \
  /odometry/filtered /sensors/core /rc/channels \
  /control/autonomy_speed_enabled /drive /ackermann_cmd \
  /commands/motor/speed /commands/servo/position \
  /commands/motor/current /commands/motor/duty_cycle \
  /localization/anchor_status /localization/status \
  /localization/map_valid /localization/map_sha256 \
  /localization/ndt_pose /localization/ndt_status /localization/odom_bridge_pose \
  /mpcc/status /mpcc/reference /mpcc/prediction \
  /tf /tf_static /diagnostics
```

保持自动 topic discovery；某些诊断/控制模式 topic 没有消息不代表失败。不要同时录制另一个 `-a` 包。录包带来的负载是本次运行负载的一部分，后续对比保持相同录制列表。

另开一个可选终端记录 NX 负载：

```bash
tegrastats --interval 1000 | tee "$MPCC_RUN/tegrastats.log"
```

若需要记录功耗模式，操作终端执行 `sudo nvpmodel -q`，把结果记入 notes；本轮沿用当前功耗/时钟配置。

## 8. 操作终端：静止初始化与锁车检查

先把车辆放在轨迹附近，车头朝轨迹前进方向。准备的是 **base_link 在 map 中的初值**，不是 Livox 在 map 中的初值。位置单位米，角度单位弧度；不能把下列输入随意全部设成零。

```bash
read -r -p '输入 base_link 的 x y z roll pitch yaw（米/弧度）: ' \
  MPCC_INIT_X MPCC_INIT_Y MPCC_INIT_Z MPCC_INIT_ROLL MPCC_INIT_PITCH MPCC_INIT_YAW
ros2 run aims_racer_system relocalize_known_map.py "$MPCC_MAP" \
  --pose-frame base_link \
  --x "$MPCC_INIT_X" --y "$MPCC_INIT_Y" --z "$MPCC_INIT_Z" \
  --roll "$MPCC_INIT_ROLL" --pitch "$MPCC_INIT_PITCH" --yaw "$MPCC_INIT_YAW" \
  --timeout 60
```

等待初始化命令返回成功。静止观察 10–20 秒：

```bash
ros2 topic echo /localization/status --once
ros2 topic echo /mpcc/status --once
ros2 topic echo /ackermann_cmd --once
ros2 topic info /drive -v
```

应看到定位 `ready=true`，accepted anchor 序号持续增加；控制器 `backend=acados_cpp`、`command_profile=rate_bounded_v2`、`worker_ready=true`、`enabled=0`，锁车输出速度为零，`/drive` 只有一个 publisher。锁车时 `authority=0` 是正常的；`READY` 表示控制器未启用，不代表所有 enable 前提已经成立。

在 RViz 中以 map 为固定坐标系，确认静止点云与地图重合、车的位置/方向正确、保存参考轨迹在预期位置。`/mpcc/reference` 在 map，`/mpcc/prediction` 在 odom，由 TF 对齐。

此时检查 `/odometry/filtered` 的 frame=odom、child=base_link，静止速度接近零。观察 topic rate 可一次检查一个；不要同时开多个高频大消息 echo。

```bash
ros2 topic echo /odometry/filtered --once
ros2 topic hz /odometry/filtered
```

检查完按 Ctrl+C 退出 `hz`。若定位不 ready、持续跳变、车辆在地图中朝向不对，先解决初始化，不进入行驶阶段。

## 9. 遥控模式与 enable 的正确顺序

当前 CH1 转向、CH3 油门、CH5 锁车、CH6 ESC 模式、CH7 控制来源、CH8 标定、CH10 手动限幅。

| 条件 | 通道条件 / 观察 |
|---|---|
| 锁车 | CH5 < 992 |
| 解锁 | CH5 >= 992 |
| 导航来源 | CH7 >= 992 |
| 速度模式 | CH6 < 582 |
| 标定关闭 | CH8 <= 992 |

开关的实际物理方向以 `/rc/channels` 为准。**CH10 只限制手动油门，不限制 MPCC 的导航速度。**

1. 保持车辆静止、油门中位，人员准备接管。
2. 选择速度模式、导航来源，标定关闭；然后解锁。
3. 控制器仍未 enable，应继续输出零速度。
4. 确认 `/control/autonomy_speed_enabled` 为 true。
5. 调用 enable；**成功返回后车辆即可开始运动**。

```bash
ros2 topic echo /control/autonomy_speed_enabled --once
ros2 service call /mpcc/enable std_srvs/srv/SetBool '{data: true}'
```

返回应为 `success: true`、`message: RUNNING`。若失败，不要循环重试，先按第 12 节定位原因。起步要求车速绝对值 <=0.1 m/s、朝向与轨迹切线差 <=30°；尽量把车放在参考线上，当前并没有 corridor 越界停车。

## 10. 三阶段行驶试验

### A. 0.5 m/s 短段，先检查能否正常运动和停止

运行约 5–10 秒，在进入不便接管的位置前主动停止。观察方向、速度、连续转向是否正常。正常停车：

```bash
ros2 service call /mpcc/enable std_srvs/srv/SetBool '{data: false}'
```

观察从 STOPPING 回到 READY，车辆实际停稳，再锁车。服务返回只表示接受停车请求，不表示车辆已经停稳。第一次不以求完一圈为目标。

### B. 0.5 m/s 完整一圈

短段正常后，停止终端 C/D 的 MPCC 和录包，基础链路与 NDT保持运行。按第 3 节新建 `v05_lap01` 记录目录，其他终端重新 source 当前实验 env。重启 MPCC/录包，静止复查后再 enable。

默认相对于每次 enable 的投影起点完成一圈并停车，最终状态 COMPLETE。观察中途是否出现无计划的 FAULT/RECOVERING、明显左右摆动、持续超目标速度或定位跳变。正常结束后锁车。若摆动明显，停止并保留数据，不在同一轮同时修改权重、时域和门控。

### C. 1.0 m/s

仅在 0.5 m/s 完整一圈表现正常后进行。先停稳、锁车、停止终端 C/D，再创建新实验目录，并在第 3 节将选择改为：

```bash
export MPCC_BUNDLE="$MPCC_BUNDLE10"
```

记录目录标注 `v10_lap01`，其余启动命令不变。配置、轨迹、solver 仍从同一个新选 bundle 读取；已有进程不会因重新 export 而更换配置，必须重启 MPCC。

## 11. 人工停止与收尾

正常停止用 enable=false。需要立即撤销导航权限时，用遥控锁车；需要人工操控时切回手动，先确认油门中位。撤销导航权限后控制器会进入 FAULT、需要停稳并重新 enable，这是预期行为，不是求解器故障。零目标速度也不等于物理车辆瞬时停止。

车辆停稳、锁车后，录包继续约 5 秒再 Ctrl+C，让 bag 正常写出 metadata。随后停止 MPCC、NDT 和基础链路；不要先关电再结束录包。

```bash
ros2 bag info "$MPCC_RUN/bag" > "$MPCC_RUN/bag-info.txt"
printf '\n结果：短段/完整一圈；是否接管；停止原因；是否摆动；场地/初始位置：\n' \
  >> "$MPCC_RUN/notes.txt"
```

补充 notes 中的实际结果。保存 bag、`controller/runtime.csv`、三份 console log、负载日志、配置与哈希；若拍视频，尽量包含车的轨迹与轮子转向，并记录视频对应哪一轮。

## 12. 常见阻塞原因

| 现象 / 返回消息 | 处理 |
|---|---|
| `fresh state, selector authority and actual input history required` | 检查解锁、导航速度模式、标定关闭；检查 EKF 和 `/ackermann_cmd` 新鲜度。不要伪造 authority。 |
| `qualified alignment payload required` | 检查环境脚本是否加载更新的 C++ monitor，而不是旧 monitor。 |
| `matching map, anchor snapshot and fresh trusted localization required` | 检查 NDT ready、地图一致、anchor 是否持续更新。 |
| `start requires stationary vehicle` | 停稳后重试；若静止速度仍偏大，记录并检查估计器。 |
| `start heading outside 30 degrees` | 静止时调整车头/初始化，使其朝轨迹前进方向。 |
| `controller must be sole command publisher` | 停止其他 PP/旧 MPCC/测试程序，确认 `/drive` 只有一个 publisher。 |
| RUNNING 后 `Plan expired` | 保存 runtime.csv 与状态，检查 failed/rejected/late/handover_rejected；不能仅凭此消息认定 NX 求解超时。 |
| 突然定位 lost / epoch changed | 先停稳，检查定位/是否发生重初始化；恢复后显式重新 enable。 |
| COMPLETE 附近停止 | 默认单圈结束行为；本次先保留观察，不调整末端策略。 |

## 13. 这次数据回来后分析什么

- 轨迹：map 中实际轨迹与固定参考的横向/航向误差；不要直接把 odom 曲线叠到 map 参考上。
- 输出：同时看 `/drive` 和实际选中的 `/ackermann_cmd`；转向反向次数与车身响应分开统计。
- 时序：runtime.csv 的完整交付、锁等待、接管、输出间隔及全部失败/拒绝计数。
- 定位：原始 LIO、EKF 与 trusted NDT anchor 的年龄、间隙、跳变，解释控制停止的具体来源。
- 场地表现：起步、弯道、直线、停车分别评估；先记录纵向差异，不将速度 PID 辨识作为本轮完成条件。

相关证据：[接口修订](../reports/2026-10-10-mpcc-standard-interface.md)、[软件/联合负载验证](../reports/2026-10-09-mpcc-runtime-repair-validation.md)。
