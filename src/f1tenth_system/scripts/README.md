# 标定采集工具（纵向 / 横向）

本目录下目前维护两类标定工具：

- **纵向标定**：建立速度、电流、加速度/减速度之间的关系，用于电流控制和速度规划。
- **横向附着 / 侧滑标定**：通过固定半径圆逐级升速，估计向心加速度、侧向力、等效附着系数和侧滑临界速度。

纵向标定目前有两套采集方案：

- **有定位方法（Pure Pursuit 自动跑圈）**：依赖里程计/定位（Odometry），车辆自动沿“跑道/8字”轨迹行驶；直线执行 Stage A/B/C 纵向采集，弯道自动切 speed mode 稳定通过，更适合长时间、可重复的采集。
- **无定位方法（Manual Steer 遥控介入）**：不依赖定位；纵向由脚本按“直线扫电流 / 弯道定速”发布，横向由遥控器介入，更适合你当前遇到 fastlio/IMU 漂移或定位不稳定时的采集。

两套方案都统一输出话题：`/calib/ackermann_cmd`（中控/下游需按本文的 jerk 约定解析）。

---

## 0.1) 构建与环境准备（必须）

本目录脚本通过 `ament_python` 安装为可执行文件，首次使用或脚本更新后请先编译：

```bash
cd ~/RallyCore
colcon build --packages-select f1tenth_system
```

然后在**当前终端** `source`（按你当前 shell 选择对应脚本）：

```bash
# bash
source install/setup.bash

# zsh
source install/setup.zsh
```

---

## 0.2) `/calib/ackermann_cmd` 消息约定（中控必须按此解析）

本仓库用 `AckermannDriveStamped.drive.jerk` 作为“模式标志位”，用于在同一个 topic 上区分 speed/current：

- `AckermannDriveStamped.drive.jerk == 2.0`：电流模式（current mode）
   - `drive.acceleration` = 电流(A)
   - `drive.steering_angle` = 转向(rad)
   - `drive.speed` 不使用（可置 0）
- `AckermannDriveStamped.drive.jerk == 0.0`：速度模式（speed mode）
   - `drive.speed` = 目标速度(m/s)
   - `drive.steering_angle` = 转向(rad)
   - `drive.acceleration` 可置 0

---

## 0.3) 横向附着 / 侧滑标定：固定半径圆逐级升速

脚本：`src/f1tenth_system/scripts/lateral_grip_calib.py`

用途：主动跑固定半径圆，逐级提高速度，并用 **odom 速度 + IMU yaw rate** 估算横向能力：

- `ay_yaw = v_odom * yaw_rate_imu`
- `mu_y = abs(ay_yaw) / 9.81`
- `Fy = vehicle_mass * ay_yaw`
- `radius_est = v_odom / yaw_rate_imu`

默认车辆质量为 `vehicle_mass:=4.5` kg。第一版只做固定半径圆，不做整条赛道曲率扫描；`/odom.angular.z` 只作为诊断项，因为当前 VESC odom 的角速度可能来自车辆模型。

订阅：
- `/odom`（`nav_msgs/Odometry`，默认速度来源）
- `/livox/imu_ekf`（`sensor_msgs/Imu`，默认 yaw rate / 横向加速度来源）
- `/sensors/core`（`vesc_msgs/VescStateStamped`，遥测记录与安全检查）

发布：
- `/calib/ackermann_cmd`（`AckermannDriveStamped`，`jerk=0.0` speed mode）
- `/calib/lateral_status_text`（`visualization_msgs/Marker`，RViz 状态文本）

安全默认：`armed:=false` 时**不会发布运动命令**，只等待/检查 topic 并打印配置。实车运行必须显式打开：

```bash
ros2 launch f1tenth_system lateral_grip_calib.launch.py \
   armed:=true \
   vehicle_mass:=4.5 \
   test_radius:=3.0 \
   speed_start:=0.5 \
   speed_end:=4.0 \
   speed_step:=0.25
```

常用参数：

```bash
# topic / 输出
ros2 launch f1tenth_system lateral_grip_calib.launch.py \
   armed:=true \
   odom_topic:=/odom \
   imu_topic:=/livox/imu_ekf \
   vesc_topic:=/sensors/core \
   output_dir:=lateral_grip_run1

# 如果 IMU 方向相反，可先低速验证后调整符号
ros2 launch f1tenth_system lateral_grip_calib.launch.py \
   armed:=true \
   imu_yaw_axis_sign:=-1.0 \
   imu_lateral_axis_sign:=1.0
```

输出文件：
- `lateral_grip_samples.csv`：逐帧采样数据。
- `lateral_grip_results.csv`：每个方向/速度点的均值、方差、`mu_y`、`Fy`、半径误差。
- `lateral_grip_summary.yaml`：给后续控制器参考的 `mu_left`、`mu_right`、`mu_safe`、安全横向加速度上限。

侧滑/极限判断：
- IMU yaw rate 与 `v/R` 偏差持续过大。
- 估计半径 `radius_est` 与指令半径偏差持续过大。
- `mu_y` 超过 `mu_abort`。
- 速度继续升高但横向加速度不再增长。
- odom/IMU/VESC 超时或用户中断。

建议先低速验证方向和符号：

```bash
ros2 launch f1tenth_system lateral_grip_calib.launch.py \
   armed:=true \
   speed_end:=1.5 \
   test_radius:=3.0
```

rosbag 录制建议：

```bash
ros2 bag record -o lateral_grip \
   /odom \
   /livox/imu_ekf \
   /sensors/core \
   /calib/ackermann_cmd \
   /calib/lateral_status_text \
   /tf /tf_static
```

---

## 1) 有定位方法：Pure Pursuit 自动跑圈标定（依赖 /odom）

### 1.1 适用场景

- 能稳定提供 `nav_msgs/Odometry`（例如 `/odom` 来自 EKF / fastlio / 轮速里程计融合等）。
- 希望“可重复、可长期跑”的标定采集：直线段自动扫电流、弯道段自动定速返回目标速度区间。

### 1.2 核心脚本

#### (1) `longitudinal_calib.py`：自动跑圈 + 电流/速度分段标定

脚本：`src/f1tenth_system/scripts/longitudinal_calib.py`

订阅：
- `/odom`（`nav_msgs/Odometry`）
- `/vesc/sensors`（`vesc_msgs/VescStateStamped`，用于 ERPM/遥测）

发布：
- `/calib/ackermann_cmd`（`ackermann_msgs/AckermannDriveStamped`）
- `/calib/current_trajectory`（`nav_msgs/Path`，用于 RViz 显示生成的全局轨迹）
- `/calib/lookahead_point`（`geometry_msgs/PointStamped`，用于 RViz 显示 PP 前视点）
- `/calib/status_text`（`visualization_msgs/Marker`，用于 RViz 显示阶段、速度、电流状态）

RViz：可直接打开 `src/f1tenth_system/rviz/pp.rviz`，其中 `Calib Trajectory` / `Calib Lookahead` / `Calib Status` 分别显示全局轨迹、PP 前视点、当前速度/目标速度/电流/阶段状态。

推荐按三阶段运行：

```bash
# Stage A：定速保持，统计每个速度点的平均电流
ros2 run f1tenth_system longitudinal_calib.py --ros-args \
   -p workflow:=pp_speed_hold \
   -p speeds:="[1,2,3,4,5,6]" \
   -p hold_time_sec:=10.0 \
   -p output_path:=speed_hold_current_results.txt

# Stage B：基于 Stage A 的保持电流，对速度区间做电流阶梯加速试验
ros2 run f1tenth_system longitudinal_calib.py --ros-args \
   -p workflow:=pp_accel_interval \
   -p v_start:=1.0 -p v_end:=6.0 -p dv:=1.0 \
   -p base_current_file:=speed_hold_current_results.txt \
   -p current_step:=3.0 -p current_max:=80.0 \
   -p output_path:=speed_interval_accel_results.txt

# Stage C：负电流减速试验
ros2 run f1tenth_system longitudinal_calib.py --ros-args \
   -p workflow:=pp_decel_current \
   -p v_start:=3.0 -p v_end:=8.0 -p dv:=1.0 \
   -p decel_low_speed:=1.0 \
   -p decel_current_step:=3.0 -p decel_current_min:=-20.0 \
   -p output_path:=decel_current_sweep_results.txt
```

常用参数（示例）：

```bash
# 跑道尺寸（建议先从更大半径/更长直道开始，降低横向扰动）
ros2 param set /longitudinal_calib track_radius 3.0
ros2 param set /longitudinal_calib track_straight_length 10.0

# PP 参数（可先用 pp_param_tuner 调好，再搬过来）
ros2 param set /longitudinal_calib lookahead_gain 1.0
ros2 param set /longitudinal_calib min_lookahead 0.3
ros2 param set /longitudinal_calib max_lookahead 4.5
ros2 param set /longitudinal_calib lateral_error_gain 1.0
ros2 param set /longitudinal_calib heading_error_gain 0.1
ros2 param set /longitudinal_calib curvature_ff_gain 0.1

# PP 高速转向限幅（只限制 PP 自动生成的 steering）
ros2 param set /longitudinal_calib max_steering_angle 0.35
ros2 param set /longitudinal_calib steering_limit_start_speed 4.0
ros2 param set /longitudinal_calib steering_limit_full_speed 6.0
ros2 param set /longitudinal_calib high_speed_max_steering_angle 0.18

# 轨迹相对定位坐标系的偏置（现场对齐用，可在线调）
# 默认 true：轨迹局部原点先放到首帧 odom 的 (x,y)，下面的 x/y 是在此基础上的微调
ros2 param set /longitudinal_calib use_first_odom_as_origin true
ros2 param set /longitudinal_calib traj_offset_x 0.0
ros2 param set /longitudinal_calib traj_offset_y 0.0
ros2 param set /longitudinal_calib traj_offset_yaw 0.0
```

运行逻辑（简述）：
- 轨迹为“跑道/8字”闭环：直线段用于采集，弯道段自动 speed mode 通过。
- 轨迹默认在收到第一帧 odom 后，把局部原点平移到该 odom 的 `(x,y)`；`traj_offset_x/y/yaw` 仍可在线微调。如果要恢复“以 odom 原点为轨迹原点”的旧行为，设置 `use_first_odom_as_origin:=false`。
- PP 默认参数已和 `pp_param_tuner.py` 对齐：`lookahead_gain=1.0`、`max_lookahead=4.5`、`heading_error_gain=0.1`。如果超过 2m/s 左右震荡，优先观察日志/RViz 里的 `ld`，再小幅增加 `lookahead_gain/max_lookahead` 或降低 `heading_error_gain`。
- PP 自动转向带速度相关限幅：`v<=4.0m/s` 时最大 `0.35rad`，`4.0<v<6.0m/s` 线性收紧到 `0.18rad`，`v>=6.0m/s` 保持 `0.18rad`。该限制只作用于 `pp_speed_hold / pp_accel_interval / pp_decel_current`，不影响 RC/manual 无定位 workflow。
- `/calib/status_text` 和节点日志会显示 `raw steering`、当前 `steer_limit`、`ld` 以及 `clipped`，用于确认高速是否被限幅、lookahead 是否合适。
- **Stage A / `pp_speed_hold`**：每个速度点先稳定，再只在直线累计 `hold_time_sec` 的电流样本，输出 `speed_hold_current_results.txt`。
- **Stage B / `pp_accel_interval`**：每个区间 `v0→v1` 先回到 `v0`，再只在直线用 current mode 做阶梯电流 trial；入弯会中止当前 trial 并回到 `v0` 重来。
- **Stage C / `pp_decel_current`**：每个高速度点先稳定到目标速度，再只在直线施加负电流减速；入弯会中止当前 decel trial 并重新恢复目标速度。

旧式 `workflow:=pp_auto` 连续扫逻辑已删除；现在 PP 采集只保留 Stage A/B/C。Stage B / `pp_accel_interval` 使用 `current_start_step_index` 和当前列表索引控制起始电流。

结束行为：达到当前模式定义的终止条件、Ctrl-C 或节点退出时，会发布多帧停止命令；先清零 current mode，再发送 `speed=0` 的 speed mode 指令，并日志提示完成。

rosbag 录制建议（用于后处理建模/诊断）：

```bash
ros2 bag record -o pp_calib \
   /odom \
   /vesc/sensors \
   /calib/ackermann_cmd \
   /calib/current_trajectory \
   /calib/lookahead_point \
   /calib/status_text \
   /tf /tf_static
```

#### (2) `pp_param_tuner.py`：仅用于 Pure Pursuit 参数调参（非必需，但强烈建议先跑）

脚本：`src/f1tenth_system/scripts/pp_param_tuner.py`

用途：把 PP 的 lookahead / gain / 高速转向限幅 / 轨迹尺寸等调到“不会扭来扭去、不会冲出弯”的状态，再去跑 `longitudinal_calib.py` 的标定采集。

注意：该节点默认接口与标定节点不同：
- 订阅：`/odometry/filtered`
- 发布：`/drive`
- 可视化：`/calib/current_trajectory`、`/calib/lookahead_point`、`/calib/status_text`，与 `longitudinal_calib.py` 和 `src/f1tenth_system/rviz/pp.rviz` 保持一致。
- 退出：Ctrl-C / 节点退出时会连续发布 `speed=0` 的停止命令。

如果你的系统实际使用的是 `/odom` 或需要输出到 `/ackermann_cmd`，建议用 remap/bridge 适配（不改代码）。

快速运行：

```bash
ros2 run f1tenth_system pp_param_tuner.py --ros-args -p target_speed:=2.0
```

调参节点使用同一套高速转向限幅默认值，也可以在线调整：

```bash
ros2 param set /pp_param_tuner max_steering_angle 0.35
ros2 param set /pp_param_tuner steering_limit_start_speed 4.0
ros2 param set /pp_param_tuner steering_limit_full_speed 6.0
ros2 param set /pp_param_tuner high_speed_max_steering_angle 0.18
ros2 param set /pp_param_tuner use_first_odom_as_origin true
```

推荐流程（建议先稳住横向，再做纵向标定）：

1) 先跑调参：`pp_param_tuner`（低速开始，把 cte/摆动压下去）
2) 再跑采集：`longitudinal_calib`（扩大半径/直道，降低横向扰动）

接口适配（常见 remap 示例，不改代码）：

```bash
# 1) 如果你的定位里程计话题是 /odom（pp_param_tuner 默认订阅 /odometry/filtered）
ros2 run f1tenth_system pp_param_tuner.py --ros-args \
   -r /odometry/filtered:=/odom

# 2) 如果你的车辆执行入口是 /ackermann_cmd（pp_param_tuner 默认发布 /drive）
ros2 run f1tenth_system pp_param_tuner.py --ros-args \
   -r /drive:=/ackermann_cmd
```

---

## 2) 无定位方法：RC 介入标定采集（无定位依赖）

目的：绕开 fastlio/IMU 漂移问题，让采集过程不依赖定位。

现在推荐统一使用 `longitudinal_calib.py`，通过 `workflow` 切换到无定位采集流程：
- `workflow:=speed_hold`：阶段 A，定速保持并统计平均电流。
- `workflow:=accel_interval`：阶段 B，对每个速度区间做电流阶梯加速试验。
- `workflow:=decel_current`：阶段 C，可选负电流减速试验。

RC 介入逻辑（与中控配合）：
- **直线**（遥控器方向在死区内）：允许采样/试验，必要时强制 `steering=0`。
- **弯道**（遥控器方向超死区）：暂停采样/试验，并用 speed mode 维持安全速度。
- **横向（steering）**：由遥控器介入；回正后等待 `post_turn_settle_sec` 再恢复采样，避免弯道残余电流污染。

输出：`/calib/ackermann_cmd`

> 注意：`jerk=2.0` 仍表示 current mode，`jerk=0.0` 仍表示 speed mode。

---

## 3) 新标定流程（推荐）：先定速测电流，再分速度区间测加速度

你反馈原方法效果不佳时，建议改为“两阶段”采集：

### 3.1 阶段 A：定速 1..8 m/s，测保持速度的平均相电流

目标：尽量在 **直线（steering=0）** 的情况下，分别测试 1、2、...、8 m/s 的保持电流；每个速度保持 `hold_time_sec`（默认 10s，可人工设置）。

建议：阶段 A 为了减少横向扰动，推荐关闭 RC 介入（`use_rc_steering:=false`），或确保遥控器方向始终在死区内。

> 如果开启了 `use_rc_steering:=true`：脚本会在“转弯（出死区）”期间不采样；并在“转弯结束回到直线（进死区）”后额外等待 `post_turn_settle_sec`，再开始采样，避免弯道残余电流抬升污染结果。

脚本：`src/f1tenth_system/scripts/longitudinal_calib.py`

运行示例：

```bash
ros2 run f1tenth_system longitudinal_calib.py --ros-args \
   -p workflow:=speed_hold \
   -p speeds:="[1,2,3,4,5,6,7,8]" \
   -p hold_time_sec:=10.0 \
   -p use_rc_steering:=false \
   -p vesc_topic:=/sensors/core \
   -p output_path:=speed_hold_current_results.txt
```

（如确实需要 RC 介入运行）

```bash
ros2 run f1tenth_system longitudinal_calib.py --ros-args \
   -p workflow:=speed_hold \
   -p use_rc_steering:=true \
   -p rc_topic:=/rc/channels \
   -p rc_timeout_sec:=0.25 \
   -p post_turn_settle_sec:=0.8
```

输出：
- `speed_hold_current_results.txt`：每个速度对应的平均相电流（默认策略：先等待车速达到目标并稳定一段时间，再开始采样）
- 可选 `csv_path`：记录全量时间序列，便于排查

#### 3.1.1 电压 / SOC 分层采集建议

`I0(v)` 不可能在所有电池状态下都完美成立。第一版建议把 Stage A 按负载电压分成 3 层采集，后处理时形成二维基线表：

```text
I0 = I0(v, V)
```

电压分层以 VESC 的 `/sensors/core.state.voltage_input` 为准，优先使用跑动过程中的负载电压，而不是静置空载电压：

```text
High: voltage_input >= 15.8V
Mid:  15.4V <= voltage_input < 15.8V
Low:  15.0V <= voltage_input < 15.4V
```

建议流程：
- Stage A：High / Mid / Low 三个电压层各跑一次 `2~6m/s` speed hold，得到 `I0(v,V)`。
- Stage B：至少 High / Mid 两个电压层跑 `2->6m/s` accel interval；如果低电压安全，再补 Low。
- 低压边界必须服从电池和 VESC 安全阈值；不要为了补 Low 数据强行跑到危险电压。

参考后处理公式：

```text
# Stage A：每个速度点 / 电压层的保持电流
I0(v_i, V_bin) = mean(I_q)
V_bin_mean(v_i) = mean(voltage_input)

# Stage B：每次加速 trial
a_fit = slope(linear_fit(t, v_odom))
I_q_mean = mean(state.avg_iq)       # fallback: state.current_motor
V_mean = mean(voltage_input)
I_net = I_q_mean - interp2d(I0_table, v_mid, V_mean)
v_mid = 0.5 * (v0 + v1)

# 标定净电流效率
a_fit = I_net * (k0 + k1*(v_mid - v_ref) + k2*(V_mean - V_ref))
```

最终前馈反算建议使用：

```text
I_cmd = I0(v,V) + a_ref / k_eff(v,V)
k_eff(v,V) = k0 + k1*(v - v_ref) + k2*(V - V_ref)
k_eff(v,V) = clamp(k_eff, k_min, k_max)
```

### 3.2 阶段 B：对每个速度区间 v→v+1，做电流阶梯扫描测加速度

目标：对 1→2、2→3、...、7→8 m/s 区间，基于阶段 A 的“基准电流”起步，以 `current_step`（默认 3A）逐步增大到 `current_max`（默认 80A），测到达时间 $t$，估算加速度 $a = \Delta v / t$。

如果已经确认“基础电流”只会维持匀速、没有必要重复跑，可以设置 `current_start_step_index:=1`，让每个区间从 `base_current + current_step` 开始；默认 `0` 表示仍从 `base_current` 开始。

脚本：`src/f1tenth_system/scripts/longitudinal_calib.py`

运行示例：

```bash
ros2 run f1tenth_system longitudinal_calib.py --ros-args \
   -p workflow:=accel_interval \
   -p v_start:=1.0 -p v_end:=8.0 -p dv:=1.0 \
   -p base_current_file:=speed_hold_current_results.txt \
   -p current_step:=3.0 -p current_start_step_index:=1 -p current_max:=80.0 \
   -p vesc_topic:=/sensors/core \
   -p odom_topic:=/odom \
   -p output_path:=speed_interval_accel_results.txt
```

（如确实需要 RC 介入运行，且只在直线段做 trial）

```bash
ros2 run f1tenth_system longitudinal_calib.py --ros-args \
   -p workflow:=accel_interval \
   -p use_rc_steering:=true \
   -p rc_topic:=/rc/channels \
   -p rc_timeout_sec:=0.25 \
   -p post_turn_settle_sec:=0.8
```

输出：
- `speed_interval_accel_results.txt`：每条记录为 `v0 v1 current_A t_sec accel reached`

备注：该脚本会在每次试验前先用 speed mode 把车“拉回/稳定到 v0”，再切到 current mode 计时；这样能尽量隔离“速度稳定后→升档加速段”的电流影响。

> 速度反馈来自 `odom.twist.twist.linear.x`（由 `odom_topic` 指定），电流反馈来自 `vesc_topic`。

#### 3.2.1（可选）减速电流测试模式：每个速度点 v 做 v→1m/s 的负电流减速

同一个脚本支持 `workflow:=decel_current`：对 `v_start..v_end`（步长 `dv`）每个“高速度点” v，先 speed mode 跑到 v；然后按电流 `0, -step, ... , current_min` 做减速，直到速度低于固定低速阈值 `decel_low_speed`（默认 1m/s）；立刻切 speed mode 回到 v，再测下一个电流。

运行示例：

```bash
ros2 run f1tenth_system longitudinal_calib.py --ros-args \
   -p workflow:=decel_current \
   -p v_start:=3.0 -p v_end:=8.0 -p dv:=1.0 \
   -p decel_low_speed:=1.0 \
   -p decel_current_step:=3.0 -p decel_current_min:=-20.0 \
   -p max_speed_during_rc:=5.0 \
   -p vesc_topic:=/sensors/core \
   -p odom_topic:=/odom \
   -p output_path:=decel_current_sweep_results.txt
```

（如需固定停止阈值/恢复速度，可显式覆盖）

- `decel_low_speed`：默认停止阈值（默认 1.0m/s）
- `decel_stop_speed_threshold`：停止阈值强制覆盖（默认不启用）
- `decel_baseline_speed`：恢复速度（默认用 `v`，不启用则为 NaN）

输出：
- 每条记录为 `target_speed start_speed current_A t_sec decel_mps2 stopped`

RC 介入规则（`use_rc_steering:=true`）：转向出死区会暂停 current mode 并用 speed mode 保持当前速度（但不超过 `max_speed_during_rc`）；回正后会按规则恢复并重新开始该次减速记录。

### 3.3 离线验证纵向模型

脚本：`src/f1tenth_system/scripts/longitudinal_model_verify.py`

第一版验证模型暂时不把温度放进主方程，重点比较三种净电流模型：

```text
I_net = I_q_mean - I0(v0)

Model A: a = k * I_net
Model B: a = I_net * (k0 + k1*(v_mid - v_ref))
Model C: a = I_net * (k0 + k1*(v_mid - v_ref) + k2*(V_mean - V_ref))
```

其中 `I_q_mean` 优先用 `state.avg_iq`，备选 `state.current_motor`。温度后续更适合作为热降额/限流诊断量；电压和 duty 会输出到验证表里，用于判断电池压降和 duty 饱和。当前脚本仍使用 Stage A 的一维 `I0(v0)`，等 High / Mid / Low 电压层数据足够后，再升级为二维基线：

```text
I_net = I_q_mean - I0(v_mid,V)
I_cmd = I0(v,V) + a_ref / k_eff(v,V)
```

如果只有 Stage A/B 文本结果，可以先用命令电流快速验证：

```bash
python3 src/f1tenth_system/scripts/longitudinal_model_verify.py \
   --base speed_hold_current_results.txt \
   --samples speed_interval_accel_samples.csv \
   --valid-v-end 6.0 \
   --out-prefix longitudinal_model_verify_csv
```

如果有 rosbag，推荐从 bag 里重建 `avg_iq / voltage / duty / accel_fit`：

```bash
python3 src/f1tenth_system/scripts/longitudinal_model_verify.py \
   --base speed_hold_current_results.txt \
   --bag /home/nuc/RallyCore/bag/pp_accel_interval2 \
   --valid-v-end 6.0 \
   --model-accel-max 2.5 \
   --out-prefix longitudinal_model_verify_bag
```

如果要用两包或多包做电压调制粗验证，使用 `--bags` 聚合：

```bash
python3 src/f1tenth_system/scripts/longitudinal_model_verify.py \
   --base speed_hold_current_results.txt \
   --bags /home/nuc/RallyCore/bag/pp_accel_interval1 /home/nuc/RallyCore/bag/pp_accel_interval2 \
   --valid-v-end 6.0 \
   --model-accel-max 2.5 \
   --out-prefix longitudinal_model_verify_pp_accel_1_2_voltage
```

输出：
- `*_trials.csv`：每次 trial 的 `bag_id`、`bag_path`、`v_mid_mps`、`iq_mean_a`、`i_net_a`、`voltage_mean_v`、`duty_max`、`accel_fit_mps2`、有效/剔除原因。
- `*_model.json`：保留分速度区间/全局旧式 `k`、`b`、`R2`、`RMSE`，并新增 aggregate Model A/B/C 的系数、`R2`、`RMSE`、电压范围、duty 范围，以及 Model C 相比 Model B 的 RMSE 改善幅度。

rosbag 录制建议（强烈建议，便于补齐“轮速/加速度/延迟/介入”诊断）：

必录（纵向建模最小集）：

```bash
ros2 bag record -o longitudinal_calib \
   /odom \
   /sensors/core \
   /calib/ackermann_cmd \
   /rc/channels
```

可选（用于交叉验证/链路排查，按你车上实际存在的 topic 选择）：
- `/vesc/sensors`：有些 launch/节点用这个名字发布 VESC 遥测（你当前是 `/sensors/core`，优先录它）
- `/ackermann_cmd` 或 `/drive`：如果你系统实际生效入口不是 `/calib/ackermann_cmd`，录一份便于确认 remap/执行链路
- `/tf` `/tf_static`：如果你需要把速度与定位/姿态对齐做诊断
- `/imu` `/livox/imu`：用于排查 IMU 噪声/漂移导致的速度估计异常（不是纵向标定必需）

---

### 3.4 离线分析 Prompt（阶段 A+B(+减速) 输出文件 + 可选 rosbag）

把下面 prompt 里的路径替换成你实际生成的文件路径，直接丢给分析助手即可。

```text
你是车辆纵向动力学/标定工程师。请基于采集输出文件（加速/减速）与可选 rosbag，建立“速度分段的电流→加速度/减速度映射”，并给出可落地的查表/拟合结果与质量诊断。

输入文件（把路径替换成实际文件）：
1) 阶段A：speed_hold_current_results.txt
   - 格式：speed_mps  mean_current_A  std_current_A  samples
2) 阶段B：speed_interval_accel_results.txt
   - 格式：v0  v1  current_A  t_sec  accel_mps2  reached
3) 阶段C（减速，可选）：decel_current_sweep_results.txt
   - 格式：target_speed  start_speed  current_A  t_sec  decel_mps2  stopped
可选：
- 阶段A/阶段B 的 csv_path（若存在），用于排查异常与可视化
- rosbag 目录（若存在）：用于重建“轮速/加速度时间序列”、做延迟与一致性诊断
  - 建议包含：/odom、/sensors/core、/calib/ackermann_cmd、/rc/channels

采集约定/重要前提：
- 弯道期间不采样/不做 trial；回正后会等待 post_turn_settle_sec 再开始采样/试验。
- 阶段B：每次 trial 前会先用 speed mode 拉回并稳定到 v0，再切 current mode 计时加速到 v1。
- 速度反馈来自 /odom.twist.twist.linear.x，电流反馈来自 VESC 遥测（相电流/iq 等字段）。
- 重要：加速度不仅与电流有关，也与当前速度有关（反电动势/占空比上限/电池电压下垂等因素会导致“同样电流在不同速度下的有效扭矩/加速度不同”）。因此建模时应把速度因素显式纳入（分速度段或二维模型）。
- 阶段C：每次 trial 先 speed mode 稳定到 target_speed，再切 current mode 负电流减速；当速度低于低速阈值（默认 1m/s）即结束并切回 speed mode。
- 若 use_rc_steering=true：遥控出死区会暂停/重启 trial（bag/日志可用于识别被介入打断的区段）。

你需要完成的工作：
1) 读取与检查
   - 解析两个 txt，列出阶段A速度点的数量、速度范围、是否有缺失/NaN
   - 解析阶段B所有 trial，统计 reached=1 的比例；按区间(v0->v1)分组统计样本数
   - 如存在阶段C：统计 stopped=1 的比例；按 target_speed 分组统计样本数
   - 找出明显异常：t_sec<=0、accel_mps2 非法、current_A 乱序或重复等

2) 阶段A：基线电流曲线
   - 画出 I_base(v)=mean_current_A vs v
   - 对 I_base 做平滑/插值（例如线性插值或分段拟合），输出一个可查询的函数/表
   - 给出每个速度点的 std 与样本数，提示置信度

3) 阶段B：建立“电流→加速度”模型（按速度区间分段）
   对每个速度区间（例如 1->2, 2->3, ...）：
   - 只使用 reached=1 的 trial 进行拟合
   - 画出散点：current_A vs accel_mps2
   - 提供两种建模结果并对比：
     A) 直接拟合：a = k*I + b（线性回归）
     B) 扣基线电流：I_eff = I - I_base(v0)，拟合 a = k*I_eff + b 或 a=k*I_eff
   - 给出每段的 k,b，R²、RMSE，并指出是否存在非线性/饱和（高电流段加速度提升变慢）

   额外建议（用于体现“反电动势/速度相关性”）：
   - 除了按 (v0->v1) 分段，也可以直接拟合二维关系：a = f(v, I)（例如 a = k1*I + k2*v + k3 或分段面片/查表）。
    - 推荐把“扭矩电流”与“饱和指标”分开处理：
       - I_torque：优先用 `/sensors/core` 的 state.avg_iq（或 state.current_motor）
       - v：用 /odom.twist.twist.linear.x
       - 饱和/电压限制诊断：state.duty_cycle、state.voltage_input
    - 一个可落地的拟合形式（示例，不限定）：
       - 分速度段：在每个速度桶内拟合 a = k*I_torque + b
       - 或二维：a = k1*I_torque + k2*v + k3，并对 |duty_cycle| 接近 1 的样本单独标注/剔除
    - 如果 bag 中能拿到电池电压/占空比/电机侧速度（都在 `/sensors/core` 的 state 字段里），可进一步做归一化或诊断：同一速度下电压下降、占空比接近饱和会导致可用加速度变差。

4) 输出可落地结果（非常重要）
   - 生成“查表形式”的结果：对每个速度区间输出一张 I->a 的单调表（必要时做单调化）
   - 或输出分段线性参数表：每段 (v0->v1) 的 k,b，以及推荐使用范围
   - 给出反算形式建议：给定目标加速度 a* 时，如何得到电流指令 I*
   - 建议加入约束：I_max、最小可用电流、超时/未到达的处理

5) （如有阶段C）建立“负电流→减速度”模型
   - 对每个 target_speed：画 current_A vs decel_mps2 的散点
   - 输出可用的 braking 查表或拟合参数（允许与加速模型不对称）
   - 指出 0A 时的自然滑行减速度（滚阻/风阻/坡度的体现）

6) （如有 rosbag）用时间序列补齐轮速/加速度诊断（建议做，但不是硬性要求）
   - 从 /odom.twist.twist.linear.x 计算 a(t)：说明差分方式、滤波/平滑参数、端点处理
    - 从 /sensors/core 提取并对齐以下字段（vesc_msgs/VescStateStamped）：
       - 相/扭矩相关电流：优先用 state.avg_iq；备选 state.current_motor（单位 A）
       - 电池侧电流：state.current_input（单位 A）
       - 电池电压：state.voltage_input（单位 V）
       - 占空比：state.duty_cycle（通常在 [-1, 1]）
       - 电机侧速度：state.speed（注意：该字段常是 ERPM/电机侧量，单位可能不是 m/s；建议主要用于相对变化/延迟诊断，不直接当车速）
    - 与 /odom 做交叉验证（比例/延迟/符号），并识别“反电动势/电压限制”导致的饱和区：
       - 当 |duty_cycle| 接近 1 或 voltage_input 明显下垂时，同样的 iq/电流指令可能对应更小的可用加速度
       - 这部分应在模型里显式体现（速度分段/二维模型），或在拟合时标注并剔除饱和样本
    - 可计算电功率用于诊断（非强制）：P_elec = voltage_input * current_input（W）；比较 P_elec 与 v*a 的量级变化，排查异常/符号/延迟
   - 用 /calib/ackermann_cmd.drive.jerk 区分 speed/current mode，并对齐每个 trial 的开始/结束时刻
   - 标注并剔除遥控介入（/rc/channels 出死区）的区间，或单独统计其影响

7) 质量诊断与下一轮建议
   - 哪些区间数据太少/噪声大（建议增加 repeats 或延长 reset/stable 时间）
   - 哪些电流档经常 timeout（提示 current_max、trial_timeout_sec 或 dv 需要调整）
   - 建议的 post_turn_settle_sec、speed_tolerance、stable_required_sec 的经验取值范围

输出要求：
- 给出关键图表列表（你会画哪些图、每图说明）
- 给出最终建议的表格/参数（清晰列出每个区间的 k,b、以及 braking 侧的对应表/参数，或查表点）
- 如果需要插值，说明插值方法与边界处理方式（超出速度范围怎么办）
```

---

> `/calib/ackermann_cmd` 的 jerk 模式约定见上方「0.2) 消息约定」。

---

## 重要提示（请先读）

- 本文当前**主要维护/推荐**的是上面的「3) 新标定流程（两阶段：A 定速测电流 + B 分速度区间测加速度）」。
- 旧的 `speed_hold_current_logger.py`、`speed_interval_accel_sweep.py`、`manual_steer_speed_stages.py`、`manual_steer_current_sweep_stages.py` 已删除；对应能力已由 `longitudinal_calib.py` 的 `workflow:=speed_hold/accel_interval/decel_current` 覆盖。
- 从这里往下的内容以“补充资料/录包与分析说明”为主；如有不一致，**以脚本代码与上方第 3 节为准**。

---

### 1) rosbag 录制

推荐（纵向标定/减速扫电流，最小集）：

```bash
ros2 bag record -o longitudinal_calib \
   /odom \
   /sensors/core \
   /calib/ackermann_cmd \
   /rc/channels
```

扩展（需要做 IMU/定位/执行链路诊断时再加）：
- `/imu` `/livox/imu`
- `/tf` `/tf_static`
- `/ackermann_cmd` 或 `/drive`（实际生效入口）
- `/vesc/sensors`（若你系统用这个命名）

你当前的命令（全量版本，没问题）：

```bash
ros2 bag record -o manual_control1 \
  /odom \
  /odometry/filtered \
  /imu \
  /livox/imu \
  /livox/imu_ekf \
  /sensors/servo_position_command \
  /ackermann_cmd \
  /sensors/core \
  /rc/channels \
  /calib/ackermann_cmd
```

---

### 2) 分析 rosbag 的 Prompt（可直接丢给分析助手）

把 `{bag_dir}` 替换成你的 bag 路径。

```text
你是车辆纵向动力学/标定工程师。请基于一次“手动转向介入 + 三阶段定速”的 rosbag 数据，建立速度分阶段/分桶的纵向模型，并评估电流与加速度的关系。

背景与约定：
- 上游节点发布 /calib/ackermann_cmd（AckermannDriveStamped）。本方案 drive.jerk=0.0（speed mode）。
- 当遥控器方向在死区内：认为直线（steering=0）。
- 当遥控器方向超出死区：认为弯道（steering=遥控器值）。
- 电机遥测来自 /sensors/core（VescStateStamped），包含 speed、avg_iq（电流相关）等。

数据位置：
- rosbag 目录：{bag_dir}

请你完成：
1) 数据检查：列出 bag 中所有 topic、消息数、估计频率；检查时间戳是否单调、是否有长间隙。
2) 对齐与派生量：
   - 用 /sensors/core/state/speed 和 /odom.twist.twist.linear.x 互相交叉验证车速（说明可能的比例/延迟差）。
   - 从 /odom 速度计算加速度 a（说明差分、低通滤波/平滑、延迟补偿策略）。
3) 直线/弯道分段：
   - 首选用 /rc/channels 的转向通道按“与 joystick_control_v2 一致的死区规则”判定直线段；
   - 备选用 /calib/ackermann_cmd.drive.steering_angle 绝对值阈值判定。
4) 分阶段统计：
   - 按三阶段目标速度区间分段（或按速度区间自动聚类）。
   - 每阶段输出：平均速度、速度方差、平均电流(或 avg_iq)、平均加速度、加速度噪声水平。
5) 建模与结论：
   - 给出至少一个可落地的模型：例如 a = k(v)*I + b(v)（k、b 随速度分段常数），或直接用查表策略。
   - 报告 R² / RMSE，并指出主要误差来源（弯道横向扰动、地面坡度、电池电压变化、延迟）。
6) 质量诊断：
   - 是否存在“直线段仍有转向输入/车辆摇摆”的污染；
   - 是否存在电流饱和、死区或速度闭环震荡。
7) 下一轮采集建议：
   - 每阶段应该维持的最小直线时间、建议的目标速度范围；
   - 需要补录的 topic（若缺失就说明影响）。
8）考虑占空比达到最大的情况，并给出这种情况下的标定代码建议

输出要求：
- 给出最终建议的映射形式（公式/分段参数/查表），以及你使用的筛选规则（阈值、时间窗口、滤波参数）。
```

---

### 3) 备注：关于“/calib/ackermann_cmd”如何接入

本节点**只负责上游发布** `/calib/ackermann_cmd`。
如果你当前的控制链路不是直接使用该 topic，请用 launch remap 在不修改任何上下游代码的前提下接入。

示例（把输出 remap 到真正生效的 ackermann 入口）：

```bash
ros2 run f1tenth_system longitudinal_calib.py \
  --ros-args -p workflow:=speed_hold \
  -r /calib/ackermann_cmd:=/ackermann_cmd
```
