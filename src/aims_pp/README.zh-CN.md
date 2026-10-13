# 独立增强型 Pure Pursuit

[English](README.md)

`aims_pp` 将恢复的标定脚本增强 PP 公式实现为独立控制器。运行只依赖
NumPy、PyYAML 与 ROS 2 Humble，不加载 CasADi、acados、SciPy 或 MPCC 求解器。
车辆几何及硬限幅读取真实车辆 YAML：当前默认轴距 0.36 m，最大转角 0.45 rad。

默认前视距离为 `clip(|v|+0.3, 0.3, 4.5)` m。转向公式为
`atan(轴距 × 2 × 前视点横向偏移 / 前视距离²)`，加上 0.1 倍航向误差与
0.1 倍曲率前馈，再限幅。横向误差增益为 1。4–6 m/s 区间内，转角限幅由
车辆硬限幅线性收紧到 0.18 rad。输出速度取参考速度、启动速度上限
（默认 **1 m/s**）及车辆最大速度三者最小值。

```bash
ros2 launch aims_pp pp.launch.py \
  reference_directory:=/绝对路径/参考目录 \
  vehicle_config:=/绝对路径/vehicle.yaml \
  log_directory:=/绝对路径/新运行目录 \
  speed_limit:=1.0 repeat_laps:=false auto_start:=false
ros2 service call /pp/enable std_srvs/srv/SetBool '{data: true}'
ros2 service call /pp/enable std_srvs/srv/SetBool '{data: false}'
```

启动参数为 `reference_directory`、`vehicle_config`、`pp_config`、
`log_directory`、`speed_limit`、`repeat_laps`、`auto_start`、`use_sim_time`。
`pp_config` 留空时使用安装的 `config/pp.yaml`；几何不允许由 PP 调参覆盖。
`auto_start` 必须为 false，需要明确调用使能服务。有日志时若目录已存在
`runtime.csv`，启动失败，避免覆盖原始记录。

节点使用单调时钟定时器以 50 Hz 发布可靠 `KeepLast(1)` `/drive`，消息为
`AckermannDriveStamped`，时间为发布 ROS 时间，坐标系为 `base_link`。
还发布 10 Hz `/pp/status`（`DiagnosticArray`）、持久化 `/pp/reference`
（`Path`，`map`）及 `/pp/lookahead_point`（`PointStamped`，`map`）。
`/pp/enable` 为 `SetBool`。启动默认禁用，停用、故障、完成一圈均输出零速度
及零转角；运行中重复使能会被拒绝，避免重置计圈起点。

参考目录需包含标准 `reference.json`：严格递增的 `knots`、按降幂存储的
五次多项式 `coefficients[6][区间][2]`、`length`、左右宽度、`frame_id=map`，
以及 schema 1 的 `speed_profile`（`positions`、`speeds`）。元数据需包含
闭合圈标志、`source_frame=base_link`、地图 SHA-256 及完整车辆几何
`wheelbase/rear_offset/half_length/front_extent/rear_extent/half_width`。
参考坐标代表后轴。已有 bundle 根目录的标准 JSON 可直接读取，不打开求解器库。
加载器检查所有区间及首尾位置、一阶和二阶导数连续，并核查有限值、采样切向量、
速度、通道宽度及车辆几何，随后才创建 `/drive` 发布者。

可信定位使用 `/localization/status` 中唯一的
`aims_racer_system/localization` 诊断记录。`/localization/anchor_status`
是监视器的上游锚点事件流。需要协议 1、匹配地图哈希和有效标志，以及和 epoch、
锚点序号、锚点源时间绑定的原子 `map_odom_*` 配准。锚点 ROS/单调时间年龄
均不得超过 1 s，健康心跳接收年龄不得超过 0.3 s。刷新 TF 时间不构成定位证据。
仿真时钟冻结时输出定时器仍运行，单调源年龄检查仍可触发停机。
还要求 `/localization/map_sha256`、`/localization/map_valid` 正确，
`odom/base_link` 状态源年龄和接收年龄均不超过 0.1 s，
`/control/autonomy_speed_enabled` 新鲜且授权，最终 `/vesc/command` 合法、
新鲜且处于速度模式。非单位四元数、非有限值、未来时间戳、状态或进度跳变、
前向航向不符、第二个 `/drive` 发布者、定位 epoch 改变或
ROS 时钟回退均触发停机。输入恢复后不会自动继续，需再次明确使能。

保存的左右宽度是参考准备的名义宽度，不代表已经测量的赛道边界。
车身通道检查由车辆 YAML 的 `enforce_corridor` 控制，默认 false。
只有明确设为 true 时，车身超出这些宽度才会触发停机。

默认跑一圈：从使能成功时的后轴投影位置累计进度，刚使能就经过全局参考接缝
不会被误判完成。`repeat_laps=true` 可循环。没有求解器调度、轨迹计划或额外
高阶转向平滑。

`runtime.csv` 记录回调及命令 ROS/单调时间、源年龄、回调耗时、输出间隔、
阶段、速度、转角及累计进度。回调只写入最多 1024 行的队列，单独线程写盘；
丢弃数及写盘错误出现在状态诊断中。

数值 API 为 `PPConfig`、`EnhancedPP`、`NumericReference`、
`LocalizationHealth`、`TrackingInputs`、`PPTrackingRuntime`。
标定模块可调用 `TrackingInputs.localization_reason()` 仅检查定位，
通过 `PPTrackingRuntime.association()` 复用前向航向及由 `enforce_corridor`
控制的可选车身通道检查；
标定模式仍须自己核查最终执行命令及授权。

测试命令为 `python3 -m pytest src/aims_pp/test -q`。ROS 测试需已构建的
Humble 工作区和隔离 `ROS_DOMAIN_ID`。旧代码数值对照测试只解析恢复文件的
数值类测试样本，并核对 SHA 与 Git 来源，不执行硬件模块，也不依赖主机绝对路径。测试验证公式和 ROS 接口，
尚不证明 ARM 实时性或实车跟踪安全。投影采用有界粗采样及 Newton 迭代，
参考自交或相邻分支重叠时需单独验证关联。标准 Humble rclpy 不传递 DDS 消息
信息到回调，最终命令年龄核查依赖源 ROS 时间戳及单调接收时间；可提供消息信息
的适配器还会检查 DDS 源时间年龄。
