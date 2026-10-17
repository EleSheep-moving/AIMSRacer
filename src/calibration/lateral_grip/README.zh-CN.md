# 横向固定名义转向标定

源码目录 `src/calibration/lateral_grip`，ROS 包名 `aims_lateral_grip`。持续运行的 C++ 节点和独立状态机，执行左转、实际停车确认、右转的固定转向分档车速试验。转角为 `atan(轴距/名义半径)`；该半径只用于设定转角，实际轨迹受侧滑和动力学影响，不能保证等于名义半径。完整纵横向联合滑移试验不在本次范围内。不会自动更新 `mu_safe`、规划器或生产控制器。

默认配置完全禁用运动。复制 `config/disabled.yaml` 到本次试验的绝对路径，明确填写实测轴距、名义半径、递增车速档位、速度/转角限幅、电机电流上限、电压下限、温度上限，确认后将 `physical_configuration_selected` 设为 true。填写真实源码修订/未提交状态 `source_revision`，以及可持久化的 `output_root`。`output_root` 留空时默认解析为 `$HOME/aimsracer-data/sessions/lateral-grip`。没有经过实车验收的示例物理限值；测试文件内数字只用于软件合成测试。

需要已有车辆底盘、选择器、RC、轮速里程计、电机遥测。此 launch 只启动横向节点，不启动底盘或控制器。纵向和横向标定不能同时运行；两者共用 `/calib/vesc_command`，横向独立使用 `/calib/lateral/enable` 服务防止误启动纵向节点，重复发布者会拒绝启动或锁存退出。

```bash
source /opt/ros/humble/setup.bash
source /绝对路径/工作区/install/setup.bash
ros2 launch aims_lateral_grip lateral_grip_calib.launch.py config:=/绝对路径/本次试验.yaml
ros2 run aims_lateral_grip inspect
ros2 run aims_lateral_grip start
ros2 run aims_lateral_grip stop
ros2 run aims_lateral_grip analyze /绝对路径/本次输出目录
```

`aims_racer_system lateral_grip_calib.launch.py` 是兼容入口。默认 `use_sim_time=false`；true 时拒绝驾驶。节点启动保持 IDLE，不自动开始；`start` 显式发送 `/calib/lateral/enable` SetBool true，每个进程只允许一次会话。完成/退出后需重启进程再显式开始。

RC 必须持续新鲜、解锁、标定档；`/control/calibration_enabled` 持续新鲜且为 true。轮速 `/rear_axle/wheel_odom` 的子坐标系必须是 `base_link`。`/sensors/core` 检查故障码、电流、电压、温度。每个带时间戳的控制输入同时检查源时间及稳态接收时间新鲜度、非单调/冻结时戳。RC 和 Bool 原消息没有源时戳，记录明确标注接收 ROS 时间，不能当作采样源时间。

节点只发布 `/calib/vesc_command`，选择器独占 `/vesc/command`。检查两端唯一发布者，以及选择器最终命令与最近请求的有限时间历史一致，包含延迟上限。IDLE 发送零电流以建立反馈；没有 Ackermann jerk 编码。必需控制输入失效、权限丢失、限值超限、最终命令不一致、时钟倒退/冻结会锁存退出并请求零电流。零电流是滑行，不能声称已经停车。方向切换必须轮速持续新鲜且连续低于停车阈值，超时退出，不以等候时间冒充停车。

输出目录有 `metadata.json`（规范参数 SHA256、可执行文件及 SHA256、声明源码版本）、`evidence.csv`（各通道源/接收时间、原始值/协方差/状态、命令请求和最终命令）、周期原子更新及退出最终 `summary.json`。Ctrl-C 保留数据和终止原因；进程关闭仍需要选择器/底盘看门狗，不能把关闭进程等同物理停车。

独立记录 `/state_estimation/{rear_axle,center_of_mass}/{velocity,acceleration,yaw_rate}`，分别为 TwistWithCovarianceStamped、AccelWithCovarianceStamped、Imu，并记录对应诊断。估计质量缺失/异常仅 WARN 并继续记录，不作为所有观察量输出的质量门控；必需控制输入失效仍退出。

离线分析生成 `stage_stats.csv`、`analysis.json`、`gg.png`。AX/AY 必须取自同一条加速度消息，其他异步通道不可伪装源时间对齐，阶段标记基于接收时状态。`Vx*r` 只能作对比代理量，不能当真实横向加速度；G-G 样本不能证明完整轮胎摩擦边界。构建/测试方式及英文细节见 [README.md](README.md)。合成 ROS 测试仅用隔离 domain 227 和 localhost，禁止连接真实车辆或 domain 0。软件测试不等于实车验收或观测器真值验证。

车速按 `ramp_rate`（m/s²）逐步增加。新鲜轮速进入目标 `speed_tolerance` 后连续满足 `settle_seconds`，再满足 `hold_seconds` 及至少 `minimum_hold_samples` 个不同轮速源时戳，才能进入下一档；中途偏离目标会重置稳定/保持计时。`stage_timeout` 使达不到目标或数据不足的阶段按未完成退出，不能按时间直接当作成功。分档 G-G 统计只采用 HOLD 阶段加速度。

启动前及运行中异步验证 `/vesc_driver_node/get_parameters` 返回的 `current_startup_enabled` 为 bool false；true、缺失、类型错误或结果过期均拒绝/退出，防止底盘的启动电流替代隐藏了速度请求。`inspect` 提供配置有效性、驱动器参数就绪、权限、RC、最终命令匹配、唯一发布者及控制输入源/接收年龄（0轮速、1电机、2 RC、3权限、4最终命令），使拒绝原因可检查。

实际估计器诊断订阅 `/state_estimation/{rear_axle,center_of_mass}/status`；另独立记录可选 `/livox/imu`、`/livox/imu_bias_corrected` 原始证据。元数据/证据/汇总写入及 flush 失败会锁存退出，阻止进一步速度请求。离线另写 `radius_vxr_diagnostics.csv`，用至多25ms源时间差的邻近样本计算 Vx/r 半径诊断和 Vx*r 对比，保留各通道原时戳，不能冒充同源采样或真实轮胎力。
