ackermann_mux
=========

**简体中文** · [English](README.md)

当前生产安装使用 C++ RC 选择器 `joystick_control_v2`。它选择手动或自主命令并发布 `/vesc/command`；统一整车 launch 提供经过实跑验证的 CH3/CH1 发射机配置。原始 f1tenth/twist_mux 来源归属保留在包元数据和许可材料中。

<a id="rc-controller-variants"></a>
## RC 控制器配置变体

两种配置均使用同一个 C++ 可执行文件 `joystick_control_v2`。通过只读参数 `channel_profile` 选择配置。

| `channel_profile` | 油门 | 转向 | 锁定 | ESC 模式 | 控制来源 | 限制 | 标定 |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| `sequential_ch1_ch2`（默认） | CH1 | CH2 | CH3 | CH4 | CH5 | CH6 | CH7 |
| `steering_ch1_throttle_ch3_aux_ch5_to_ch10`（整车 launch） | CH3 | CH1 | CH5 | CH6 | CH7 | CH10 | CH8 |

CH3/CH1 配置有意不使用 CH2、CH4 和 CH9。发射机需要自定义映射时，仍可通过 ROS 参数覆盖通道参数。

```bash
ros2 run ackermann_mux joystick_control_v2 --ros-args \
  -p channel_profile:=steering_ch1_throttle_ch3_aux_ch5_to_ch10
```


生产安装只包含 `joystick_control_v2`。旧 mux 和 Python 工具不再构建或安装；统一整车 launch 负责启动。

<a id="arbitration-and-calibration-safety"></a>
## 仲裁与标定安全

订阅使用 best-effort QoS，只保留最新样本。选择器以 100 Hz 发布 `/vesc/command` 和 `/control/autonomy_speed_enabled`。锁定和 RC 有效性优先于标定、导航和手动命令。自主标志独立于 `/drive` 的可用性描述 RC 选择，因此命令生产者能够启动，而不会形成 enable 依赖环。

| 参数 | 默认值 | 含义 |
| --- | ---: | --- |
| `publish_rate_hz` | 100.0 | Wall timer 频率；独立于 ROS 仿真时间 |
| `rc_timeout_sec` | 0.2 | 自收到有效 RC 帧后的最大时间 |
| `nav_timeout_sec` | 0.2 | 导航命令接收年龄/源年龄上限 |
| `calib_timeout_sec` | 0.2 | 标定命令接收年龄/源年龄上限 |
| `require_calibration_stamp` | true | 拒绝不含源时间戳的标定命令 |
| `command_speed_limit` | `speed_limit_max_speed` (12.0) | 外部速度绝对上限，m/s |
| `command_current_limit` | 100.0 | 外部电流绝对上限，A |
| `command_duty_limit` | 0.8 | 占空比绝对上限；也是手动旋钮的最大值 |

所有接收年龄计算使用 `std::chrono::steady_clock`。非零源时间戳在接收时与 ROS 时间比较；该源年龄随后随 steady time 推进。这样可拒绝过旧/未来消息，并避免暂停或重置 ROS 时钟延长已接受命令的有效期。命令生产者必须与选择器使用同一个 ROS 时钟。没有时间戳的旧式导航命令使用接收年龄；默认拒绝没有时间戳的标定命令。

标定入口为 `/calib/vesc_command`（`vesc_msgs/VescCommand`），使用显式的速度、电流、占空比或制动电流模式；默认 `brake_max=0`，拒绝非零制动。未选中的目标字段必须为零，frame_id 必须为 `base_link`。进入标定、解锁或 RC 恢复后必须收到新的源时间戳命令，且不得早于待命时刻。检查源时间戳和 DDS 发布时刻后，使用单调时钟累计年龄；过期后下一个选择器节拍发送零速度并回正转向。

加速度模式（4，包括零加速度）保留但明确拒绝。`/drive` 仍使用 Ackermann 速度提议，jerk 必须为零。导航仅支持速度模式。不支持的模式、缺失/过期输入、格式异常的 RC、非有限命令值，以及超出外部速度/电流/占空比上限的命令，均产生明确的零速度命令。转向被钳制到 `steering_limit`。持续 RC 中断时节点继续存活并保持停止；RC 恢复后手动控制可以恢复。标定在恢复之后仍要求新命令。

当前车辆、建图和比赛 launch 均采用经过实跑验证的手动电流范围 **3–100 A**，由 CH10 控制；CH3 控制油门比例，油门死区为 50。直接运行可执行文件而不传入整车参数时，后备范围为 3–20 A；该后备值不是整车 launch 配置。外部标定电流有独立上限。现有 `channel8_*`、`speed_channel8_*`、`current_channel8_*` 和 `steering_channel_mid` 别名继续作为后备；规范参数名优先。所有选择器参数在启动时验证，此后只读。日志在状态变化时记录。

<a id="verification"></a>
## 验证

```bash
colcon build --symlink-install --packages-up-to ackermann_mux
```


本地 native 测试对照 120 条已记录的 Python 手动输出覆盖两种配置，以及超时、待命状态转换、接管、不支持的模式和无效命令。ROS 管线测试仅在隔离 DDS domain 中启动选择器和合成发布者，包含暂停仿真时钟的情况。它们不启动车辆接收器或 VESC 驱动。
回归源码和 fixtures 位于 `tests/`，用于开发；生产安装仅包含选择器可执行文件。

在本机（2026-10-03），隔离合成工作负载采用 100 Hz RC、50 Hz 导航输入和 200 Hz 输出；预热后采样三个 3 s 窗口，原始 Python 选择器测得单 CPU 核心占用 29.0–32.0%，C++ 为 4.3–4.7%。两者均输出约 200 commands/s。这些是进程 CPU 测量，不是最坏延迟上界或实车驾驶测试。

车辆 `vehicle.launch.py` 将 `publish_rate_hz` 显式配置为 **100 Hz**，用于 VESC 指令链；独立组件默认值也为 100 Hz。
