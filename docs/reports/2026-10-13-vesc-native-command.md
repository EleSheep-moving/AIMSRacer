# 2026-10-13 VESC native command migration

本次只进行软件验收。未打开车辆串口，未发送任何实机电机/转向指令（包括零速），未启动整车 launch。所有 ROS 测试使用隔离 DDS domain，驱动测试只连接 `/dev/pts/*` 虚拟串口。实机试验必须等车主到现场并明确指示。

## 实现

- 新增 `vesc_msgs/VescCommand`，显式 speed/current/duty/brake-current/acceleration 模式，速度字段为 `target_speed_mps`。Header 为中控发布 ROS 时间和 `base_link`；非选中目标严格为零，全字段有限。
- 中控 100 Hz 发布 `/vesc/command`。RC 电流档保留，档位、限幅旋钮、方向、优先级、RC 超时及标定重新待命逻辑保留。标定改为 `/calib/vesc_command`。
- 删除 `/ackermann_cmd` 控制链、六个 Float64 执行订阅和 `ackermann_to_vesc_node`。无 `/vesc/applied_command`。
- 驱动负责 m/s→ERPM、rad→servo、电流启动辅助；电机与 servo 在单个串口写锁内成对发送。单消息 KeepLast(1)、100 ms lifespan、Header/DDS 源时间校验避免旧模式与目标积压。
- 单调时钟 100 ms watchdog；无论转换 offset 或 ERPM minimum 如何配置，零速度、异常停止与 watchdog 都发送字面值零 ERPM；驱动超时保留转向，中控 RC/源超时零速并回正。
- 40 A 电流启动辅助默认保留，运动确认使用新鲜遥测和单调时钟，可通过 `current_startup_enabled=false` 禁用。消息中的电流是请求目标，启动期间实际发送目标可以不同。
- 加速度模式（包括零请求）保留但明确拒绝；没有临时加速度→电流比例换算。默认 `brake_min=brake_max=0`，禁止非零制动；仅虚拟串口测试显式启用正制动上限。
- MPCC 反馈读取新消息的 m/s 和 rad；只允许速度模式进入历史。保留 100 ms 内、速度/转角差值小于 1e-6 的自身输出匹配及内部加速度/转向角速度元数据。非速度模式撤销正在运行的自动控制。`/drive` 提议仍为 50 Hz，优化请求及模型未改动。
- core/IMU 100 Hz、921600 baud、轮速里程计和默认 0.04 方差、servo command echo 保留。新录包替换控制话题，并保存实际选用 VESC 配置文件；历史录包和实验统计不改写。

完整消息语义见 [控制接口](../vesc-command.md)。

## 软件验收

隔离构建目录：`/home/aims/aimsracer-data/sessions/2026-10-13/vesc-native-command`。

- 六个改动包隔离构建成功。
- 功能 CTest/pytest 汇总 162 项通过，零错误、零失败、零跳过；未包含仓库的通用格式 lint。新增配置快照检查与最终链路复核另有 13 项通过。
- 120 组历史 Python 手动遥控输出经过新模式字段映射后保持一致；RC 断流、锁定、导航权限及标定重新待命回归通过。
- 四线程发送 2000 组电机/转向命令，共 4000 帧，CRC 与配对连续性通过。
- PTY 驱动测试覆盖 50/100 Hz 遥测、100 Hz 指令、正反向与限幅、非零 offset 下零速、各模式、默认制动禁用、40 A 启动与禁用、无效消息、DDS 排队、模式替换、暂停及回退 ROS 时间。
- 全链路测试使用真实安装的中控、驱动、里程计和虚拟 VESC。3 秒窗口去掉首尾各 0.2 秒，测得命令、core、IMU、wheel odom 均为 **99.97 Hz**。旧控制话题没有发布者/订阅者，中控暂停后 driver watchdog 停止虚拟目标。
- MPCC 共 25 项模型/自动启动/定位时钟回归通过；其中新增 native 反馈测试验证单位、近期输出匹配、过期消息拒绝和非速度模式退出。
- 主系统 launch、录包、后轴传感器与估计器回归通过。上述结果不代表电机运动、实机制动或加速度控制已验收。

正式安装再次验证：全链路与 launch/录包共 13 项通过；正式 MPCC native 反馈 CTest 通过；安装中只剩 `vesc_to_odom_node`，不含旧 converter 入口。

## 部署与回退

正式工作区重新构建六个改动包，不启动节点。旧 `install/vesc_ackermann` 已备份至上述 session 的 `previous-install/vesc_ackermann`，重新生成干净安装以清除遗留 converter 可执行文件和 launch。

修改前主仓库为 `95091264a9503f612248c4d8f5e1a434cd4a0dd8`，VESC 子仓库为 `9e054cced00d1d881524643838eb4f76148657d4`。需要回退时，在保存当前工作后恢复该主仓库提交及其子模块并重新构建；仅切换源码不会恢复安装中的旧接口。

VESC 子模块更新为 `ca5a6e5245fd8f601af13d0315d23e3de2c5c3cc`，已先推送至 `EleSheep-moving/vesc` 的 `main`。
