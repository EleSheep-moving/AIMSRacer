# VESC 发送与指令积压处理

VESC 子模块固定到 `9e054cced00d1d881524643838eb4f76148657d4`，沿用 100 Hz 遥测/指令链以及两层 watchdog、舵机双重限位、40 A 启动电流和轮速里程计方差扩展。

## 风险核实和处理

原 `SerialPort::async_send` 使用调用者缓冲区且发起 `async_write_some`，临时 packet 和重叠写入会有生命周期、部分写入及帧交错风险。当前 VESC 路径已在先前的 100 Hz 修改中采用同步完整帧写入：持有 packet 字节副本直到写完，并用 mutex 串行发送，处理部分写入。此次保留该实现，并增加实际接口并发压力测试。底层通用库的 `async_send` API 不是当前 VESC 的发送路径。接收缓冲区改为每个接口实例独立持有。

原指令链的订阅深度为 10，在 100 Hz 时可能积累约 100 ms 的历史目标；积压样本进入回调后还会刷新 watchdog。此次将转换器 Ackermann 订阅、驱动六类 actuation 订阅，以及转换器电机/舵机发布队列改为可靠的 `KeepLast(1)`，每个 topic 只保留最新待执行样本。

增加 `command_max_age_sec`，默认 **0.10 s**：转换器和驱动根据 DDS publication timestamp 与系统时间检查样本年龄，过期样本不下发，也不刷新 watchdog。转换器输出同时设置同样时长的 DDS lifespan。年龄参数需有限且非负，0 可关闭年龄过滤及输出 lifespan。使用系统时间使年龄检查独立于 ROS 仿真时钟。对应 QoS 的含义见 [ROS 2 官方说明](https://github.com/ros2/ros2_documentation/blob/humble/source/Concepts/Intermediate/About-Quality-of-Service-Settings.rst)。

边界：已写入内核或物理链路的帧无法撤回；同步串口写入在设备堵塞时仍可能阻塞，不能把这项修改当作硬实时传输延迟保证。跨主机指令源需同步系统时钟；middleware 缺少 publication timestamp 时沿用深度 1，但无法检查年龄。

## 验证

- 三个相关包编译通过，安装的车辆参数与源码一致。
- `interface_writes` GTest：四线程发送 2,000 个临时 packet，收到 1,000 RPM 帧及 1,000 servo 帧，无帧边界/CRC 错误、无发送异常。
- `driver_link` 两个 pytest 场景（50/100 Hz）：原频率、数据解码和五种电机模式 watchdog 测试通过；暂停真实驱动后让 RPM 和舵机消息老化超过 100 ms，恢复时不重放；一批旧 RPM 之后的新目标 8765 正常执行，旧目标未重放。
- `vehicle_extensions` 两个 pytest 场景：保留 watchdog、恢复输入、舵机上下限、40 A 启动电流和默认/覆盖方差；暂停转换器后积压的旧 Ackermann 目标不被执行，新目标可恢复。
- 完整 PTY 链路的 core、两类 IMU、wheel odom、ackermann、电机/舵机命令、servo echo、串口查询和下发帧均实测 **100.0 Hz**，无帧错误。

本次指令压力测试使用模拟串口，未进行实车运动测试。数据保存在 `/home/aims/aimsracer-data/sessions/2026-10-13/vesc-command-freshness/`，CTest 原始结果位于工作区 `build/vesc_driver/test_results` 与 `build/vesc_ackermann/test_results`。

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ctest --test-dir build/vesc_driver -R '^(driver_link|interface_writes)$' --output-on-failure
ctest --test-dir build/vesc_ackermann -R '^vehicle_extensions$' --output-on-failure
```
