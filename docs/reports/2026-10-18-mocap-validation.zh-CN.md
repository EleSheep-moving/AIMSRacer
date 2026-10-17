# 动捕校对工具交付与验证

已按批准方案实现，并同步到主机与 NX 的 `AIMSRacer/src/mocap_validation/`。持续时间采集为 C++，离线参考、分析和绘图为 Python；工具只采集分析，动作仍沿用遥控、现有标定程序或 MPCC。

[使用说明与完整命令](../operations/mocap-validation.zh-CN.md) · [现场配置模板](assets/2026-10-18-mocap-validation/config.template.json) · [固定来源与提交](assets/2026-10-18-mocap-validation/sources.lock.json)

## 已交付

| 工作 | 可查看的结果 |
|---|---|
| 状态估计 | 后轴和质心 VX/VY/AX/AY/yaw rate；未平移、未去偏误差；三种求导窗口敏感性；覆盖率、缺口；静止、直线、转弯、S 弯分项；消息时间年龄与独立波形偏移诊断 |
| FastLIO | raw 与后轴结果分开；安装旋转、世界/车体系速度与杆臂换算；固定坐标对齐或指定完整静态窗口只估计一次；位姿、速度、漂移、时间年龄与响应偏移 |
| 执行器 | 最终 `/vesc/command` 的速度、电流、占空比、制动、转向事件；到 ERPM/Vicon VX/AX/yaw rate/AY 的响应范围、持续确认和阈值敏感性；不可辨识原因逐事件保留 |
| 共用采集 | 原始/纠偏 IMU、两参考点五量、FastLIO、导航 EKF、轮速、指令、反馈同时录包；C++ 另存源/DDS 发布/DDS 接收/回调 ROS/系统/单调时间和 CDR 哈希 |
| 可追溯性 | 选定配置、实际参数、可读取本机进程 ELF/启动参数、采集程序版本和哈希、原包/提取文件哈希、队列丢失和时间异常 |

统一命令 `inspect / record / extract / analyze --task state|fastlio|actuator|all / demo`。Ctrl-C 等待录包与时间文件落盘；磁盘写入、最终 flush、健康文件损坏或录包收尾失败会返回失败并尽可能保留最终清单。没有新车辆控制发布者，也没有修改生产节点埋点。

## 版本、构建和部署

- 工具包 `aims_mocap_validation 0.1.0`。
- `vrpn`：ROS Humble release `7.35.0-11`，提交 `ff688a4507797c35a48710be678c06defc5e6629`。
- `vrpn_mocap`：`1.2.0`，提交 `1414a4fdc570c1b5b8c7a17a32a29eaa8bb87895`。
- 两套官方源码及许可证随目录保存；桥明确启用 `use_vrpn_timestamps=true`。
- NX CMake 4 的兼容选项由本工具构建脚本提供，线程支持经实际探测后再补旧变量；未修改上游源码。
- 主机 Docker 和 NX ARM 均先在隔离目录构建测试，再构建正式目录。主机准备好的容器为 `aims-mocap-main-20261018`，使用 host 网络，工作空间挂载 `/work`，数据目录挂载到主机原绝对路径。
- NX 实际工具前缀为 `/home/aims/AIMSRacer/install/aims_mocap_validation`，桥前缀为 `/home/aims/AIMSRacer/install/vrpn_mocap`。VRPN 从新源码安装的独立 `install/vrpn` 链接，不再依赖旧临时二进制接入。
- NX 原 VRPN 安装保存在 `~/aimsracer-data/experiments/mocap-validation-20261018/install-vrpn_mocap-before/`。旧 Vicon 入口已保留兼容转发。
- 主机和 NX 新目录 **1333 个文件逐项哈希一致**，目录清单 SHA256：`b6dc4e44a550ebe659bfbaee6dc2dca60a2a2bb7fde87baf7b837720c1310a46`。两端 ELF 因架构不同分别记录哈希，不要求二进制相同。
- NX 部署前后核对 **620 个生产源文件和安装文件，变化为 0**。估计器、FastLIO、导航 EKF、MPCC 和驱动逻辑保持原样。

[源码一致性](assets/2026-10-18-mocap-validation/source-parity.json) · [主机源/二进制清单](assets/2026-10-18-mocap-validation/host-receipt.json) · [NX 源/二进制清单](assets/2026-10-18-mocap-validation/nx-receipt.json) · [生产文件核对](assets/2026-10-18-mocap-validation/nx-protected-runtime-after.json)

## 验证证据

| 环境 | 新工具测试 | 原 Vicon 兼容测试 | 正式安装接口再验证 |
|---|---:|---:|---:|
| 主机 x86_64 / ROS Humble Docker | 51 通过 | 19 通过 | 51 通过 |
| NX aarch64 / ROS Humble | 51 通过 | 19 通过 | 51 通过 |

测试包含解析三维运动、两参考点、偏置/已知波形偏移、时钟映射、遮挡/缺口、周期多解、固定/静态世界对齐、已知执行器事件、同时动作和渐变干扰、实际 CDR 提取、DDS 元数据、Ctrl-C 落盘、写入失败和损坏健康文件。分片包兼容测试仍通过。

本地实际 VRPN 服务模拟器经官方桥送入真实 ROS 消息：坐标正确，刻意设置为两秒前的服务器时间仍被保留，验证没有用节点当前时间覆盖它。这是客户端接入的合成测试，不证明现场 Vicon 相机的时间语义。

现有实车包 `2026-10-15/022604_194643_race/bag` 已只读提取：raw FastLIO 2229 条、后轴 LIO 2115 条、导航 EKF 44604 条、轮速 22403 条、最终命令 22405 条、VESC 反馈 22407 条。原包 SHA256 保存在提取元数据。该包没有 Vicon 和新五量输出，因此本项只证明实包提取，未声称独立精度通过。

[主机正式安装测试日志](assets/2026-10-18-mocap-validation/host-main-tests.log) · [NX 正式安装测试日志](assets/2026-10-18-mocap-validation/nx-main-tests.log) · [旧实包提取元数据](assets/2026-10-18-mocap-validation/real-bag-metadata.json)

## 报告与现场入口

[完整合成示例：总报告、三项子报告、曲线、CSV/JSON](assets/2026-10-18-mocap-validation/demo/report.zh-CN.md)。**该示例用于演示软件输出，不是实车精度结果。** 示例没有 S 弯，分层报告明确标为 missing；S 弯分类另有算法测试。

NX 已创建现场配置：`~/aimsracer-data/sessions/2026-10-19/mocap-validation/config.json`，未知项仍保留 `null`。

```bash
# NX：先填服务器、刚体/话题，再检查接入
cd ~/AIMSRacer
src/mocap_validation/run.sh inspect \
  --config ~/aimsracer-data/sessions/2026-10-19/mocap-validation/config.json
src/mocap_validation/run.sh record \
  --config ~/aimsracer-data/sessions/2026-10-19/mocap-validation/config.json \
  --output ~/aimsracer-data/sessions/2026-10-19/mocap-validation/run01
```

主机对应使用 `src/mocap_validation/host.sh`。提取和分析命令见使用说明。工具不会启动任何车辆动作。

现场还需填写/测量：Vicon 地址与刚体接入、VRPN 与相机采样及 NX 时钟的关系、标记到车体系/后轴/质心的外参、FastLIO 固定世界对齐或指定静态窗口。未完成时可以先录制，不能将未标定结果当成独立验收。响应偏移与消息时间年龄分别报告；当前没有实际舵角反馈，不能把整车转向响应单独解释为舵机机械延迟。本版没有设置未经讨论的实车合格线。
