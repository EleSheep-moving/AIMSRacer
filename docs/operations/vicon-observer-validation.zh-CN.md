# Vicon：后轴／质心 VX、VY、AX、AY、yaw rate 验证采集

> 2026-10-18 整理范围：本页记录已部署工作区的用法；这次文档提交不包含对应的新运行时代码，命令须在已部署环境执行。当前后轴／质心五量以[统一状态指南](state-observer.zh-CN.md)为准。


[平面 G](planar-acceleration.zh-CN.md) · [Yaw rate](yaw-rate.zh-CN.md)

计划于 **2026-10-19** 使用 Vicon 检查估计器；用户确认接入方式尚未确定，已做成可配置。在线五量估计及发布为 C++，launch 与离线验证为 Python。MPCC 暂保留当前导航 VX，待估计器稳定性测好后再改输入。

已准备 五量对比工具与现场命令（记录中的工作区路径：`verification/vicon_state/README.zh-CN.md`）：`inspect` 检查接入、`record` 录包、Ctrl-C 停止、`extract` 提取、`analyze` 生成后轴／质心误差、覆盖率、时间年龄、窗口敏感性和波形偏移报告。Vicon 位姿类型、话题、时钟和标记刚体外参在 配置模板（记录中的工作区路径：`verification/vicon_state/config.template.json`） 中填写；未知项保持 null，不假设已标定。另有 `demo` 可先运行完整合成示例。软件测试不代表已经取得实车独立真值。

## 现场先核对三件事

1. **位置和姿态代表哪个刚体点。** 标记刚体原点不能直接当作后轴。记录刚体到 `base_link` 后轴的平移和旋转，确认车体 x 前、y 左、z 上；记录 Vicon 世界轴映射。若将来分析质心 G，另行测量后轴到质心，不能使用 Livox 安装距离替代。
2. **时间戳代表什么、属于哪个时钟。** 区分相机采样／帧时间、服务器数据就绪、本机接收和 ROS 发布。保存原始帧号、真实帧率、可用 timecode／硬件帧号及接收时刻；明确驱动 header 的生成规则。若只有接收时刻，仍完整保存，但不能据此宣称毫秒级响应精度。
3. **实时流是否经过预测、重定时或平滑。** 优先保留原始刚体位姿，同时记录所开处理选项、帧丢失和遮挡。预测后的显示位姿不能默认为独立采样参考。

Vicon DataStream SDK 提供帧号、帧率、timecode、延迟、全局平移和四元数等接口；SDK 全局平移使用 mm，导入本仓库前换算为 m。`GetLatencyTotal()` 的零值也可能表示没有可用延迟信息，不能当成真实零延迟。[官方 SDK 1.13 指南](https://help.vicon.com/download/attachments/726007858/Vicon%20DataStream%20SDK%20Developer%27s%20Guide.pdf)

Tracker 的系统延迟、SDK 报告和桥接后的 ROS 接收年龄有各自边界；软件版本和配置应随数据保留。[Tracker 4.4 官方指南](https://help.vicon.com/download/attachments/376309877/Vicon%20Tracker%20User%20Guide.pdf) SDK 另有为降低表观延迟而设计的重定时接口，现场需确认是否使用。[官方 DataStream SDK 介绍](https://www.vicon.com/software/datastream-sdk/)

## 保存什么

本车已收到的几何为轴距 0.365 m、IMU→质心 `[-0.09,-0.03,-0.03] m`（ROS 车体系），其中高度差 −3 cm 是用户估计、不确定度未知；见测量记录（记录中的工作区路径：`src/aims_racer_system/params/vehicle_geometry_measurements.yaml`）。采集时补测垂向距离，并确认提供的 IMU 点是内置芯片原点，而非光学原点或外壳中心；同时测动捕刚体到后轴／质心目标参考点的外参。标记刚体原点不是质心或后轴时，必须先换参考点再求导。

同时保留 Vicon 原始会话／标定、原始位姿流和车辆 ROS bag。车辆侧在既有启动方式上增加 `state_observer:=true`，其他驾驶配置沿用原流程。使用专用 `verification/vicon_state/run.py record` 时原启动的 `record:=false` 避免重复记录；专用 recorder 不启动车辆或估计器。录制以下话题：

- `/livox/imu`、`/livox/imu_bias_corrected`、`/rear_axle/imu`。
- `/fastlio2/lio_odom` 及两套 `/state_estimation/{rear_axle,center_of_mass}/{velocity,acceleration,yaw_rate,status}`。
- `/rear_axle/lio_odom`、`/state_estimation/planar_acceleration`、`/state_estimation/planar_acceleration/g` 及其 status/debug。
- `/state_estimation/yaw_rate`、`/state_estimation/yaw_acceleration`、`/state_estimation/yaw_rate/status`。
- `/tf_static`、时钟同步配置与时钟偏差记录；轮速、指令、控制状态用于定位事件。
- Vicon 位姿 topic（填写配置后专用 recorder 自动加入；原 vehicle/race 默认列表不猜未知话题）、帧号、遮挡／有效性、延迟元数据；专用 `extra_topics` 可保留未知消息类型的元数据。SDK 原单位、轴顺序和四元数顺序原样记录后再转换。

参数快照之外，测量并记下 IMU 到后轴、Vicon 刚体到后轴的外参；记录固定方式、车辆载荷、轮胎及采集前后是否变动。传感器温度能获取就保留，不伪造不可用信息。

## 推荐第一批动作

先记录启动与热身状态，再录前后各约一分钟静止；随后在动捕有效区域完成匀速直线、加速／制动、左右稳定圆弧和逐步加快的 S 形换向，各做重复段。重点覆盖从小到大的 yaw 变化率。保留两次试验之间的静止与恢复，不裁掉输入历史。

这里的动作用于建立覆盖，不预先指定车辆尚未验证的速度或极限。单独记下每段动作、方向及采集时段，尽量保留一组未用于调参的重复试验作最终评估。

## 真值如何形成

先把 Vicon 标记原点的三维轨迹换算到后轴，再离线求导；不能先对标记点加速度求导后忽略其旋转杆臂。用后轴姿态把世界加速度投影到车体系，与 `a_x/a_y` 比较；静止参考加速度应为零，不是 +1g。

位姿求导会放大噪声。参考导数的窗口／带宽、边缘无效段、遮挡和丢帧必须显式保留；用多个合理窗口检查峰值和相位敏感性。插值到 200 Hz 不等于 200 Hz 独立动捕测量。导数仅用于离线核验，不输入在线观测器，也不通过手动平移曲线掩盖时钟问题。

角速度参考应从相邻姿态的三维旋转增量形成车体 omega，再取 z；有 roll/pitch 时不能简单把欧拉 yaw 差分等同于 r。位置二阶导和旋转导数的有效带宽分别说明。

新的五量接口位于 `verification/vicon_state`；旧 `body_acceleration/quality.py` 导入器保留为历史单项分析。新归一化 CSV 为：

```text
source_ns,receive_ns,px,py,pz,qx,qy,qz,qw,valid
```

其中位置单位由配置明确为 m 或 mm，杆臂始终为 m，四元数为 xyzw 且单位长度。`valid` 保留真实跟踪有效性。`source_ns` 必须带有实际来源和时钟语义；若由帧号与帧率建立时间轴，保留原始字段、映射方法和不确定度。遮挡及异常帧分段，不能作为有效位姿混入连续求导；原始记录不要删除。

配套 JSON 使用整数纳秒时钟锚点／scale／uncertainty、`body_orientation_in_marker_xyzw`、两个 `origin_in_marker_m` 和求导窗口；精确格式见新工具说明（记录中的工作区路径：`verification/vicon_state/README.zh-CN.md`）。五量在车体系表达，不需要拟合 Vicon world 到 odom 的轨迹变换；车体安装旋转及目标点杆臂仍必须明确。时钟未测定前，不把 identity 当作已同步。主要误差不平移曲线，单独的去均值相关偏移诊断也不冒充物理延迟。
