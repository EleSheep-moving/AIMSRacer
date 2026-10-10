# 车辆运行检查

本清单用于当前 native 实车流程。软件回归与实验验证放在 `verification/`；现场操作参考[启动](bringup.md)和[已知地图](known-map-mpcc.md)。

## 启动前

- 确认 RC/物理急停、供电、VESC 与 `/dev/ttyELRS`、`/dev/ttyVESC` 正常；核对 Livox 网络配置。
- 核对地图 PCD、参考和目标平台 bundle 的身份；从同一 workspace 的 install 环境启动。
- 确认 `base_link` 后轴位置、传感器安装、车体 footprint 与实际车辆相符。默认 footprint 为后轴前方 0.52 m、后方 0.10 m、左右各 0.16 m。
- 确认预期行驶区域有物理余量。当前 `enforce_corridor=false`，文件中的左右各 0.5 m 不提供真实边界保护。
- 只启动 vehicle、mapping、race 之一；已知地图 race 的 NDT 与建图 PGO 不共用 map TF 归属。

## 启用前

- RC 保持可立即接管；核对转向符号和 speed 模式单位。
- 使用实际 **map/base_link** 初值初始化，观察地图、点云和车辆位姿是否一致。
- 查看 `/localization/status` 中的 ready、map identity、epoch 和健康；查看 `/mpcc/status` 中 solver、状态新鲜度和自动启动原因。
- 确保只有一个 `/drive` 发布者，实际命令由 RC 选择器转发至 `/ackermann_cmd`。
- 车辆静止，方向在参考切线 30° 内。`auto_start=true` 会在条件满足后自主启用一次，操作员应在授权前完成现场检查。
- 需要保留数据时使用 `record=true` 并确认新会话目录可写、有可用空间。

## 行驶、停止与记录

- 观察横向误差、速度跟踪、定位健康、候选/接管拒绝与人工干预。已有 bag 响应可以继续分析；完整纵向辨识按后续加速度接口计划进行。
- 显式 `/mpcc/enable {data:false}` 会停车并取消未完成自动启动；紧急情况使用 RC/物理急停。
- 单圈默认自动停车；连续圈 `repeat_laps=true` 由操作员 stop。COMPLETE 等待与运动时间分别记录。
- 停稳并撤销权限后关闭 launch，检查 bag 元数据、`runtime.csv` 和配置快照。

默认配置是 2026-10-10 v35 实跑参数：巡航 3.5、硬上限 4、加速/制动 1、规划 ay=1（单位 m/s 或 m/s²），N15/dt=.1/RTI2。规划峰值约 2.99、实测约 2.89 m/s。实跑证据适用于该车、地图和记录条件，后续变化按同轨重复数据评估。[复盘](../reports/2026-10-10-mpcc-field-review.md)给出完整限制。
