# 10 月 5 日晚至 10 月 6 日分析与修改交接

## 归档范围与版本

本页串联昨晚实车试验、后续离线调查和今天 NX 上的修改。归档于
`feat/fastlio-ndt-mpcc`，今天修改所用基线为
`0a5b9eddb0e6294f30decf4a6b4803fd11e2d631`。
下述历史报告中的“未修改配置”“进程仍运行”等句子描述各自记录时刻，
不能作为今天的配置或进程状态。完整聊天没有逐字复制；工程结论、方法、
失败记录、证据位置、改动和未解决问题均在这些报告中保留。

## 阅读顺序

| 阶段 | 报告 | 主要发现与证据边界 |
|---|---|---|
| 昨晚实车 | [场地原始报告](2026-10-05-field-analysis/field-report.md) | 1 m/s 首次尝试约 10.48 s 后 PlanExpired；0.5 m/s 完成一圈，但横向 P95 25 cm，仍有摆动 |
| 时间与预测 | [场地诊断](2026-10-05-field-analysis/diagnostics/README.md) | 区分消息年龄、相对响应滞后和真实运动延迟；检查实际指令、TF、预测前缀与原配置 EKF 回放 |
| 模型 | [模型核查](2026-10-05-field-analysis/diagnostics/model-report.md) | 等效转向偏置、约 −0.03 m/s 行驶横向分量、gyro 杆臂偏差、低速与纵向模型问题；没有独立运动真值 |
| 权重初审 | [原始权重核查](2026-10-05-field-analysis/diagnostics/weight-review.md) | 实际代价尺度，纠正旧记录中“提高/降低”的方向，列出后续单变量实验 |
| 今天 EKF | [纵向响应修正](2026-10-06-ekf-speed-response.md) | Q(vx) 0.025→0.4；安装配置同数据回放相对轮速滞后 125→45 ms、110→40 ms；更少平滑 |
| 今天权重 | [权重与摆动调查](2026-10-06-mpcc-weight-response.md) | 新增单变量短段与数值整圈实验；steering_acceleration_weight 0.3→1.2 是候选，尚未部署 |
| 求解可行性 | [迭代预算补充](2026-10-06-iteration-budget.md) | 离线扰动案例 30/60/100 次均过期；部分固定初值违反侧向加速度约束，更多迭代不能解决；不是实车失败请求的直接证明 |
| 今天实机 | [共享 gyro 零偏修正](2026-10-06-gyro-bias-correction.md) | 真实 MID360/VESC 静止采集；gyro z 均值 1.074→0.0054°/s，修正消息延迟 P95 0.83 ms；45 项回归通过 |

## 今天已应用到 NX 的修改

位置：`/home/aims/AIMSRacer-fastlio-ndt`，分支 `feat/fastlio-ndt-mpcc`。

1. `src/aims_racer_system/params/ekf_rear.yaml`：仅 Q(vx) 对角项改为 0.4。
2. `src/controller/config/vehicle.yaml`：生产 `solver_max_iterations` 30→50。
   权重实验使用的冻结配置仍是 30；迭代补充检查测试 30/60/100，不是生产 50 次的闭环验收。
3. 新增 `livox_gyro_bias`、参数、校准核心、测试和共享 launch 接线。
   `/livox/imu_bias_corrected` 同时供后轴 LIO 杆臂补偿与后轴 IMU；
   FAST-LIO 继续接收原始 `/livox/imu`。

MPCC 权重、机械转向零点、外参和 FAST-LIO 本次均未改变。
启动后需要静止标定约 10 s，并确认 `/imu/gyro_bias/status` 的 ready=true，
再初始化全局定位和控制。仅看到 EKF odom 不代表标定已经完成。

今天没有落地行驶或 MPCC 闭环验收。静止角速度均值降低不等于长期全局
姿态误差已经测得，也不证明行驶横向偏差或摆动已经解决。

## 数据保存与可复现范围

昨晚原始材料位于 NX：

`/home/aims/aimsracer-data/sessions/2026-10-05/225646-field-ndt`

首次自动尝试在已关闭、恢复 metadata 的 bag 中。0.5 m/s 完整圈没有完整
传感器 bag，只有控制器日志和独立健康观察；不能混用两次运行的证据。
活跃 SQLite 录包曾因数据库锁停止，原因和记录缺口保留在原始报告。

今天三个证据目录均位于 NX：

- `/home/aims/aimsracer-data/sessions/2026-10-06/ekf-vx-response`
- `/home/aims/aimsracer-data/sessions/2026-10-06/mpcc-weight-response`，含 `iteration-budget`
- `/home/aims/aimsracer-data/sessions/2026-10-06/gyro-bias-correction`

本地小型证据副本在 `/home/elesheep/AIMSRacer-fastlio-ndt/log/` 下的
`ekf-vx-response-20261006`、`mpcc-weight-response-20261006` 和
`gyro-bias-correction-20261006`。这些 log、原始 bag、NPZ 和求解器缓存
不会随 Git clone 获取。报告和主要图片已放入 docs，可独立阅读；
重新计算仍需要报告列出的外部数据、脚本和安装环境。

## 分支 docs 合并说明

初次归档核对的是本地 refs 和 NX 工作目录，当时尚未 fetch 或 push；下列
提交是发布前基线，不是本次发布后的 HEAD。初次归档时今天的代码与报告尚未
提交；本轮按用户要求分别提交并推送相关分支，实际提交见 Git 历史。新增报告已从
NX 同步到本地集成工作目录；本次只同步文档，没有把 NX 今天的源码补丁
复制到桌面源码树。

| 分支 | 核对的本地 HEAD | 文档职责 |
|---|---|---|
| `feat/aims-mpcc` | `ed2b14e` | FAST-LIO 工作线程、历史时序与 MPCC 基础文档 |
| `feat/wheel-imu-ndt-localization` | `2ef3ad1` | 轮速/IMU + NDT 的历史独立方案、NDT 耗时审查、Orin 静止报告 |
| `feat/fastlio-ndt-mpcc` | `0a5b9ed` + 未提交修改 | FAST-LIO + NDT 当前集成、固定地图目标审查、一秒 hold、今天的三项修改和分析 |
| `mpcc-sim` | `f2a85ed` | 仿真说明与 controller 同步；本地比所见 origin 多一个提交 |
| `main` | `8143cff` | 较早的平台文档；没有当前集成与今天的报告 |

合并时保留所有日期报告，不用新结论覆盖旧实验。独立轮速/IMU + NDT 方案
应标为历史方案；当前 FAST-LIO + NDT 启动说明作为当前运行入口。
`architecture.md`、`operations/bringup.md`、`operations/known-map-mpcc.md`、
`deployment/orin.md` 和 `reports/README.md` 的同名冲突需要按当前代码核对，
不能直接将任意一侧整文件覆盖过去。多个分支的报告索引需合并链接。

初次归档时 NX 的 `/home/aims/AIMSRacer`（`feat/aims-mpcc`）另有未提交的旧
docs 修订和其他源码改动。本轮发布将旧 docs 修订单独提交至 MPCC 分支，
保留其他源码改动。原 docs 差异已额外备份到本地
`log/docs-archive-20261006/nx-aims-mpcc-docs.patch`；未自动应用，以保留
两个工作目录之间的上下文。后续合并应包含本轮 MPCC 文档提交；这份
补丁保留作发布前快照，不应再重复应用。

## 尚未解决的事项

- 零偏修正后的行驶横向速度与真实轨迹误差。
- 新 EKF + 50 次迭代 + 零偏修正的落地闭环效果。
- 机械转向等效偏置、物理响应和低速速度反馈的独立辨识。
- 权重候选的同轨迹、同速度、重复实车对照。
- 失败求解请求完整初值、约束残差和代价分项的记录。
- 长时间温漂、轮滑、更高速度及多圈定位稳定性。

后续顺序：固定今天已应用的估计链路，先采完整低速数据验证状态与
求解可行性，再做单变量权重对照。本轮仅提交、推送相关分支，正式合并到 main 后续执行。
