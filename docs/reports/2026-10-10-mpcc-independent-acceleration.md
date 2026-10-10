# N15 / 1.2 m/s：移除组合加速度硬约束

2026-10-10，NX 本机。按操作员要求，当前试验使用独立纵向加减速限制和曲率速度规划，移除纵向加速度与横向转弯的组合椭圆硬约束。求解方法保持 SQP_RTI，最多两次 RTI；没有采用全 SQP 或线搜索。

## 日志结论与修改范围

原 N15 / dt=0.1 s / 1.2 m/s 隔离运行捕获的 101 个不可变请求中，78 个通过、23 个拒绝。23 个拒绝全部包含 `operating_envelope` 超限，其中 10 个还包含预测末端同类超限；独立速度、转角、转向速率和输入边界均未超限。长预测末端超限表示候选在未来预测时间上的组合加速度超过配置的椭圆边界，不代表当前车辆已经偏离轨迹。

新配置显式设置 `combined_accel_constraint_enabled: false`。旧 bundle 未记录该字段时仍按原规则运行，避免旧原生求解器与新配置静默不一致；新的 manifest 和 config 必须一致。关闭只支持当前严格 `rate_bounded_v2`。

移除覆盖：OCP 阶段和末端椭圆行、阶段零的椭圆加速度收紧、候选校验、接管时的椭圆修正、实际输出证书，以及过期计划/速度上限覆盖时的椭圆制动预算。诊断仍可显示椭圆利用率，但不据此拒绝或限幅。

保留：硬速度上限 1.5 m/s、纵向加速/制动各 0.5 m/s²、转角 0.45 rad、转向速率 2 rad/s、非有限值拒绝、原生状态码校验、定位与权限新鲜度、完整交付预算 50 ms。目标速度规划继续用 `lateral_accel_limit=1.0` 按曲率降速；该值在本轮不再构成实际候选横向加速度硬限制。权重、车辆模型与几何不变，corridor 停车仍关闭。

N=15、dt=0.1 s，预测时间 1.5 s；请求上限 20 Hz、输出 50 Hz；原始计划 TTL 按 horizon 自动得到 1.2 s。

## 与 NPU 的对应关系

核对 NPU 原始实车版本 `real-car-original`，commit `6c5012b9cd310c8fca5281c408298ffb5d4b3885` 的 [acados_solver.py](https://github.com/npu-ius-lab/Roboracer_China_2026/blob/6c5012b9cd310c8fca5281c408298ffb5d4b3885/controller/src/f1tenth_dynamic_mpcc/python/f1tenth_dynamic_mpcc/acados_solver.py#L362)：没有本项目的组合加速度椭圆硬约束。它有独立纵向加减速和转向速率硬限制，以及前后轮侧偏角软约束，并使用曲率速度规划。

本次对齐的是移除这项组合硬约束。没有移植 NPU 的动态轮胎模型、侧偏角软约束、物理限值或整个求解配置。

## 验证

| 检查 | 结果 |
|---|---|
| 原 101 个请求按原顺序回放 | 101/101 通过，剩余约束最大超限为 0 |
| 回放求解时间 | 中位数 0.615 ms，P95 0.850 ms，最大 1.065 ms |
| 独立滞后车辆模型闭环 | 658 周期通过；横向 RMS 0.00148 m，最大 0.00764 m |
| 闭环求解时间 | 中位数 0.501 ms，P95 0.700 ms，最大 1.114 ms |
| 隔离 ROS 整圈 | 约 31.89 s，COMPLETE；636 请求，0 求解失败、0 超时 |
| ROS 完整交付时间 | 中位数 1.102 ms，P95 1.956 ms，最大 5.311 ms |
| 整圈接管 | 631 激活；4 次收尾预测负速度被拒绝；1 次停稳取消 |
| Python 相关回归 | 29 项通过；含 OCP 行删除、corridor 行保留、独立边界和非有限值拒绝 |
| C++ 相关回归 | 新模式候选/执行/停车/速度覆盖，以及旧模式输出/执行/接管检查通过 |

隔离 ROS 测试使用 domain 194，所有驾驶与输入话题都 remap 到 `/mpcc_protocol`，没有连接实车执行器。它包含求解、交付、接管、输出和自动停车流程，但不是新配置的实车验收。停车附近的 4 次拒绝来自剩余的速度下限，车辆正常完成停车；没有再出现组合椭圆拒绝。

## 可追溯产物

实验根：`/home/aims/aimsracer-data/experiments/mpcc-acados-runtime/field-20261010`。

- 新原生 bundle：`bundle-v12-n15-independent-accel`。
- 配置：`vehicle-v12-independent-accel.yaml`。
- 原始请求和拒绝：`n15-repair/baseline-capture/controller/runtime.csv`、`request_snapshot.json`、`request_validation.json`。
- 新回放：`n15-repair/independent-accel-replay.json`。
- 新闭环：`n15-repair/independent-accel-core.log`。
- 隔离整圈：`n15-repair/independent-accel-runtime/report.json`、`controller/runtime.csv`。
- 测试输出：`n15-repair/independent-accel-python-tests.log`、`independent-accel-cpp-tests.log`。
- 曾研究的全 SQP 实验保存在 `n15-repair/bounded-sqp-*`；该方案未安装到实车，新版代码没有采用它。

## 部署状态

18:53 已安装新版原生节点，并将共享实验 env 切换到新 bundle。当前目录为 `/home/aims/aimsracer-data/sessions/2026-10-10/mpcc-acados/185351_v12_n15_independent_lap01`。实车链路复查通过：READY、worker_ready=true、enabled=0，N15 / dt=0.1 s，组合约束关闭，定位 ready 且 alignment 有效，`/drive` 唯一发布者，`/drive` 与 `/ackermann_cmd` 速度均为零。

新配置尚未进行实车行驶，也未开始本轮录包。基础链路、NDT 和 RViz 延续既有运行。节点旧二进制保存在 `n15-repair/mpcc_rt_node.before-independent-accel`；本次运行目录保留源码补丁、配置、哈希和 `disabled-check.json`。
