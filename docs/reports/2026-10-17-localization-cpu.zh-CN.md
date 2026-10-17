# 重定位持续 CPU 占用检查

> 历史诊断：这是调整 NDT 频率前的线程/等待策略对照。随后采用 5 Hz 的选择与部署见 [NDT 5 Hz 记录](2026-10-17-ndt-5hz.zh-CN.md)。本次文档整理不再次测量或更改参数。

本次检查针对正常行驶的持续 CPU 占用。检查开始时 NX 整车定位链未运行；保留现场参数，在私有 ROS 域中运行实际安装的 NDT 节点，用最近 MPCC bag 的既有 `/fastlio2/body_cloud` 与非 map 的 TF 做等速回放。没有启动 VESC、遥控接收、MPCC 或标定驾驶节点。

## 现场链与历史数据

实际 NDT 来自 `/home/aims/AIMSRacer/install/lidar_localization_ros2`；完整标定系统配置来自 `/home/aims/aimsracer-data/code/calibration-full-step/install/aims_racer_system`。NDT 配置是 NDT_OMP、2 线程、35 最大迭代、1 m 分辨率、0.2 m 扫描体素、输入队列深度 1。局部地图裁剪关闭，固定地图在激活时构建目标并预热搜索树。额外 Hessian localizability 诊断关闭。NDT_OMP 源码默认邻域为 DIRECT7。构建缓存记录为 Release。

最近 MPCC bag：`2026-10-15/022604_194643_race`。已保存的定位诊断中，整段扫描处理约 48.3 ms 平均、90.3 ms P95；配准平均约 41.1 ms、P95 83.1 ms。alignment_time 在 anchor 与 timing 消息中重复出现，因此这里的原始统计计数不等于独立帧数。独立质量监控的 quality_time 平均约 5.1 ms，状态消息可重复引用同一次质量结果。这些历史数值是墙钟耗时，不是 CPU 百分比。

## NX 隔离回放结果

原 bag 60–80 秒，200 帧点云、相同起始位姿和 TF，1 倍速；前 3 秒不计入稳态 CPU。CPU 按进程各线程 `/proc` utime/stime 的差分计算，100% 等于一个逻辑核。节点不重编译，候选只用子进程环境或参数覆盖，不改现场配置。

| 设置 | CPU / 一个核 | 配准平均 | 整帧 P95 |
|---|---:|---:|---:|
| 当前默认 2 线程 | 65.4% | 30.7 ms | 67.4 ms |
| 2 线程、OMP_WAIT_POLICY=PASSIVE | 65.9% | 31.3 ms | 68.6 ms |
| 1 线程 | 58.1% | 43.7 ms | 89.6 ms |

三个回放均有 198 个 committed 源时间戳（初始化需确认），共同时间戳的 map→odom 平移和旋转最大差为 0。该短窗口内未发现输出变化，但不是全工况精度或定位接受率证明。稳态测量有 168/168/169 个样本，来自测量窗口边界与接收时刻；不是不同的点云输入数量。

默认设置配准阶段约占整帧耗时 82.8%；其余约 6.4 ms 包括扫描准备、fitness 最近邻评分和其他管理开销，当前记录不能把它们进一步分开。被动等待未见收益；单线程约省 11.2% 进程 CPU，但整帧 P95 增加约 33%，不建议因此改掉当前 2 线程配置。没有复现整机多核满载，短隔离回放不能覆盖整车并发、可视化、雷达前端等共同负载。后续应优先细分配准的实际迭代次数、点数、fitness 耗时，再评估计算优化。

OpenMP 等待策略的官方语义：https://gcc.gnu.org/onlinedocs/libgomp/OMP_005fWAIT_005fPOLICY.html 。本次测量不支持将它作为现场 CPU 优化。

## 现场一条命令采样

NX 已准备被动采样脚本：

```bash
~/profile_localization_cpu.sh 60
```

它只读进程/线程 CPU、内存和 `/localization/status`、`/localization/anchor_status`、`/localization/ndt_status`，不发布驾驶或重定位指令，不启动整车。整车已启动、正常行驶时在另一终端运行。输出自动放在 `~/aimsracer-data/sessions/YYYY-MM-DD/localization-cpu-时间/`，包含 `cpu.csv`、`diagnostics.jsonl` 和 `metadata.json`。脚本已做语法检查及隔离域两秒空闲采样验证；尚待真正行驶时采样。

证据：本目录 `summary.json`、`paired-output-comparison.json`、`nx-evidence/*-result.json`、`*-trace.json`、`*-node.log`。本次未修改定位算法、参数文件或安装二进制。

## 仓库内证据

[汇总](assets/2026-10-17-localization-cpu/summary.json) · [配对输出](assets/2026-10-17-localization-cpu/paired-output-comparison.json) · [安装来源](assets/2026-10-17-localization-cpu/installed-provenance.json)

原始线程 trace、完整日志和采样脚本仍保存在 NX `aimsracer-data/sessions/2026-10-17/localization-cpu-review/`。
