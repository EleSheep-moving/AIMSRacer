# 2026-10-14 滤波参数与 IMU 驱动预测比较

完整结论和方法见 [项目报告](../../2026-10-14-longitudinal-filter-tuning.zh-CN.md)。

推荐先使用现有 KF 的调参候选做下一次观测验证。全部三类方案已完成离线测试；默认配置和 NX 尚未改变。详细表、因果处理、保留验证、转弯退化、轮速/LIO 参考差异及代码复现均在报告中。

参数：jerk PSD 30、扰动 PSD 0.01、轮速方差 0.0025、比力方差 16。

文件：`training_grid.csv` 为 36 个 KF 设置，`inertial_training_grid.csv` 为 52 个惯性设置；`selected.json` 与 `inertial_selected.json` 记录主搜索冻结结果，`inertial_slow_selected.json` 记录补充慢轮速限制；`cross_run_metrics.csv`、`inertial_cross_run_metrics.csv`、`inertial_slow_metrics.csv` 为所有对照；`turning_checks.csv` 为转弯与全移动段指标；`baseline_recording_comparison.csv` 解释原录制与接收顺序回放的差异；`validation.json`/`inertial-validation.json` 记录核验范围。

`replayed` 保存每种方法的网格及原生输出，`provenance.json` 保存输入哈希。脚本只读历史输入，不启动或操作车辆。

本页列出的 CSV/JSON 已随附件目录入库，可通过[本组附件索引](README.zh-CN.md)读取；`replayed/` 完整数组与重放环境不在本次发布范围，来源路径仅作溯源说明。
