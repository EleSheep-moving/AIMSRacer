# 独立后轴 Vx 验证

[English](README.md) · [操作说明](../../docs/operations/velocity.zh-CN.md) · [质量报告](../../docs/reports/2026-10-15-vx-quality.zh-CN.md)

生产节点是 C++ `vx_observer`；Python 只作为离线数值参考。观测器 API 不接收轮速、EKF、LIO twist 或电流。轮速图只在冻结参数后生成。

| 工具 | 用途 |
|---|---|
| `extract.py` | 原始 SQLite 只读抽取、完整三维和协方差、源/接收时间及CDR/NPZ哈希 |
| `observer.py` / `test_observer.py` | 九状态参考，13项初始化/延迟/失效回归 |
| `synthetic.py` | 固定种子解析真值、九组候选、增益/相位/RMSE门槛与冻结 |
| `evaluate.py` | B/C/MPCC原始接收顺序回放，不加载轮速 |
| `native_acceptance.py` / `compare.py` | 合成及完整实包的离散状态精确、浮点≤1e−8一致性 |
| `ros_replay.py` | 完整1×传感器DDS回放、全部预期输出序列、RSS、耗时和调度迟到 |
| `sensitivity.py` | 冻结参数后的时间/杆臂扰动；差值不是独立精度 |
| `report.py` | 独立导出对比图片、频谱/创新统计及可选轮速/旧积分对照 |
| `resume_nx.sh` | NX恢复网络后在专用目录完成最终构建、全部测试、跨架构及完整ROS回放 |

Python3 需要 NumPy/SciPy/matplotlib/pytest。抽取和实际ROS回放另需 source ROS Humble 及标准消息；生产包依赖不变。

```bash
OPENBLAS_NUM_THREADS=1 python3 -m pytest -q verification/velocity/test_observer.py
OPENBLAS_NUM_THREADS=1 python3 verification/velocity/synthetic.py --output /new/external/synthetic
```

整个包新增 CTest `velocity_core_test`（11项）及 `velocity_runtime`（3项真正DDS/launch检查）；其余25项原有套件保留。原生 `velocity_trace` 是开发测试可执行文件。完整运行命令见英文页；所有输出目录要求新建，完整数据存储在仓库外。

NX恢复脚本使用 SSH alias `aims-nx` 和专用 `/home/aims/aimsracer-vx-observer-20261015`，不替换车辆主install。可设置 `VELOCITY_PYTHON` 指向带NumPy的Python，`VELOCITY_DATA_ROOT` 指向外部实验根目录：

```bash
VELOCITY_PYTHON=/path/to/scientific/python3 \
bash verification/velocity/resume_nx.sh /new/external/nx-results
```

2026-10-16已完整实跑该脚本：NX 27/27 CTest、三包双架构一致性及完整1×回放42255/42255输出均通过，证据见`accepted/nx`。`accepted/`指向质量报告最终有效运行；保留原始拒绝、停发和重初始化记录。软件、同源LIO一致性与Vicon真实速度精度分别验收。
