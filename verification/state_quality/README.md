# 五量输出质量复核

入口报告：[2026-10-17 输出质量汇总](../../docs/reports/2026-10-17-state-output-quality.zh-CN.md)。

`audit_outputs.py` 只读归档数据，复算后轴 VX/AX/AY/yaw rate 的输出统计、共同源时间、加速度代数关系、yaw KF 矩阵递推、旧 EKF/LIO 速度对照和输出 PSD。独立 VY 与质心输出缺失时明确保留缺失，不新增运行时估计器，不拟合或调整滤波参数。

依赖 Python 3、NumPy、SciPy、Matplotlib。示例：

```bash
cd /home/elesheep/AIMSRacer
/home/elesheep/aimsracer-data/sessions/2026-10-13/longitudinal-acceleration-review/.venv/bin/python \
  verification/state_quality/audit_outputs.py \
  --data-root /home/elesheep/aimsracer-data \
  --output /home/elesheep/aimsracer-data/experiments/state-quality-20261017/review-new
```

输出目录必须不存在。`--repo` 默认为脚本所在仓库。输入路径以 `--data-root` 为根，要求存在：

- `experiments/vx-observer-20261015/accepted/` 中冻结 trace、指标、合成结果和 NX 验证记录。
- `experiments/planar-acceleration-20261015/quality-02/mpcc-trace.csv`。
- `experiments/yaw-rate-20261015/quality-02/mpcc-trace.csv`。
- `sessions/2026-10-15/022604-mpcc-review/critical-review/raw.npz`、元数据和 `model-consistency-20261015/velocity-input-review.json`。
- 仓库内各分支原报告附件和源码快照清单。

脚本核验原 trace/NPZ 哈希，保留 int64 纳秒，不以浮点读取绝对时间。静止与 RUNNING 使用报告中固定源时间窗口；标准差 ddof=0，P95 指绝对值、不扣静止均值。无效输出和间隔在图中断开；PSD 的两个比较信号使用共同支持，对已知无效输出的插值区间断开。PSD 规则、样本支持、哈希及软件版本可由报告和 JSON 检查。

生成 `summary.json`、`metrics.csv`、`five-output-overview.png`、`output-psd.png` 和 `output-psd.json`。其中记录的质心高度 ±1 cm 敏感性是输入几何扰动场景，使用现有三轴 omega/alpha，不是新 CG 输出或测量误差界。

本工具没有重新解码 db3、重跑 CTest/DDS 或验证在线 NX 安装。矩阵复算验证同模型同输入的算术一致性，不能代替独立动态精度验收。
