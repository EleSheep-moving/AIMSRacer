# 轮速—IQ 离线标定与查询

本模块交付实测 Q 轴电流 `avg_iq` 与**区间平均轮速加速度**的正反向查询。
它不发布车辆命令；拟合、数据筛选与验收均不读取 IMU、EKF、LIO 或滑移估计。
既有采集节点与在线控制器不变。本次仅归档并恢复离线工具，不替换当前已选用的 B/C 仿射模型，也不自动向车辆下发电流。

## 环境与运行

在工作区根目录执行。查询模块仅依赖 Python 标准库；拟合使用 NumPy、SciPy
和 Matplotlib（NumPy ≥1.23.5，Matplotlib ≥3.5）。积分使用等价的 `scipy.integrate.trapezoid`，兼容 NumPy 1／2；SciPy 需提供 `scipy.optimize.isotonic_regression`（至少 1.12）。可用已有分析虚拟环境，也可安装包的 `wheel-model-fit` 可选依赖。
无需启动 ROS。安装此包后还会提供 `wheel-current-fit`、`wheel-current-query` 两个命令。

```bash
export PYTHONPATH="$PWD/src/calibration/longitudinal_current${PYTHONPATH:+:$PYTHONPATH}"
task_python=/absolute/path/to/analysis-venv/bin/python
task_source=/absolute/path/to/stage-bc-review

# 输出必须是新目录或空目录；拒绝覆盖 source/evidence、raw-extracts。
"$task_python" -m aims_calibration.wheel_current_cli fit \
  --source-root "$task_source" --output "$task_source/wheel-only-model-v1"

# IQ + 轮速 -> 平均轮速加速度，单位 m/s²
python3 -S -m aims_calibration.wheel_current_cli query \
  --model "$task_source/wheel-only-model-v1/model.json" --speed 3 --iq 80

# 期望平均轮速加速度 + 轮速 -> IQ，单位 A
python3 -S -m aims_calibration.wheel_current_cli query \
  --model "$task_source/wheel-only-model-v1/model.json" --speed 3 --acceleration 5
```

`-S` 仅用于演示查询无需第三方包；不用于拟合。运行目录已存在结果时改用新的
输出目录，不必删除或覆盖旧结果。输出 JSON 中 `valid=false` 表示不支持或输入无效，
命令退出码为 2；成功为 0。参数缺失由 argparse 给出用法和退出码 2。

## Python 接口

```python
from aims_calibration.wheel_current_model import WheelCurrentModel

model = WheelCurrentModel.load('/absolute/path/model.json')
prediction = model.predict_acceleration(iq_a=80.0, wheel_speed_mps=3.0)
inverse = model.current_for_acceleration(accel_mps2=5.0, wheel_speed_mps=3.0)
print(prediction.to_dict())
print(inverse.to_dict())
```

结果包含 `value`、`valid`、`reason`、`segment` 和可选 `domain`。
`segment` 区分 baseline、low、high。这里的反查值是**期望的实测平台 IQ**，
并不代表已经向 VESC 发送请求，也不保证请求电流与实测 IQ 完全一致。

- 零加速度返回对应轮速的 basic current。
- 平坦区反查返回最接近 basic current 的 IQ；若等距则取较小 IQ。
- 所有反查候选都正向复核，误差使用 `math.isclose(rel_tol=1e-9, abs_tol=1e-9)`。
  这是数值反解一致性容差，绝非标定精度。
- 数值、速度或电流超出支持区域时不外推、不静默限幅。

## 数据和模型

输入来自六个正式 B/C 录制组及冻结的 Stage A 三次基础电流曲线。读取原始
NPZ 中的 wheel、vesc、final、request 四个频道，核验文件 SHA256 和原生时间戳。
原包、原 trial 结果及旧研究分析均只读。初期探索包不混入主训练。

电流平台起点使用首次持续进入实测平台带，终点使用离开平台前的最后一个采样。
平台带容差为 `max(1 A, 3×1.4826×MAD, 10%×实测阶跃幅度)`，持续条件为
三个原生样本中至少两个满足。为抑制孤立噪声，退出搜索限定在最终模式切换前
20 ms 至后 250 ms；可能遗漏更早发生的持续电流衰减，这一限制保留在记录中。
反馈时间和遥测滤波边界未知，不能将这些窗口当作 VESC 内部纯物理延迟。

候选窗口至少需要三个有效原生轮速样本，不加 35 A 或 150 ms 排除门槛。
保留原失败原因、平台内比例、轮速斜率 R²、前后段斜率及单采样边界敏感性。
原失败试验仍可能贡献支持域内响应；全窗口和用于拟合的子窗口分别导出。

拟合限定前进轮速 1～5 m/s，首次越过零速之后的信号不再加入训练。
高电流 B 使用域内平台平均响应；低电流及 C 使用宽 1 m/s、中心间隔
0.5 m/s 的实际速度窗口，边缘按标定域裁剪。重复的相同样本区间去重，
每个试验的全部子窗口权重之和为 1。

### 单调查表的构建

正向 B 实测平均 IQ≥50 A 时汇合不同速度。对 5 A 电流组的加权均值做单调
最小二乘回归。50 A 共享锚点由 B 中 ≥40 A 的边界数据进行同样回归并在
50 A 处插值得到，随后约束高段加速度不低于此锚点。这个边界估计步骤在验证
每一折都重新拟合，未使用留出组的数据。边界数据必须从两侧夹住 50 A，
否则明确拒绝拟合，不用端点限幅制造锚点。分组均值代表该组的响应；首末组的
拟合值恒定延伸到该组的真实观测极值，支持域不越过原始观测的电流范围。

其余 B 和 C 数据按实际平均轮速归入最近的 0.5 m/s 档，输入坐标变换为：

`u = (IQ − IQ0(v)) / (50 − IQ0(v))`

在宽度 0.125 的 u 组上做加权单调回归，负 u 的加速度约束不大于零，正 u
约束在零至共享 50 A 锚点之间。节点 u=0、1 分别固定为零加速度及共享锚点。
速度行之间对相同 u 插值，保证 basic current 与 50 A 两个边界都连续。

支持域由各速度行记录到的负向电流范围、加速数据至共享 50 A 边界及高段实测
最大电流确定；速度行之间要求两行均支持。表格可以在观测节点之间插值，
但不会声称插值区域已逐点实测。高段速度支持边界按记录覆盖的 0.5 m/s 档
向外取整并限制在 Stage A 的 1～5 m/s 域内，具体范围随模型导出。
这种形状约束和分组会平滑异常响应，原观测和误差必须同时查看。

## 验证与交付

输出目录包含：

- `model.json`：版本、单位约定、曲线节点、支持域、来源和验证。
- `audit.csv/json`：全部 336 次正式尝试及未形成训练数据的原因。
- `training-intervals.csv`：真实样本区间、实际速度、IQ、加速度和归一权重。
- `heldout-predictions.csv`：留组预测、不支持原因及每条区间权重。
- `verification.json`：独立最小二乘复算、旧窗口边界核对、网格单调/反查检查。
- `README.md`、四张图、`artifact-sha256.json`：能力说明、曲线、覆盖率和产物哈希。

验证分别整组留出名义起始速度和请求电流档位，同一 trial 的窗口始终同行。
B 电流档位按相对冻结起始 basic current 的 5 A 增量分组；C 按请求电流
所在的 5 A 档分组。报告每区间加权的 RMSE、MAE、偏差与**测试覆盖率**，
没有为提高指标而只报告成功试验。缺少覆盖的测试输入不会被补零或计为准确。

第一版按实测能力交付，没有人为指定精度通过线。区间响应时长存在差异，
它尚不等价于瞬时动力学、在线前馈控制器或实车闭环验收。
