# 纵向加速度估计

[English](README.md)

这是隔离在 `feat/longitudinal-acc-observer` 分支上的**离线、因果比较**。输入是历史轮速、后轴 IMU 比力以及可选的 LIO 姿态，输出标量速度、加速度及比较证据。这里没有 ROS 发布器、启动文件或车辆驱动接口。

## 文件索引

| 文件 | 职责 |
|---|---|
| [observer.py](observer.py) | 卡尔曼观测器、整数源时间、不可修改的历史输出、延迟观测更新 |
| [replay.py](replay.py) | 校验输入哈希、按接收顺序回放、仅使用训练录包选参数 |
| [evaluate.py](evaluate.py) | 独立构造离线参考、指标、图表和已有结果图表重建 |
| [test_observer.py](test_observer.py) | 数值、时钟、历史、干扰项和合成真值测试 |
| [test_pipeline.py](test_pipeline.py) | 完整回放因果性、接收时间保持、参考缺口和起步指标回归测试 |
| [实施计划](../../docs/plans/2026-10-12-longitudinal-acc-observer.md) | 已确认的范围与任务清单 |
| [比较报告](../../docs/reports/2026-10-12-longitudinal-acc-observer.zh-CN.md) | 结果、解释与剩余验证 |

## 模型和时间关系

纯轮速状态为 `[v,a]`，融合状态为 `[v,a,d]`：

```text
dv/dt = a
da/dt = 白噪声 jerk
dd/dt = 随机游走
轮速 = v + 噪声
后轴纵向比力 = a + d + 噪声
可选的历史 LIO 纵向重力 ≈ d + 噪声
```

`a` 是车体纵向速度的导数，`d` 包含重力投影、加速度计零偏与 `(ω × v)x`。最后一项只是近似先验，不是完整干扰的精确观测。当前录包的 Livox/base 旋转外参为单位旋转；更换安装姿态后，需要先做明确的旋转换算。

每个观测保留整数源时间，实际接收到后才加入估计。晚到的观测插入有界历史，重放之后的状态；当前标量估计预测到 IMU 接收时刻，之前发出的估计不会被追溯修改。**没有预测 LIO 位姿或姿态，最新 IMU 输出不等待 LIO。**

历史最多保留 2 秒、4,096 条观测及一个后验锚点。`count_trimmed`、`too_old` 和数值失败计数可检查历史截断及观测接纳情况。输入中断时仍可查询估计，但协方差会增长。本实验没有添加加速度到电机的执行链，也没有更改生产 EKF。

## 复现

需要 Python 3.10、NumPy、SciPy、Matplotlib 和 pytest。主机使用已有镜像 `aimsracer-mpcc:main-dev-20261011`，镜像 ID 保存在报告证据里；NX 使用系统 Python。无需 ROS 环境。

```bash
python3 -m pytest -p no:cacheprovider verification/acceleration -q
python3 verification/acceleration/replay.py \
  --inputs /path/to/2026-10-11/longitudinal-acceleration \
  --forces /path/to/2026-10-11/imu-source-time-alignment \
  --output /path/to/new-evidence --jobs 4
```

输入加载器要求八次完成的场地录包、对应提取文件 SHA256、后轴比力回放哈希和源时间信息。保留 `RUNNING` 前 10 秒及 `COMPLETE` 后 3 秒。比力来自之前源时间适配器调查时冻结的生产核心回放，不用新估计的 LIO 姿态重新计算。

固定参数复现，不重新调参：

```bash
OPENBLAS_NUM_THREADS=1 python3 verification/acceleration/replay.py \
  --inputs /path/to/2026-10-11/longitudinal-acceleration \
  --forces /path/to/2026-10-11/imu-source-time-alignment \
  --output /path/to/nx-evidence \
  --selected-configs /path/to/selected-configs.json
python3 verification/acceleration/evaluate.py --evidence /path/to/nx-evidence
```

输出包括输入信息、元数据哈希、全部训练试验、冻结配置、逐包 `metrics.csv`、带时间估计和英文 16:9 PNG。输出时间为相对整数 `origin_ns` 的浮点秒，避免把 Unix 纳秒时间直接转为浮点。JSON 指标文件用 Python 的 `NaN` 表示不可用指标，CSV 同样保留缺失，不可悄悄填零。

## 证据边界

两次 V35 录包不参与调参。轮速加速度参考使用居中的 165 ms Savitzky–Golay 差分和零相位 3 Hz 滤波。未来轮速**只参与评分**，不会输入观测器；这些指标与轮速方案共享传感器，不能证明真实加速度精度或不存在轮胎打滑。起步时间差相对非因果参考，不是实测硬件延迟。LIO 与 MID360 IMU 也不独立。NX 耗时属于串行离线计算，不代表 FAST-LIO/EKF/NDT 联合负载或实车验收。
