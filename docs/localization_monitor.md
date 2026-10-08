# 定位健康监控

`known_map_localization.launch.py` 默认启动 C++ 可执行文件
`aims_racer_system/localization_monitor`。节点名、话题和参数文件保持原有接口。
Python 版本保留为参考实现；默认启动流程不再使用它。

## 修改原因

2026-10-08 权重实验 A1 在定位健康检查中出现
`ekf_age_sec=0.110562809`、`ekf_receive_age_sec=0.003381803`，超过
100 ms 的源时间新鲜度门限。同期录包中的 EKF 仍连续发布且源时间新鲜，
因此证据指向监控端消息处理积压。现有数据不能进一步区分 Python 执行、
DDS 投递和操作系统调度各自的影响。

旧监控 EKF 订阅深度为 500；200 Hz 时可积压约 2.5 秒数据。
C++ 实现使用可靠订阅、深度 2。点云订阅使用传感器 QoS、深度 1。
用于点云时间对齐的 EKF 历史独立保留 5 秒，不依赖订阅队列。
匹配质量计算使用单个后台任务，主执行器不等待计算结果。
后台任务只读取点云、变换快照和地图，不修改健康状态。

## 保留的保护语义

- EKF 源时间和单调时钟接收时间均不得超过 100 ms。
- 点云两种时间不得超过 500 ms；可信锚点两种时间不得超过 1 秒。
- 重复源时间不能刷新接收看门狗；未来或倒退的输入时间无效。
- 锚点必须符合 epoch、event_sequence、anchor_sequence 和提交时间协议。
- 时钟倒退后必须进入新 epoch；失效后需要三次有效提交恢复。
- 健康序列随 epoch 重置；静态 TF 不会刷新可信锚点。
- 点云匹配质量仅用于诊断，不参与 ready 判定。
- MPCC 的故障锁存逻辑不变。

点云诊断仍使用源时间 EKF 插值和最近一次已提交的 map→odom 修正，
不插值修正、不外推 EKF。PCL 最近邻使用单精度坐标，诊断数值可能与
Python 双精度实现有微小差异；内点距离阈值仍为 0.25 m。

## 新增诊断字段

在 `/localization/status` 中：

| 字段 | 含义 |
|---|---|
| `implementation` | `cpp` 表示当前使用原生实现 |
| `ekf_callback_age_sec` | 最近有效 EKF 回调开始处理时的源时间年龄 |
| `ekf_max_callback_age_sec` | 进程启动以来上述年龄最大值 |
| `ekf_callback_count` | 有效、非重复 EKF 回调数量 |
| `ekf_stale_callback_count` | 上述回调中源时间年龄超过 EKF 门限的数量 |
| `quality_worker_busy` | 是否存在尚未收取结果的点云质量任务 |

`ekf_age_sec` 仍表示发布健康状态时最新已处理 EKF 的源时间年龄；
`ekf_receive_age_sec` 仍表示距离处理该消息已过去的单调时钟时间。
新增计数和最大值按进程生命周期累计，不随 epoch 重置。
回放工具记录原生可执行文件 SHA256，以标识实际运行版本。

启动命令和 `params/localization_monitor.yaml` 的保护门限无需调整。
编译成功只能说明构建完成，延迟改善幅度及实车效果需要单独验证。
