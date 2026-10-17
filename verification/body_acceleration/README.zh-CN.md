# 车体加速度质量审核与因果重放

完整可运行命令、字段约定及未来静态/动捕接口见 [README.md](README.md)。质量报告见[中文报告](../../docs/reports/2026-10-15-body-acceleration-quality.zh-CN.md)。

工具只读 raw/corrected/rear IMU 和 rear LIO，**不读取 EKF 数组、不用轮速作为调参目标**。`extract.py` 对 ROS2 sqlite 使用 `mode=ro`，保留真实消息接收时间、源时间、完整 covariance，记录原包前后 size/mtime 和选定 CDR 的 SHA256。原始数组、完整事件和重放轨迹均放在外部数据目录；仓库只收录紧凑指标及图。

`quality.py` 必须写入不存在的新目录。优先传 `--exact-input` 使用 corrected IMU 的真实接收时间；否则明确标注 rear 接收时间代理。事件严格按接收时间排序，LIO 源时间保持不变；gyro bias 未就绪的开头不会补造 IMU。当前抽取没有保留跨话题 sqlite id，因此脚本遇到相同接收时间的跨话题事件会拒绝重放；当前证据无此冲突。

静态单一姿态只能描述短时波动，不能同时辨识加速度计 bias、scale 和倾角。Allan 输出列明 tau、cluster_samples、pairs；28 秒数据不拟合长期 bias instability。PSD 不跨超过 1.5 个中位采样周期的空隙，短于 512 点的片段不使用；报告注明有效支持点数，运行 PSD 仍混有运动、振动和噪声。

未来独立长静态数据通过 `--static-input` 导入；已知多姿态标定通过 `--calibration` 导入，逐轴检查可辨识性。未来动捕通过 `--reference-csv` 和 `--reference-metadata` 导入，必须明确仿射时钟映射、时钟不确定度、世界坐标变换、marker 到后轴的刚体变换及微分窗口。旋转 marker 杠杆臂在微分前处理；保留原/映射时间戳与边缘有效掩码。居中微分需要未来样本，只能作为独立离线参考，不进入在线观察器。当前没有动捕数据。

原生 CSV 的历史列名 `raw_ax` 指重力恢复及杠杆臂补偿后的**未滤波后轴惯性加速度**，不是原始 IMU 加速度或比力；`ax` 是滤波纵向估计，`force_x` 才是安装旋转/单位转换后的比力。图例据此明确区分。

`runtime_replay.py` 用独立 localhost ROS 域 193，以 1× 记录接收间隔重放指定时间窗，只启动观测器；共享时间偏移保持原定发布时刻的名义源年龄。实际调度和 DDS 另有抖动，不强制复现原包的 DDS 回调顺序。它保留原始输出、完整状态、调度偏差和源码/程序哈希。回调耗时是最多 512 次调用的滚动窗口 P99 最大/最终值，不是传感器延迟。初始化、回放和收尾阶段分别统计；此处使用录制的 LIO，没有并行运行定位全栈。命令见英文 README 的 “One-timespeed ROS receipt replay”。
