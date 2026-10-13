# PP 与标定节点的联合负载回放

[English](README.md)

`replay_runtime.py` 将已安装的 PP 或标定节点、命令选择器与原始 FAST-LIO、EKF、NDT 回放同时运行。它不启动 VESC 驱动，强制使用独立的本机 ROS 域，注入虚拟遥控权限，并按原始时间戳回放电机遥测。

编译功能分支并加载 `install/setup.bash` 后执行：

```bash
python3 verification/calibration/replay_runtime.py \
  --bag /绝对路径/已知地图录包 --map /绝对路径/map.pcd \
  --seed /绝对路径/初值.json --bundle /绝对路径/场地bundle \
  --output /绝对路径/新证据目录 --mode pp --seconds 60 --domain 223
```

用 `--mode calibration --domain 224` 检查稳速标定节点。输出目录必须尚不存在。初值沿用定位回放格式（`position`、`orientation`，表示 map 中的 base_link）。bundle 必须包含 `reference.json` 与对应的 `input_config.yaml`。本工作区需要 `build/aims_racer_system/tf_authority_audit`。

`summary.json` 分别统计有效输出回调耗时与发布间隔。P95 是样本的第 95 百分位，并非最坏情况上限。PP 传感器回调和标定闲置阶段不参与有效输出统计；若有效样本为零，不能据此评价运行性能。应同时查看 CSV、状态与定位证据。

脚本仅为调度测试在输出目录中创建全圈区间，**它不是实车认可的直线标定区间**。录包中的车辆响应来自原来的指令，因此不能验证新指令的物理响应、电流标定或实车安全性；不得把该合成参考用于实车。操作流程见[当前指南](../../docs/operations/pp-calibration.zh-CN.md)。
