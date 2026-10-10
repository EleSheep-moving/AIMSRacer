# 离线参考恢复

这里保留两个只读取原始输入、写入新 CSV/JSON 的参考恢复工具；生成 native bundle 的脚本在 `src/aims_mpcc_rt/scripts/`。车辆使用 [race 入口](../../docs/operations/known-map-mpcc.md)，离线步骤见 [MPCC 使用](../../src/controller/docs/usage.md#准备参考)。

| 工具 | 输入与输出 |
| --- | --- |
| [recover_closed_lap.py](recover_closed_lap.py) | recorder 的 `odom/base_link` CSV；选择含首尾的一圈 row，校正有限闭合漂移，输出 odom-frame 候选 CSV 与 JSON |
| [recover_pgo_map_lap.py](recover_pgo_map_lap.py) | 保存的 `poses.txt`、同目录 `map.pcd`、实际后轴/Livox geometry、已恢复的同圈 CSV；选择 PGO key-pose 区间，输出 `map/base_link` 候选 CSV 与身份/匹配 JSON |

查看真实参数后，依据原始记录选择闭环区间：

```bash
python3 tools/reference/recover_closed_lap.py --help
python3 tools/reference/recover_pgo_map_lap.py --help
```

`recover_closed_lap` 的 `--first-row/--last-row` 和地图恢复的 `--first-pose/--last-pose` 都包含两端索引。闭环漂移校正不等于地图配准，也不测量边界；地图恢复需要确认 CSV 与 PGO pose 是同一圈，并使用该车的 mounting geometry。不要将 odom CSV 仅改 frame 名就当作 map reference。

建图运行中，用 [save_map.sh](../save_map.sh)请求 `/pgo/save_maps`：

```bash
bash tools/save_map.sh "$HOME/maps/my-new-map"
```

工具要求新地图输出，核对服务成功并检查 `map.pcd` 与 `poses.txt` 实际存在。保留完整地图目录，不覆盖已经绑定参考的 PCD。参考恢复后，按 `prepare_path --map-file` 绑定 map SHA，再 export/build 新 bundle；记录原始 CSV、区间、geometry 和 map 身份。
