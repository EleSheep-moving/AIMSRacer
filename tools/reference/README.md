<a id="离线参考恢复"></a>
# Offline reference recovery

**English** · [简体中文](README.zh-CN.md)

This directory retains two reference-recovery tools that only read original inputs and write new CSV/JSON files; native bundle generation scripts are in `src/aims_mpcc_rt/scripts/`. Vehicle operation uses the [race entry point](../../docs/operations/known-map-mpcc.md); offline steps are in [MPCC usage](../../src/controller/docs/usage.md#准备参考).

| Tool | Input and output |
| --- | --- |
| [recover_closed_lap.py](recover_closed_lap.py) | Recorder `odom/base_link` CSV; select inclusive rows for one lap, correct bounded closure drift, and output a candidate odom-frame CSV and JSON |
| [recover_pgo_map_lap.py](recover_pgo_map_lap.py) | Saved `poses.txt`, `map.pcd` in the same directory, actual rear-axle/Livox geometry and the recovered CSV for the same lap; select a PGO key-pose interval, and output a candidate `map/base_link` CSV and identity/matching JSON |

Inspect actual parameters, then choose a closed-loop interval from the original recordings:

```bash
python3 tools/reference/recover_closed_lap.py --help
python3 tools/reference/recover_pgo_map_lap.py --help
```


`recover_closed_lap`'s `--first-row/--last-row` and map recovery's `--first-pose/--last-pose` both include both endpoint indices. Closure-drift correction is not map registration and does not measure boundaries. Map recovery requires confirmation that CSV and PGO poses belong to the same lap, and uses the vehicle's mounting geometry. Merely renaming the frame of an odom CSV does not make it a map reference.

During mapping, use [save_map.sh](../save_map.sh) to request `/pgo/save_maps`:

```bash
bash tools/save_map.sh "$HOME/maps/my-new-map"
```


The tool requires a new map output; verify service success and the actual existence of `map.pcd` and `poses.txt`. Retain the complete map directory and do not overwrite a PCD already bound to a reference. After reference recovery, bind the map SHA using `prepare_path --map-file`, then export/build a new bundle; record the original CSV, intervals, geometry and map identity.
