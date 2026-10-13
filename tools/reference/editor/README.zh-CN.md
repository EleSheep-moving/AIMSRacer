# 离线参考路径编辑器

[English](README.md)

在原始 PCD 地图上绘制和编辑 **map 坐标系 XY 平面中的前进闭合后轴路径**。FastAPI 默认只监听 `127.0.0.1:8765`。Three.js **0.176.0** 和 OrbitControls 已随仓库保存，浏览器运行时不访问 CDN。工具不创建 ROS 节点，也不发布车辆指令。

## 安装与启动

主机 Python 可能没有 NumPy/SciPy，推荐独立 Python 3.10–3.12 虚拟环境：

```bash
cd /path/to/AIMSRacer
python3 -m venv /tmp/aims-reference-editor-venv
/tmp/aims-reference-editor-venv/bin/pip install -r tools/reference/editor/requirements.lock
export PYTHONPATH="$PWD/src/controller:$PWD:${PYTHONPATH:-}"
/tmp/aims-reference-editor-venv/bin/python -m tools.reference.editor.server \
  --config src/controller/config/vehicle.yaml \
  --map /absolute/path/to/original/map.pcd \
  --output /absolute/path/to/new-reference-bundles
```

打开 `http://127.0.0.1:8765`。也可不传 `--map`，在浏览器上传原始 PCD。`--reference /path/to/prepared-reference` 可打开已有 map 参考路径。输出目录放在 Git 仓库外；同名 bundle 已存在时拒绝覆盖。

在 NX 上启动服务后，本机建立 SSH 隧道：

```bash
ssh -N -L 8765:127.0.0.1:8765 USER@NX
```

然后访问本机同一地址。依赖安装需要联网，安装完成后的编辑只访问本地服务。`requirements.lock` 固定运行与测试的所有依赖版本，已在 Python 3.12 验证；目标架构需要相应 wheel。

也可使用 `aimsracer-mpcc:main-dev-20261011` 镜像，先安装 lock 中依赖，挂载仓库、地图和输出目录。Linux 容器用 `--network host`，服务保持默认 loopback。现有容器若把仓库挂在 `/repo`：

```bash
docker exec aims-pp-calibration-dev bash -c \
 'python3 -m pip install -r /repo/tools/reference/editor/requirements.lock'
docker exec -it aims-pp-calibration-dev bash -c \
 'cd /repo; export PYTHONPATH=/repo/src/controller:/repo; python3 -m tools.reference.editor.server --output /evidence/reference-bundles --map /data/sessions/2026-10-13/pp-calibration-editor/map.pcd'
```

CasADi 仅用于离线准备中的规范路径类符号接口；PP 读取导出的数值系数时不需要 CasADi 或优化器。

## 操作顺序

1. 加载**原始 PCD**。地图 SHA-256 对原始字节计算，原件保存在输出目录 `.maps/<sha>.pcd`，只读。显示使用 0.05 m 体素，最多 200,000 点，显示降采样不改变地图身份。Z 最小/最大值可过滤地面和相关结构。
2. XY 视图中选择 Add，点击至少六个不同的后轴点。Move 拖动点；Insert 在控制点连线上插入；Delete 删除点；Undo/Redo 恢复编辑。第一个点为起点。Move 选中点后用 “Selected → start” 修改起点；Reverse direction 保留起点并反转后续点。起点/方向改变会清除进度区间。滚轮缩放，中键或右键平移。“3D inspect” 允许旋转并禁用路径点编辑。X 红、Y 绿、Z 蓝；比例尺单位米。
3. 设置左右走廊宽度，验证并预览。使用与 native MPCC/PP 完全相同的 `PeriodicQuintic`，显示曲率、几何转角需求 `atan(wheelbase * curvature)` 和现有 `SpeedPlanner` 速度。图中每条曲线独立纵轴缩放，并标明最大值。配置中的欠转向系数也计入速度相关转角需求并接受转角限制检查。
4. Mark 模式点击曲线上两个位置标注试验区间，也可输入 `start_s/end_s`。`s` 是控制点连线累计长度的周期参数，**不等于精确几何弧长**。`start_s > end_s` 表示跨起点区间；起终点相同、重复 ID 和重叠区间（包括跨起点重叠）不合法；区间左闭右开 `[start_s,end_s)`，相邻边界允许。“straight” 是人工标注，使用前检查曲率。路径编辑会改变区间的物理位置，需重新核查。
5. 保存/重开 JSON project；project 不含地图，需要加载匹配原始 PCD。Load reference ZIP 支持 `path.csv`、`metadata.json` 及可选 `reference.json`、project、regions。地图与车辆几何必须匹配。已有冻结速度与当前配置不同时，默认拒绝导入；勾选 “Replan frozen speed using selected vehicle config”（启动时可用 `--reference` 配合 `--replan-speed`）明确重规划速度。project 和新参考 metadata 保存旧参考 SHA、旧速度曲线 SHA 和当前配置文件 SHA；证据与当前配置绑定。Export 创建新 bundle 并可下载 ZIP。

## 验证范围和文件

拒绝不足六个唯一有限点、重复或零长度线段、控制多边形自交、零切向量、周期接缝不连续、采样曲线自交、转角超限、非法走廊和区间。检查按不超过 0.02 m 的进度间隔采样，每个控制点至少 20 个样本；最多 1,000 控制点和 20,000 验证样本。检查是**采样检查**，不能数学证明每个样本间均无缺陷。PCD 用于显示；**不保证避障或车身净空**。

编辑器直接保存控制点，不把预览点重新拟合。导出与 native bundle 共用 `aims_mpcc.reference_export.serialize_reference(path, config)`。重开时校验 project、path 和已有冻结系数一致。

- `path.csv`：原始编辑控制点，列 `x,y`。
- `metadata.json`：schema 1、map、后轴、`source: gui`、`source_frame: base_link`、`closed_lap: true`、原始 `map_sha256`，以及精确配置的六个几何字段：`wheelbase/rear_offset/half_width/half_length/front_extent/rear_extent`。非对称前后尺寸时 `half_length` 为 null。GUI 不伪造录制时间戳。新录制路径标注 `source: recorded`；无 source 的旧文件仍受支持。
- `reference.json`：schema 1，节点、降幂系数 `coefficients[6][interval][2]`、周期长度、走廊、frame、metadata、`speed_profile: {schema_version: 1, positions, speeds}`。
- `project.json`：可编辑点、宽度、地图 hash、区间。
- `calibration_regions.json`：`{schema_version: 1, reference_sha256, regions: [{id, name, kind: "straight", start_s, end_s}]}`；hash 对 `reference.json` 的**精确文件字节**计算。

native 资格名改为 `reference_verified()`，旧 `recording_verified()` 保留为兼容接口。资格要求闭合及与配置完全匹配的几何；它不代表障碍检查或实车部署验收。已有场测文件不改写。

## 测试与许可证

```bash
export PYTHONPATH="$PWD/src/controller:$PWD:${PYTHONPATH:-}"
/tmp/aims-reference-editor-venv/bin/pytest -q -p no:cacheprovider \
  src/controller/tests/test_reference_export.py tools/reference/editor/tests/test_editor.py
/tmp/aims-reference-editor-venv/bin/playwright install chromium
/tmp/aims-reference-editor-venv/bin/pytest -q -p no:cacheprovider tools/reference/editor/tests/test_browser.py
```

浏览器测试实际执行绘制/编辑、保存重开、3D、导出重载，并核查请求仅在 loopback。设置 `EDITOR_REAL_PCD=/path/map.pcd` 、可选 `EDITOR_REAL_PROJECT=/path/project.json` 和 `EDITOR_SCREENSHOT=/path/editor.png` 可额外检查真实地图并保留截图。Playwright 只用于测试。

PCD 用 [MapIV/pypcd4](https://github.com/MapIV/pypcd4) 1.4.3（BSD-3-Clause）。[Three.js r176](https://github.com/mrdoob/three.js/tree/r176) 为 MIT，许可证与 vendored 文件 hash 在 `static/vendor/`。规范样条原作者署名仍保留在 `vendor/track.py` 和仓库 notices。
