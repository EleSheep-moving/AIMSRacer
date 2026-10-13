# Offline reference editor

[简体中文](README.zh-CN.md)

Draw and edit a **forward closed rear-axle path in the map XY plane**, using the original PCD as context. FastAPI serves the editor on loopback; Three.js **0.176.0** and OrbitControls are vendored under `static/vendor/`. Runtime does not contact a CDN. The editor imports no ROS nodes and publishes no vehicle commands.

## Install and run

Use a dedicated Python 3.10–3.12 virtual environment. The host Python may lack NumPy/SciPy; activating ROS alone does not install the browser backend dependencies.

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

Open `http://127.0.0.1:8765`. `--map` is optional: upload a PCD through the browser. `--reference /path/to/prepared-reference` opens an existing map reference. Outputs are created under `--output`; an existing bundle name is rejected. Keep outputs outside the Git checkout.

For an NX running this server on loopback, establish a tunnel on your workstation:

```bash
ssh -N -L 8765:127.0.0.1:8765 USER@NX
```

Then open the same localhost URL. Python dependencies must already be installed on the NX. Initial package installation needs network access; browser use thereafter only needs the local server.

The existing `aimsracer-mpcc:main-dev-20261011` Docker image is another Python/ROS environment. Install the lock file in that image or a derived image and mount the checkout, original PCD, and output directory. Run with `--network host` on Linux and keep the server's default `--host 127.0.0.1`. Example for a prepared container named `aims-pp-calibration-dev` with checkout at `/repo`:

```bash
docker exec aims-pp-calibration-dev bash -c \
 'python3 -m pip install -r /repo/tools/reference/editor/requirements.lock'
docker exec -it aims-pp-calibration-dev bash -c \
 'cd /repo; export PYTHONPATH=/repo/src/controller:/repo; python3 -m tools.reference.editor.server --output /evidence/reference-bundles --map /data/sessions/2026-10-13/pp-calibration-editor/map.pcd'
```

`requirements.lock` pins runtime and test dependencies including transitive packages. It was verified with Python 3.12; install matching wheels on the target architecture before field use. CasADi is needed **only offline**, because the canonical path class has a symbolic interface. Loading the emitted numeric reference in PP does not require a solver or CasADi.

## Workflow

1. Load the **original** PCD. The SHA-256 comes from its original bytes. The service stores a read-only original copy in `--output/.maps/<sha>.pcd`; voxel filtering never changes map identity. Display uses 0.05 m voxels and at most 200,000 points. Adjust Z minimum/maximum to inspect the floor and relevant structures.
2. In XY view choose Add and click at least six distinct waypoints. Coordinates represent the rear axle directly. Move drags a point; Insert adds at a control chord; Delete removes a point. Undo/redo cover path, widths and intervals. The first point is the start; use Move to select a point and “Selected → start” to rotate the ordering. Reverse direction keeps the startpoint and reverses subsequent points. Start/direction changes clear progress intervals. Wheel zooms; middle or right button pans. “3D inspect” permits rotation and disables waypoint edits. Axes indicate X red, Y green, Z blue; the scale is in metres.
3. Set corridor widths and validate. The preview shows the **same canonical `PeriodicQuintic` geometry** exported to native MPCC and PP, plus signed curvature, geometric steering demand `atan(wheelbase * curvature)`, and the existing `SpeedPlanner` profile. The plot uses separate vertical scales and labels each maximum. The understeer coefficient also contributes to the displayed operating-speed steering demand and is checked against the steering limit.
4. Mark trial intervals by clicking two curve locations in Mark mode, or entering `start_s`/`end_s`. `s` is cumulative control-chord progress, **not exact geometric arc length**. `start_s > end_s` wraps over the seam. “Straight” is a user annotation; inspect curvature before using it for calibration. Edits may move interval locations; recheck the markers after editing.
5. Save/reopen the editable JSON project. A project file does not include the PCD: load the matching original map. Load reference ZIP accepts `path.csv`, `metadata.json` and optional canonical `reference.json`, project and regions. Map/vehicle geometry must match. If an imported frozen speed profile differs from the selected config, import rejects it by default. The explicit “Replan frozen speed using selected vehicle config” checkbox (or `--replan-speed` with `--reference`) recomputes it; the saved project and reference metadata record the prior reference SHA, prior speed-profile SHA, and selected config SHA. Replan evidence is tied to the selected config file. Export creates a new prepared bundle and offers a ZIP download.

## Validation and artifacts

Preparation rejects fewer than six unique finite points, duplicate/zero-length segments, control-polygon intersections, zero tangents, a discontinuous periodic seam, sampled curve intersections, excessive steering demand, invalid widths, and out-of-range trial progress. Curve checks sample at at most 0.02 m of progress and at least 20 samples per waypoint; limits are 1,000 control points and 20,000 validation samples. These are **sampled checks**, not a mathematical guarantee against every possible between-sample defect. The PCD is display context: there is **no obstacle or footprint-clearance guarantee**.

Preview does not resample and refit the control polygon. Export saves exactly those waypoints in `path.csv` and freezes the resulting coefficient table through `aims_mpcc.reference_export.serialize_reference(path, config)`. Native bundle export calls the same serializer. Reopening verifies project/path/frozen geometry consistency when those files are present.

Each bundle contains:

- `path.csv`: exact editor control points, `x,y` columns.
- `metadata.json`: schema 1, map frame, `coordinate_reference: rear_axle`, `source: gui`, `source_frame: base_link`, `closed_lap: true`, original `map_sha256`, and exact configured `vehicle_geometry` (`wheelbase`, `rear_offset`, `half_width`, `half_length`, `front_extent`, `rear_extent`). `half_length` can be null with asymmetric front/rear extents. GUI paths contain no recorded start/end timestamps. Newly prepared recordings use `source: recorded`; legacy files without this label remain supported.
- `reference.json`: schema 1, `knots`, descending-power `coefficients[6][interval][2]`, `length`, widths, frame, metadata, and `speed_profile: {schema_version: 1, positions, speeds}`. The length is the periodic knot coordinate.
- `project.json`: editable waypoints, widths, map hash, and interval annotations.
- `calibration_regions.json`: `{schema_version: 1, reference_sha256, regions: [{id, name, kind: "straight", start_s, end_s}]}`. The hash covers the **exact bytes** of `reference.json`. Start=end, duplicate IDs and overlapping intervals (including wrap) are rejected; adjacent boundaries use half-open intervals [start_s,end_s).

Native qualification is called `reference_verified()` and checks a closed path plus exact configured geometry, retaining `recording_verified()` as a compatibility accessor. Qualification says nothing about obstacles or deployment readiness. Existing field artifacts are unchanged.

## Tests and dependency attribution

```bash
export PYTHONPATH="$PWD/src/controller:$PWD:${PYTHONPATH:-}"
/tmp/aims-reference-editor-venv/bin/pytest -q -p no:cacheprovider \
  src/controller/tests/test_reference_export.py tools/reference/editor/tests/test_editor.py
/tmp/aims-reference-editor-venv/bin/playwright install chromium
/tmp/aims-reference-editor-venv/bin/pytest -q -p no:cacheprovider tools/reference/editor/tests/test_browser.py
```

The browser test drives actual drawing/editing, save/reopen, 3D view, export/reload and checks that runtime requests remain on loopback. Set `EDITOR_REAL_PCD=/path/map.pcd`, optionally `EDITOR_REAL_PROJECT=/path/project.json`, and `EDITOR_SCREENSHOT=/path/editor.png` to additionally load a real map and retain screenshots. Playwright is a test dependency; it is not used by the running editor.

PCD parsing uses [MapIV/pypcd4](https://github.com/MapIV/pypcd4), pinned to 1.4.3 (BSD-3-Clause). [Three.js](https://github.com/mrdoob/three.js/tree/r176) uses MIT; its license and exact vendored hashes are in `static/vendor/`. The canonical spline's upstream notice remains in `src/controller/aims_mpcc/vendor/track.py` and the repository notices.
