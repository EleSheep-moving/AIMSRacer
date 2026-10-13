# 2026-10-13 PP, longitudinal calibration and reference editor

[简体中文](2026-10-13-pp-calibration-reference-editor.zh-CN.md)

## Delivery and source

Implemented on `feat/pp-calibration-reference-editor`, based on field main
`098976b93ef14696aaf0bf229eaffc062b787b5c`. Runtime/profiling snapshot:
`e0261439a5eb1753d9715a5fa041578ba3c19d14`; later documentation commits retain that code.
The feature worktrees are `aimsracer-data/code/pp-calibration-reference-editor`
under `/home/elesheep` and `/home/aims`. Main remains unchanged, with no merge or
real motor execution in this delivery.

- Offline FastAPI/Three.js editor: immutable original PCD, XY rear-axle closed curve,
  Z filtering, 3D inspect, undo/redo, project persistence, preview, canonical numeric
  reference/speed export and hash-bound trial intervals. Shared quintic serialization
  matches native and PP readers. It does not certify free space.
- `aims_pp`: independent enhanced PP, old numeric steering parity, actual geometry,
  explicit enable, canonical reference and trusted health; no generated solver needed.
- `aims_calibration`: speed hold, positive-current intervals and negative-current
  deceleration; PP-core or RC steering; selected command echo, unique physical
  samples, actual `current_motor`, invalid-trial handling and bounded background logs.
- System launch chooses one race controller, or a separate calibration producer.
  Calibration alone disables the driver's startup-current substitution. Final
  ±100 A bounds and the tested MPCC/FAST-LIO/EKF/NDT parameters remain unchanged.
  Recording now includes raw `/calib/vesc_command`, final command and selected authority.
- English/Chinese operation guides, package guides and architecture are indexed.

## Verified software

NX rebuilt six feature packages against the unchanged main underlay. Source/install
checks matched seven PP/calibration Python modules and five production parameter
files. FAST-LIO submodule remains `afe5f4c5761d0fdf536e535df2a32017c50962a5`, VESC
`e3fa74c912f0f322b6aa8dc5218f5b8a662eb74c`; their tested main installations are reused.

- NX: **99 tests passed**: PP 32, calibration 58, new system-launch contracts 7,
  replay metrics 2. The calibration suite includes four installed producer → selector
  → VESC-driver PTY pipelines, signed wire currents and authority-loss handling.
- NX: **25 editor/reference/planner tests** and **4 portable native tests** passed.
- Desktop: complete selected ROS package regression passed; focused PP 32 and
  calibration 58 passed after review fixes; 19 system launch cases passed.
  Browser Chromium workflow passed, including delayed stale export response.
- Original real PCD: **990,482 points**, SHA
  `1db8c1905dc99ed4c0897838421118b998d15eb7cf57f50cee96a8b0744870dc`.
  GUI display is capped at 200,000 points. One NX API/PCD load took 2.12 s,
  during compilation; this is a smoke measurement, not a performance distribution.
- Review regressions cover first-current endpoint crossing, missing fresh telemetry
  at an endpoint, clock re-arm, first acknowledged-current sample bounds, log overflow
  and CSV failure invalidating authoritative JSON, and stale GUI download results.

## NX scheduling evidence

Raw Livox replay runs FAST-LIO, rear-axle adaptation, EKF and NDT simultaneously.
No real driver starts; virtual RC permission and recorded motor telemetry feed the
selector/calibration data flow. Replay lasts 60 wall-clock seconds after player start,
with a 5-second discovery delay, using the 2026-10-10 bag. That bag lasts 76.35 s;
wheel motion above 0.2 m/s occurs at offsets 15.66–37.09 s, with measured peak
2.88 m/s, so the replay covers its moving segment. Recorded wheel feedback is about
50 Hz rather than today's configured 100 Hz.

| Producer | Active samples | Callback ms: P50 / P95 / max | Output interval ms: P50 / P95 / max |
| --- | ---: | ---: | ---: |
| PP | 276 | 4.35 / 5.06 / 6.10 | 20.00 / 20.67 / 24.13 |
| Calibration / 标定 | 299 | 3.98 / 4.71 / 5.93 | 19.98 / 20.36 / 21.04 |

PP counts only `RUNNING` output callbacks, excluding sensor callbacks. Calibration
excludes IDLE/STOPPING/FAULT/COMPLETE. Durations use monotonic time; source eligibility
uses source/ROS stamps plus receipt freshness. P95 is the sample percentile, not a
worst-case guarantee. All sampled active callbacks were below 20 ms. Output interval
jitter above the nominal 20 ms is reported separately and is not a solver timeout.

**These are aggregate active segments, not continuous tracking acceptance.** Both
profiles used 40 explicitly requested virtual re-arms following faults. The untouched
source-time gates remain active. The TF auditor found 163/11,088 odom transforms with
negative signed source/receipt ROS-clock ages, minimum −5.23 ms. Monitor diagnostics
also recorded `invalid_ekf_stamp`; PP recorded stale/future odometry stops. This is
consistent with asynchronous replay-clock ordering, not proof of an actual physical future
measurement or a field latency fault. Continuous replay qualification remains limited
by this timing behavior. PP's first no-rearm post-fix run produced 59 active callbacks
before faulting; calibration produced 182. No automatic recovery was added to the
production nodes.

The first PP replay had zero active rows because rosbag's discovery phase clock
rollback cleared its one-shot static map identity. The fix retains only an already
confirmed matching static SHA while clearing dynamic health, map validity, pose,
authority and commands; explicit enable and fresh qualified data are still required.

Recorded motor responses came from the old commands. They cannot validate the new
current requests or constitute physical calibration. All-lap profiling regions are
synthetic scheduling fixtures, not field-approved straights. Real low-speed PP,
small-current trials and shutdown/RC takeover remain required before main merge.

## Reproduce and inspect


```bash
cd /home/aims/aimsracer-data/code/pp-calibration-reference-editor
source /opt/ros/humble/setup.bash
source /home/aims/AIMSRacer/install/setup.bash
source "$PWD/install/setup.bash"
python3 verification/calibration/replay_runtime.py \
  --bag /home/aims/aimsracer-data/sessions/2026-10-10/mpcc-acados/192955_v35_n15_accel1_cap4_lap01/bag \
  --map /home/aims/maps/20260928_010503/map.pcd \
  --seed /home/aims/aimsracer-data/sessions/2026-10-13/pp-calibration-editor/seed.json \
  --bundle /home/aims/aimsracer-data/bundles/field-v35-20261010 \
  --output /absolute/new-output --mode pp --seconds 60 --domain 223 --timing-rearms 40
```

For calibration use `--mode calibration --domain 224`. Outputs must be new directories.
`--timing-rearms` is an isolated test-helper option; its default is zero.

NX evidence: `/home/aims/aimsracer-data/sessions/2026-10-13/pp-calibration-editor`.
Host evidence: `/home/elesheep/aimsracer-data/sessions/2026-10-13/pp-calibration-editor`;
`nx-evidence/` is the copied NX output. Inspect `software-tests.xml`,
`source-install-provenance.json`, each profile's `summary.json`, controller CSVs,
localization events and TF audit. Generated evidence is local and is not included in
a fresh Git checkout. The [operation guide](../operations/pp-calibration.md) provides
editor, field PP and calibration startup commands.
