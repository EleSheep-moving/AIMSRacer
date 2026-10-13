# PP, longitudinal calibration and map reference editor

Approved design: preserve the current field MPCC defaults, add an independent
enhanced Pure Pursuit controller, restore three longitudinal calibration stages,
and provide an offline browser editor over the immutable prior PCD map.

## Contracts

- `aims_pp` owns a ROS-independent enhanced steering algorithm, numeric reference
  loader and ROS adapter. `aims_calibration` imports that core; only its own node
  publishes `/calib/vesc_command`. Standalone PP publishes `/drive`.
- Editor saves `path.csv`, `metadata.json`, `reference.json` and editable project.
  Canonical quintic geometry and speed serialization are shared with the native
  exporter. Metadata identifies GUI provenance, original map SHA and current
  rear-axle vehicle geometry. No recorded timestamps are fabricated.
- GUI: local FastAPI + vendored Three.js, default loopback, SSH forwarding for NX,
  top-down XY editing, 3D view, Z filtering, undo/redo and trial-interval marking.
  Display-only 0.05 m voxel reduction capped at 200000 points. PCD stays immutable.
- Preserve old enhanced PP gains (lookahead 1/.3/4.5, heading .1, curvature .1),
  use actual vehicle geometry and limits. Default standalone speed cap 1 m/s.
- Stage names: `speed_hold`, `accel_interval`, `decel_current`; steering sources
  `pp`, `rc`. Signed current uses MODE_CURRENT; default disabled, explicit current
  lists required, 100 A driver/selector limits retained. Calibration disables
  40 A startup substitution only for its driver instance.
- `/control/calibration_enabled` is true only for an actual valid selected
  calibration command. Zero requests establish selection while idle; count
  samples/time only after explicit enable, fresh data, valid authority and an
  echoed final request. Loss aborts a trial and requires explicit enable.
- Services `/pp/enable`, `/calib/enable` use SetBool. Status topics are diagnostic
  arrays. No acceleration command controller, automatic racing-line optimization,
  horizontal grip calibration, motor execution or main merge in this delivery.

## Execution checklist

- [x] Baseline: separate worktree, verify main/NX/submodules, establish desktop test image.
- [x] Reference/editor: failing geometry/provenance tests, canonical export, GUI,
      PCD formats, persistence and browser behavior verification.
- [x] PP: failing old algorithm parity/reference/freshness tests, package/core/node,
      runtime and localization-health parity tests without generated solver.
- [x] Calibration: failing selected-authority/sample/timing/state tests, three-stage
      engine/node, RC profile, native commands, results and resume semantics.
- [x] Integration: selected-calibration authority topic, race controller selector,
      calibration launch, recording and parameter snapshot tests.
- [x] Review: spec compliance followed by code quality; fix concrete findings.
- [x] Desktop: compile ROS graph and run existing functional regression + new tests;
      browser smoke on real PCD, check export/native numerical geometry.
- [x] Docs: English default and Chinese paired guides/index/launch examples.
- [x] Deploy: commit/push feature branch, fast-forward same branch on NX, build,
      source/install checks, isolated recorded-data tests and timing evidence.
- [x] Deliver identical branch SHAs, clean working trees, reports and field commands.

## Acceptance

Existing field defaults and bundles remain unchanged. Geometry preview/export and
native/PP readers agree; periodic derivatives 0..4 are continuous. Bad maps,
self-intersections, degenerate geometry and impossible steering are rejected.
Actual current is `current_motor`, never a fallback to input/battery current.
Repeated telemetry is not counted as new data. Failed/time-limited or interrupted
trials are distinct from valid results. GUI never publishes vehicle commands.
Offline, virtual-serial and isolated ROS evidence is not real driving acceptance.

Use branch `feat/pp-calibration-reference-editor` on host and NX. Keep main intact
until on-site low-speed and small-current tests pass. Do not overwrite existing
maps, references, sessions, local patches or generated field bundles.

## Delivery evidence

The implementation, software/NX checks and synchronization are complete. See
[the dated report](../reports/2026-10-13-pp-calibration-reference-editor.md) for exact
metrics and the remaining asynchronous replay-clock qualification limit. Field
low-speed PP/current acceptance and main merge are separate pending work.
