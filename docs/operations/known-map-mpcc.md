# Known-map localization and MPCC reference

The controller can now use a prepared `map`-frame reference when the V2/V3 EKF
continues to publish `odom -> base_link` and the saved PGO map is relocalized.
No submodule source is modified. The current 100 mm rear-extent reference has
been prepared at `src/controller/recordings/20260928-mapping-lap/prepared_map_1m_rear10cm`.
The map localization path has **not** been validated on the moving car.
Keep MPCC in shadow until the checks below and the vehicle checklist pass.

## Prepare the saved course offline

The [2026-09-28 recovery report](../reports/2026-09-28-map-reference.md) identifies
the recorded loop and the map-frame candidate CSV. It is generated from the
saved PGO `poses.txt` and the matching recorder CSV. Its sidecar JSON contains
the exact `map.pcd` SHA-256 and spatial matching residuals. The generated CSV
is local-only under `src/controller/recordings/`.
The prepared reference bundle contains `path.csv` for the closed loop,
`metadata.json` for frame/map/vehicle geometry, and `raw.csv` for provenance;
it is not the point-cloud map itself.

The supplied vehicle model uses a 620 mm × 320 mm body: from the rear-axle
`base_link`, 0.52 m forward, 0.10 m backward and 0.16 m to each side. The
specified course is 1.0 m wide with the reference centered, so the configured
corridor is 0.5 m on each side. These are supplied geometry assumptions; the
saved point-cloud map does not independently establish the physical track edges.
Check clearances on the car and track before moving under MPCC control.

```bash
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 ros2 run aims_mpcc prepare_path \
  src/controller/recordings/20260928-mapping-lap/closed_map.csv \
  /data/reference-map-run1 --vehicle-config src/controller/config/vehicle.yaml \
  --left-width 0.5 --right-width 0.5 \
  --map-file /home/aims/maps/20260928_010503/map.pcd
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 ros2 run aims_mpcc prepare_solver \
  /data/reference-map-run1 --vehicle-config src/controller/config/vehicle.yaml
```

`prepare_path` rejects the candidate if the configured geometry/corridor or
curvature fails. It stores the map SHA-256 with the reference. Changing the
saved map requires preparing the path and native solver for that map again.
For this checkout, the generated bundle is
`src/controller/recordings/20260928-mapping-lap/prepared_map_1m_rear10cm`.
The stable local alias `src/controller/recordings/current` points to it; use
that alias for `path_directory` so launch commands do not change when a new
bundle is prepared. Switch the alias only after preparing the replacement.
The solver normally uses its shared ccache directory under
`${XDG_CACHE_HOME:-$HOME/.cache}/aims_mpcc/ccache`; it selects a matching
compiled object by content, not by the reference directory's name.
For this car, pass
`path_directory:=/home/aims/AIMSRacer/src/controller/recordings/current`
when launching MPCC. No cache environment variable is needed.
The candidate's sampled maximum curvature is about 0.93 m⁻¹: at 1.0 m/s,
the implied lateral acceleration is about 0.93 m/s², within the configured
1.0 m/s² limit. At 1.2 m/s it would be about 1.33 m/s². This is a model-based
speed choice, not a measured tire-grip limit.

## Bring up and verify one known map

Start **one** V2 or V3 driving launch. In a second terminal start only the
known-map add-on; it consumes the already running `/fastlio2/` topics and does
not launch another Livox driver, FAST-LIO, EKF or VESC:

```bash
ros2 launch aims_racer_system base_orin_livox_bringup_v2.launch.py
ros2 launch aims_racer_system known_map_localization.launch.py \
  map_file:=/home/aims/maps/20260928_010503/map.pcd
```

Use the separate terminals and source the same ROS/workspace environment in
both. The add-on sends upstream localizer TF to `/localizer/raw_tf`; a main-repo
gate publishes `map -> odom` on `/tf` only after the upstream validity service
reports successful initial ICP and the transform is recent. It publishes
`/localization/map_valid` and a latched `/localization/map_sha256`.

Give a **rough initial pose of the Livox frame in the saved map**. Zero is only
appropriate if the sensor is again close to the map's original origin and
orientation. The wrapper requires the exact map file used by the launch and
waits for actual ICP validity and the gate's fresh map-to-odom TF; the upstream
load service returning success alone is insufficient:

```bash
bash preprocess_script/relocalize.sh /home/aims/maps/20260928_010503/map.pcd \
  --x 0 --y 0 --z 0 --yaw 0
```

Verify `/localization/map_valid` is true, that `/localization/map_sha256`
matches the prepared reference metadata, and that `map -> odom -> base_link`
has one publisher per edge. Check the car's map pose against a known physical
location. Repeated starts at different positions and motion through the loop
are necessary before drive mode. Keep Nav2 and any other `/drive` publisher
stopped for an MPCC run.

The MPCC node keeps the EKF `odom/base_link` state and solver dynamics in
`odom`. It passes the fresh planar map-to-odom alignment to the solver for
comparing predicted motion with the fixed `map` reference. It refuses enable
when map validity or the map hash is missing/mismatched, and faults if map
status expires, TF becomes stale, or a correction exceeds 0.15 m at the car
or 0.5 rad during a run. `/mpcc/reference` is in `map` and `/mpcc/prediction`
is in `odom`; RViz overlays them through TF. The map add-on checks the upstream localizer's
**initial** success and TF freshness; upstream `relocalize_check` remains true
after a successful initial match even if a later ICP update fails. The gate
therefore does not prove continuous map matching quality. This limitation
requires live observation and an independent ongoing-quality signal before
unattended operation.
