# Closed-lap recovery and PGO map reference — 2026-09-28

Inputs are the local `/home/aims/aimsracer-data/sessions/2026-09-28/mapping-run1/mapping-lap-run1.csv` and
`/home/aims/maps/20260928_010503/{map.pcd,poses.txt,patches/}`. The map has
990,482 points and 375 saved PGO key poses. This mapping session began after
the calibration bag stopped; the bag has no contemporaneous `map -> odom`
history. Generated candidate CSVs live under the Git-ignored
`src/controller/recordings/20260928-mapping-lap/` directory.

The original 713-row CSV includes about 29 s of standstill before driving and
more standstill afterward. Its whole-file endpoints are 1.18 m and 31° apart.
Rows 303–593 (zero-based, inclusive; source stamps 1790528561.984291 to
1790528590.984268) contain one forward lap near 1 m/s. They are 33.54 m long,
with 0.143 m closure displacement and 2.28° heading difference. The
`recover_closed_lap.py` tool distributes this small displacement and yaw drift
by travelled arc length, then applies a cyclic 0.75-sample Gaussian smoothing.
Maximum additional smoothing displacement was 0.016 m. It writes
`closed_odom.csv` and a provenance JSON; its frame remains `odom`.

The saved optimized PGO poses 3–69 trace the same circuit. After converting
their Livox poses to rear-axle `base_link` using the repository mounting
geometry, their map-frame route is 33.45 m and nearly closed (0.184 m,
6.14°). `recover_pgo_map_lap.py` applies an arc-length closure correction,
matches the map route to the recovered odom CSV by travelled fraction, and
rejects a mismatch above 0.15 m RMS or 0.3 m maximum after the diagnostic
rigid fit. This run matched at **0.045 m RMS, 0.090 m maximum**. The resulting
`closed_map.csv` uses the optimized PGO geometry, not a guessed constant final
`map -> odom` transform; it records the exact `map.pcd` SHA-256 in its sidecar.

The current 620 mm × 320 mm body model puts the rear axle 100 mm ahead of the
tail. Its four footprint corners are at longitudinal offsets +0.52 m and
-0.10 m and lateral offsets ±0.16 m from `base_link`. The reference is modeled
as centered in a 1.0 m-wide course, with 0.5 m left/right corridors.

The local, Git-ignored `prepared_map_1m_rear10cm/` bundle has been regenerated
with this geometry. It is 33.470 m long, has `map` as its frame, and binds to
`map.pcd` SHA-256
`1db8c1905dc99ed4c0897838421118b998d15eb7cf57f50cee96a8b0744870dc`.
The earlier `prepared_map_1m/` bundle has an 80 mm rear extent and is superseded.
Its maximum spline displacement from the recovered polyline was 0.0149 m.
The native solver for the current 100 mm rear-extent bundle was compiled with
`prepare_solver` in 312.47 s. The compiled result was copied into the normal
`${XDG_CACHE_HOME:-$HOME/.cache}/aims_mpcc/ccache` cache; preparing again via
the stable `src/controller/recordings/current` alias used that cache in 3.48 s
without an `AIMS_MPCC_CACHE_DIR` override. The preparation command
performs its built-in warm-up solve, but no separate tests or performance
measurements were run for this revision. The earlier 80 mm variant took 304 s
to compile; its timing results do not describe the revised model.

The preparation checks consistency of the configured geometry and recorded
heading with a **modeled** 1.0 m corridor. The PCD does not certify the actual track edges,
the body's protrusions or live map relocalization; check those on the vehicle
before driving. The prepared reference has about 0.93 m⁻¹ maximum sampled curvature; the
initial 1.0 m/s cruise target corresponds to about 0.93 m/s² lateral
acceleration there, under the assumed 1.0 m/s² limit. This calculation is
not a measurement of available grip or free corridor.

Reproduce the candidates from the workspace root:

```bash
python3 src/controller/tools/recover_closed_lap.py /home/aims/aimsracer-data/sessions/2026-09-28/mapping-run1/mapping-lap-run1.csv \
  src/controller/recordings/20260928-mapping-lap/closed_odom.csv \
  --first-row 303 --last-row 593 --sigma 0.75
python3 src/controller/tools/recover_pgo_map_lap.py \
  /home/aims/maps/20260928_010503/poses.txt \
  src/aims_racer_system/params/rear_axle_geometry.yaml \
  src/controller/recordings/20260928-mapping-lap/closed_odom.csv \
  src/controller/recordings/20260928-mapping-lap/closed_map.csv \
  --first-pose 3 --last-pose 69
```

The tools refuse to overwrite existing outputs. The local generated files and
the original bag/map are not committed to the source repository.
