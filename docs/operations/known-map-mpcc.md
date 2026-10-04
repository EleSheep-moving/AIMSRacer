# Known-map localization and MPCC reference

The controller can now use a prepared `map`-frame reference when the V2/V3 EKF
continues to publish `odom -> base_link` and the saved PGO map is relocalized.
No submodule source is modified. The current 100 mm rear-extent reference has
been prepared at `src/controller/recordings/20260928-mapping-lap/prepared_map_1m_rear10cm`.
The map localization path has passed a short supervised moving-car start/stop
test; tracking a complete lap remains unvalidated.
Evaluate MPCC with the RC selector in manual until the checks below and the vehicle checklist pass.

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
publisher renews `map -> odom` on `/tf` at 50 Hz after verified initial ICP
and receipt of an upstream transform. Scan/map checks are diagnostic only. It publishes
`/localization/map_valid`, `/localization/status` diagnostics and a latched
`/localization/map_sha256`.

Open RViz before relocalizing, using the main-repository configuration:

```bash
rviz2 -d src/aims_racer_system/rviz/mpcc_lio.rviz
```

Its fixed frame is `map`. Gray points are the saved map, cyan points are the
current body scan placed through the public EKF TF tree, the yellow arrow is raw Livox odometry, and the blue arrow
is rear-axle EKF odometry. Point-cloud and raw LIO displays are disabled by
default to prioritize odometry; enable map/live-cloud displays only when checking
alignment, then disable them again. Consistency diagnostics receive identical-stamp
`/fastlio2/body_cloud` and `/fastlio2/lio_odom`, independently of RViz. They sample
at most 2000 points at 2 Hz and use raw LIO pose, not EKF TF. The optional green
world-scan display uses `/fastlio2/visualization/world_cloud`, which is disabled
in the LIO configuration by default and has no localization/control consumer.
The default top-down view centers on the prepared course; the saved
`Course overview` and `Vehicle detail` views restore the course view or follow
`base_link`. TopDownOrtho `Scale` is pixels per metre: a very small value makes
the entire course disappear into a few pixels. Adjust the overview center for
a different course. Enable the optional orange rear-axle LIO arrow to
compare estimates at the same physical origin. Before successful relocalization,
missing map transforms are expected; afterward check that fixed surfaces align
and the car's pose matches its physical location. The reference appears green
when MPCC starts, using transient-local durability so late RViz subscribers
receive it. The red prediction (1.0 s with the default `horizon:=10`)
starts at the scheduled future takeover state and appears after MPCC is enabled and an
online solve is accepted. Check `/mpcc/status` as well: RViz can retain the last
prediction after a fault. The same prediction display is available while RC selects manual or autonomous
control. Continue to use the relocalization script for the initial pose.

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
are necessary before selecting autonomous control. Keep Nav2 and any other `/drive` publisher
stopped for an MPCC run.

The MPCC node keeps the EKF `odom/base_link` state and solver dynamics in
`odom`. It projects the available map-to-odom alignment into x/y/yaw and passes it to the solver for
comparing predicted motion with the fixed `map` reference. Enabling requires the matching reference map hash and an available transform.
Initial localization verification belongs to the TF publisher. MPCC does not
subscribe to map-valid status, and TF age and correction size are diagnostic;
they do not stop MPCC or reject plans. A missing transform or changed map hash
still prevents interpreting the reference. `/mpcc/reference` is in `map` and
`/mpcc/prediction` is in `odom`; RViz overlays them through TF.
Upstream `relocalize_check` reports **initial** success and remains true even
if a later ICP update fails. Background scan/map matching records consistency
without another ICP optimization and without gating TF or vehicle control.
It does not expose native ICP fitness or per-update acceptance. Check physical
pose and fixed-surface alignment during supervised tests.

The two dynamic TF edges have different owners: the rear-axle EKF has
`world_frame: odom` and publishes `odom -> base_link`; the global localizer and
gate supply `map -> odom`. EKF prediction to current time does not refresh the
global-localization edge.

A 30-second stationary observation on 2026-10-03, with MPCC stopped and
V2/localization/RViz running, found 29 raw-localizer TF publishing pauses at
approximately one-second intervals. Pause P50/P95/max were 210/233/268 ms;
every pause contained new LIO arrivals. The upstream node executes input
filtering and coarse/refine ICP synchronously in its timer callback, with
`rclcpp::spin` using a single-threaded executor. TF broadcasting, synchronized
input handling and validity-service callbacks wait for that callback to finish.
Map-cloud conversion/publication and callback catch-up can add delay afterward.
This observation locates the recurring stalls in that callback; it does not
measure ICP-internal time separately from other callback work. No submodule
code was changed.

In the same observation, LIO arrival source-age P95 was 65 ms, while observed
raw-to-gated TF forwarding lag P95 was about 2.2 ms. Sampling the latest held TF
every 20 ms gave map-to-odom age P95/max 302/382 ms, versus EKF odom-to-base-link
age 4.3/24.5 ms. At the map-TF peak, latest LIO age was 81 ms and EKF TF age
3.5 ms. A representative pause began with an already 145 ms-old TF; 210 ms of
localizer silence made the next broadcast 355 ms old, so the gate rejected it.
The previous accepted TF continued aging until input handling and broadcasts
resumed. One separate approximately 297 ms LIO-arrival gap also occurred, but
does not explain the recurring localizer pauses or that peak example.

The main-repository publisher renews the latest upstream correction at 50 Hz
with the current timestamp after initial alignment is verified. When ICP or
input callbacks pause, it continues publishing that correction. An explicit
initial-alignment reset clears it until initialization succeeds again. Service
errors leave the last verified alignment intact. Original source ages remain
visible in diagnostics; they are not rejection thresholds.

A single background worker checks at most 2,000 scan points at 2 Hz and reports
the fraction within 0.25 m of the saved map, inlier RMSE and check time. These
values, missing clouds and check failures neither stop TF publication nor
change control authority. Upstream corrections do not wait for this auxiliary
check. Parameters in `src/aims_racer_system/params/map_tf_gate.yaml` now control
only diagnostic sampling; the old freshness flags and match-score limits have
been deleted.

MPCC does not interpret localization health. It checks the reference map
identity and availability of the transform needed for its map reference, with
no map-valid subscription, map-TF age gate or correction-size gate.
EKF measurement freshness (100 ms), actual applied-command history, RC status
and real plan expiry (`horizon * 0.8 * 0.1 s`, 800 ms for horizon 10 by default)
remain enforced. Future bridge simulation cannot
turn a predicted expiry into an immediate stop. Handover prediction checks remain part of MPCC: a candidate must match its
expected takeover state and actual input boundary. Their thresholds are
30 cm position, 30 degrees yaw, 0.30 m/s speed and 20 degrees steering;
actual target differences use the same 0.30 m/s and 20 degree limits. Map
roll/pitch tilt and measured speed operating range have no refusal gate. Reply budget and iteration
limits remain 250 ms and 30 iterations at the current 5 Hz solve frequency.

The saved-map reader accepts ASCII or binary PCD, including the current mapping
output; binary-compressed PCD requires conversion before use.

Historical verification of the earlier blocking policy on 2026-10-03 used an isolated ROS domain and the outdoor
`mpcc-shadow-selfcheck-20261003-004259` bag (offset 600–650 s). The bag has no
recorded world-cloud topic, so 500 stationary scans were reconstructed from raw
Livox points, internal extrinsics and contemporaneous recorded LIO poses;
motion deskew was not reproduced. Speed stayed below 0.007 m/s. Scan/map inlier
fraction was at least 99.2%, and inlier RMSE P95 was 0.040 m. Initial verification
was supplied by a replay fixture for the already verified recorded alignment;
this did not test a fresh initial ICP or the upstream localizer end to end.

During an uninterrupted replay interval, held map-TF age sampled at 200 Hz had
P95/max 20.1/21.5 ms, while raw TF source-age P95 was about 320 ms; localization
stayed valid. A deliberately interrupted raw TF stream invalidated localization
after about 0.7 s; a 5 m scan offset failed the next consistency check, and
stopping point-cloud input invalidated it after about 0.5 s. Renewal stopped
and recovered with valid fresh inputs. Sixteen targeted gate-policy checks and
ten related coordinate/reference tests passed; the system package was rebuilt.
Actual RViz rendering showed the saved map, current points and closed reference.

The former hypothetical command-history inconsistency was reproduced and
corrected: only actual `/ackermann_cmd` messages now enter prediction history,
and manual RC selection permits explicitly enabled MPCC calculation. See the
[controller implementation](../../src/controller/docs/implementation.md) for
the numerical explanation and matched-input timing measurements. TF/RViz
verification is complete for this stationary replay; moving-car validation
remains open. Replay tests run in an isolated ROS domain without hardware drivers.

After removing the separate evaluation mode, a 52 s wall-clock-rebased replay
of the same stationary segment enabled MPCC in manual at 7 s and disabled it at
45 s. All 380 predictions were accepted, selector status remained false, and
there were no faults or deadline misses. Sampled solve P50/P95/max were
58.6/62.2/64.7 ms with horizon 10, one thread, RViz and the map gate running.
Localization stayed valid during the active interval. This verifies stationary
manual calculation and real forwarded-command history; it does not establish
moving-car tracking. A separate hardware-free check runs the real RC selector
to verify manual override, navigation routing and timeout behavior.
