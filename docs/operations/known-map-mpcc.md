# FAST-LIO2 + NDT known-map localization for MPCC

The driving graph uses FAST-LIO2 for local motion, rear axle conversion and the
200 Hz EKF for `/odometry/filtered` (`odom` / `base_link`). NDT consumes
`/fastlio2/body_cloud` (`livox_frame`), predicts from EKF TF at the scan stamp,
and owns `map` → `odom`. The existing static `base_link` → `livox_frame` mount
must describe the measured sensor extrinsic. NDT's 50 Hz TF publication holds
the last committed correction; those timer publications are not new anchors.

## Prepare the map reference


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

## Start and initialize localization

Build the pinned NDT dependency and AIMSRacer workspace using the dependency
bootstrap supplied with this branch, then source the resulting ROS 2 overlay:

```bash
source /opt/ros/humble/setup.bash
python3 src/aims_racer_system/replay/setup_ndt_dependencies.py log/fastlio-ndt/deps
MAKEFLAGS="-j2 -l2" colcon build --base-paths src log/fastlio-ndt/deps --executor sequential --symlink-install \
  --packages-up-to aims_racer_system aims_mpcc
source install/setup.bash
```

The bootstrap clones pinned dependencies and applies the recorded patches in
`log/fastlio-ndt/deps`. Build in a ROS 2 Humble environment with the dependency
build packages installed; source the resulting overlay in each terminal.
The standalone wheel/IMU EKF and deskew pipeline are not needed for this graph.
The commands below are repository launch commands. Execute all terminals in the
same ROS domain and with the same clock choice.

Start the existing vehicle graph (Livox, FAST-LIO2, VESC, EKF, static frames):

```bash
ros2 launch aims_racer_system base_orin_livox_bringup_v2.launch.py
```

Start the NDT add-on with an immutable saved PCD map. `map_file` is mandatory.
The helper configures and activates the NDT lifecycle node automatically.

The default `ndt_fastlio.yaml` uses the full immutable PCD as a fixed NDT
target. Online local-map cropping is disabled, following the pinned upstream
Jetson preset. Lifecycle configure constructs the target and runs upstream's
search-tree warm-up before activation; wait for `Registration target warm-up`
and `NDT configured and active` in the startup log. Initialization still needs
three fresh consistent scan matches. Map loading/warm-up is startup work and
must not be interpreted as a recurring scan-time cost.

The 0.5 m `viz_voxel_leaf_size` affects `/localizer/map_cloud` only. The
registration target and independent consistency monitor use the original PCD;
its SHA-256 does not change. Scan downsampling remains 0.2 m, NDT resolution
1.0 m, scan maximum range 30 m and registration thread count two. A much larger
map requires a separate memory/startup assessment before enabling a crop policy.

```bash
ros2 launch aims_racer_system known_map_localization.launch.py \
  map_file:=/absolute/path/to/map.pcd use_sim_time:=false
```

For replay, run the FAST-LIO2/EKF graph with simulated time and launch this add-on
with `use_sim_time:=true`; the bag must supply `/clock`. Do not run another
`map` → `odom` publisher in this graph. `map_tf_gate.py` belongs to the earlier
localizer workflow and is not launched by the NDT add-on.

Initialize with the pose of **base_link in map**. Angles are radians in
roll/pitch/yaw order. The explicit frame flag prevents interpreting a Livox pose
as a rear axle pose. Replace these values with a verified initial pose:

```bash
ros2 run aims_racer_system relocalize_known_map.py /absolute/path/to/map.pcd \
  --pose-frame base_link --x 1.0 --y 2.0 --z 0.0 \
  --roll 0.0 --pitch 0.0 --yaw 0.5 --timeout 60
```

For replay append `--ros-args -p use_sim_time:=true`. This command checks the
map SHA-256, NDT lifecycle active state, and an `/initialpose` subscriber before
publishing a single map/base_link pose. It returns success only after observing
a new localization epoch and a fresh, ready tracking status. Initial candidates
are validated by NDT; the first two consistent candidates do not commit a TF
anchor. Initialization has no absolute correction magnitude gate. Once tracking,
NDT's configured starting limits are fitness ≤ 1.5, translation correction ≤
0.5 m and full rotation correction ≤ 10°. These are initial tuning values.

Monitor the interfaces:

```bash
ros2 topic echo /localization/anchor_status
ros2 topic echo /localization/status
ros2 topic echo /localization/map_valid
ros2 topic echo /localization/map_sha256
```

The authoritative anchor diagnostic is `lidar_localization/anchor`, protocol
version 1. `epoch` changes on NDT restart or reinitialization.
`event_sequence` increases for every event, while `anchor_sequence` increases
only for committed corrections. `anchor_committed=true` is the only event that
refreshes trusted source time. Timer TF and diagnostic heartbeats cannot refresh
it. A rejected scan retains a fresh trusted anchor in `hold`; `ready` and
`map_valid` stay true during that short hold. After 0.5 s without a new trusted
anchor, either in scan source time or monotonic receive time, health is `lost`
and the controller must stop. Recovery requires three consecutive new committed
anchors. Duplicate or out of order events cannot advance this count. A new
epoch clears the old trust immediately. Clock rewind fails closed until NDT
establishes a new epoch. EKF must be fresh within 0.1 s and body cloud within
0.5 s, with monotonic watchdogs for both streams.

The monitor publishes `aims_racer_system/localization` at 10 Hz with protocol,
epoch, anchor sequence, state, ready, source age and source stamp.
`health_sequence` increases on every health publication, including watchdog
heartbeats without a new NDT event, and restarts at 1 on a new native epoch.
Consumers reject old or duplicate health sequences within an epoch; an earlier
ready heartbeat cannot overwrite a later loss heartbeat. The map SHA is
transient local. `map_valid` is the same health decision. The heartbeat uses a
steady timer so a paused simulated clock still expires the receive watchdog.

Independent scan/map consistency values are diagnostics. A worker interpolates
the EKF pose at the body cloud's source stamp, applies the static Livox mount and
the last committed discrete map/odom snapshot, then measures nearest map point
agreement. It never interpolates between map corrections and never changes
localization readiness. BLAS threads are scoped to one for this monitor process.
PCD diagnostics support ASCII and binary XYZ maps; compressed PCD must be
converted before launch. RViz receives NDT's `initial_map` as
`/localizer/map_cloud`, preserving the existing map display topic.

Acceptance before vehicle driving requires runtime evidence from the built
ROS image: verify the single TF owners, initial three-candidate commit behavior,
source and receive watchdog expiry, three-commit recovery, epoch change handling,
and the controller's resulting stop commands. Pure policy tests establish the
state-machine contract; they do not establish closed-loop or vehicle safety.

## Use the prepared reference with MPCC

Start MPCC with `path_directory` set to the prepared bundle and verify its
map hash matches `/localization/map_sha256`. The controller reads the structured
protocol version 1 health from `/localization/status` before interpreting a
map reference. A fresh `tracking` or `hold` status with `ready=true` permits
map-reference use; unavailable, malformed or stale status stops map-reference
control. Health received at monotonic time must remain fresh, so timer TF
renewal cannot mask a localization outage. Check `/mpcc/status` and the actual
applied command stream when exercising this gate.

The EKF state and solver dynamics remain in `odom`. MPCC uses the current
map/odom correction to compare motion against the fixed map reference.
`/mpcc/reference` is in `map`, and `/mpcc/prediction` is in `odom`; RViz overlays
them through TF. Open `src/aims_racer_system/rviz/mpcc_lio.rviz` with fixed frame
`map`, check that stationary surfaces align and that base_link is at the
verified physical pose, and ensure one publisher per dynamic TF edge. Keep
other `/drive` publishers stopped during an MPCC run. Evaluate with RC manual
selection until the localization fault checks and the vehicle checklist pass.
Historical ICP or stationary replay measurements from the earlier localizer
workflow do not validate this NDT branch.
