# NDT target configuration audit and NX follow-up

## Correction and source

The initial integration enabled a 30 m local target, rebuilt after 5 m of XY
displacement. This missed explicit guidance in the pinned upstream Jetson
documentation and bypassed upstream's map-load search-tree warm-up. The source
reviews and earlier replay acceptance did not catch this configuration mistake.

Pinned upstream sources, checked against the unmodified dependency files:

- [Jetson guidance](https://github.com/rsasaki0109/lidar_localization_ros2/blob/5f795a6cd886a20ade4175cb70bde630ac9ec785/docs/mid360_legged_jetson.md#jetson-defaults): full target; crop disabled because dense target recrops stall output. Hardware validation is explicitly unperformed in that workspace.
- [Jetson preset](https://github.com/rsasaki0109/lidar_localization_ros2/blob/5f795a6cd886a20ade4175cb70bde630ac9ec785/param/mid360_legged.yaml): crop off, visualization downsample on.
- `src/component_lifecycle.cpp`: full-target branch calls `setInputTarget` then `warmUpRegistrationTarget`; crop branch defers target construction to scans.
- `src/component_alignment.cpp`: warm-up uses about 100 probe points to trigger PCL's lazy target search-tree build before activation.
- `include/lidar_localization/map_initialization_policy.hpp`: visualization filtering is separate from the registration target.

Configuration revision: `4c7448b`, branch `feat/fastlio-ndt-mpcc`. Dependency
commits and patches, FAST-LIO, EKF and admission thresholds are unchanged.

## Configuration and implementation audit

| Item | Evidence / decision |
| --- | --- |
| Online target crop | Corrected: `enable_local_map_crop: false`; inactive radius/distance overrides removed. Full raw PCD is the target throughout an activation. |
| Target warm-up | Restored upstream configure-time warm-up; no fabricated anchor is committed by the probe. Activation and three real scan confirmations remain required. |
| Map visualization | Corrected: `viz_downsample: true`, `viz_voxel_leaf_size: 0.5`. This reduces the transient-local display payload, without changing registration points or map hash. |
| Timing interpretation | Corrected old report language: `alignment_time_sec` is all of PCL `align()`, including lazy target-tree construction, not just optimization iterations. Full processing also includes target setup and fitness. |
| Scan / NDT / display scales | Retained scan leaf 0.2 m, NDT cells 1.0 m; display leaf 0.5 m is independent. No registration-map thinning was introduced. |
| Runtime map replacement | Existing trusted-PCD callback rejects `/map` replacement; restart with a new PCD rather than silently rebuilding a target under a live anchor. |
| Registration threads / search | Retained two explicit OpenMP threads; pinned OMP constructor uses `DIRECT7`. Upstream warning about multi-threaded `KDTREE` in separate G2 scoring does not apply to this registration path. No G2 supervisor is launched. |
| Callback blocking | Existing three-thread executor and separate timer/initial-pose/cloud groups; align and fitness release shared state lock. Fixed target avoids recurring crop construction under that lock. Scan filtering still holds state and TF gaps must be measured. |
| Double deskew / seed sources | Existing IMU, preintegration, deskew, twist, previous-delta and internal EKF/smoother disabled. Same-stamp rear-axle EKF TF plus the last trusted anchor is the only registration prediction. |
| Extra Hessian work | Explicitly pin `enable_registration_localizability_diagnostics: false` (already false by default). Independent scan-map consistency remains diagnostic. |
| Retries and duplicated gates | Trusted cloud path performs one alignment and one native admission decision; upstream recovery/seed gates are bypassed in that path. Rejected candidates cannot update the anchor or the next seed. |
| Trust vs TF timer | Timer only restamps the held transform; source age and monotonic health watchdogs still expire trust. 50 Hz TF is not 50 Hz registration. |
| Long-term path growth | Upstream legacy path appends without a bound, but the trusted cloud and timer branches return before legacy path accumulation/publication. It is not active in this graph. |
| Admission thresholds | Fitness 1.5, correction 0.5 m / full rotation 10 degrees and source age 0.5 s retained for the same-data comparison. These are provisional integration thresholds, not author-validated car limits. B's tilt/wheel/map discrepancies remain unresolved. |

The author Jetson preset is for legged robots consuming raw scans. Its IMU,
deskew, twist and range choices are not copied into this already-deskewed
FAST-LIO/EKF chain. Four or six registration threads also need a full-stack NX
comparison before changing the two-thread allocation.

## NX replay evidence

Complete 1x A/B replays ran on NX with fresh localhost domains 200 and 201.
Installed and source YAML SHA-256 both equal
`2bef6438dda9706df40c8d18875b7eaffdb4bd7cff7f6d79a7597db77c53be40`;
both summaries record that hash and the unchanged map SHA-256. Only parameters
and documentation changed, so the existing symlink installation needed no C++
rebuild. Both players completed and exact raw sensor delivery passed: A
478 LiDAR / 9,539 IMU; B 714 LiDAR / 14,285 IMU.

| Metric | A crop | A fixed target | B crop | B fixed target |
| --- | ---: | ---: | ---: | ---: |
| All `align()` P95 (ms) | 54.78 | 53.19 | 90.43 | 79.63 |
| All `align()` maximum (ms) | 569.98 | 82.72 | 621.94 | 114.02 |
| Full processing P95 (ms) | 61.87 | 60.04 | 217.61 | **321.01** |
| Full processing maximum, including startup (ms) | 686.01 | 89.17 | 734.93 | 414.14 |
| Full processing >500 ms after first anchor | 0 | 0 | 5 | 0 |
| Map/odom TF maximum receive gap (ms) | 23.77 | 25.74 | 125.43 | 24.33 |
| Native trusted commits | 435 | 441 | 278 | 308 |
| Independent inlier fraction median | 99.71% | 99.71% | 29.49% | 24.43% |

A passed all 12 checks; B passed 10 of 11, with continuous tracking still
failing. The B checks therefore establish containment/delivery/TF contracts,
not usable continuous localization. A scan-time cold spike is gone; target
warm-up took 0.542 s for A and 0.506 s for B, in configure before activation.
Neither run rejected a scan as stale. Map/odom and odom/base_link each retained
one publisher. B odom/base_link maximum receive gap was 14.07 ms.

These results support keeping the fixed target for this map: repeated half-second
spikes disappeared and TF timer stalls fell substantially. They **do not** show
that all processing got faster. Full-map scoring creates a remaining problem
in B's poor-overlap portion: the P95 of `processing - align` increased from
188.56 to 294.01 ms. This residual includes scan preparation, fitness and lock
handoff; it is not a separately measured fitness duration. Source inspection
identifies `getFitnessScore()` nearest-neighbor evaluation against the raw full
target as the leading hypothesis. It must be separately timed before attributing
all the residual to it.

Do not blindly apply the independent monitor's 0.25 m search bound to native
fitness. PCL's existing score averages squared nearest-point distances;
excluding distant scan points would change that metric and could make a bad
match appear acceptable. Any scoring optimization must preserve admission and
make exact scores versus rejection bounds explicit.

B's first rotation rejection remains a good-fitness result (0.0652), full
rotation correction 10.091 degrees but yaw only 1.410 degrees. Removing map
recrops does not remove the 3D scan versus planar EKF mismatch associated with
this handheld/tilted dataset. No threshold was loosened to force acceptance.

Observed total device RAM peak was 2,919 MiB, CPU temperatures 57.75--61.84 C;
these replay observations do not cover a live driver/control workload. These
replays exercise raw LiDAR/IMU/wheel input, FAST-LIO, EKF, NDT and monitor;
they do not launch actuator/control nodes. No pose-ground-truth or vehicle
multi-lap claim follows from timing or tracking-health acceptance. Owned replay
and tegrastats processes were checked stopped after collection.

Artifacts: `log/fastlio-ndt/A-NX-fixed-target`, `B-NX-fixed-target` and
`NX-fixed-target-tegrastats.log` on the isolated NX workspace. Historical
comparison is `A-NX-audited-final` / `B-NX-final`, with cropping enabled.
Local copies, including event/TF logs and summaries, are under
`log/fastlio-ndt/NX-evidence/`; `NX-fixed-target-comparison.json` contains the
computed metrics for all four runs.

## One-second anchor hold follow-up

At the user's request, revision `a951995` changes the monitor's configured and
default anchor hold age, and the controller's matching anchor age, to 1.0 s.
Native per-scan source-age admission remains 0.5 s; EKF/cloud freshness remain
0.1/0.5 s, controller health-message timeout 0.3 s, recovery three commits and
TF publication 50 Hz. Slower registration alone therefore does not relax sensor
health or permit a stale new correction. This change does not establish a
minimum registration throughput.

Desktop and NX each passed 33 related policy/ROS adapter tests. New cases prove
750 ms hold, expiry strictly beyond 1.0 s and expiry with a frozen source clock
despite fresh heartbeat messages. The NX system/controller overlays were refreshed
(7.21 s); the installed controller import reports the 1.0 s default, and replay
preflight checks installed monitor and YAML hashes against source.

`A-NX-anchor-1s-fault-isolated` (fresh localhost domain 203) passed all 12 checks
with complete 478 LiDAR / 9,539 IMU delivery. A 1.2 s NDT process pause revoked
readiness after 0.906 s from pause start: the last trusted source stamp was
already approximately 0.094 s old. Exactly three fresh commits restored ready.
The intentionally paused NDT TF gap was 1,206.76 ms; local odom/base_link remained
continuous with maximum gap 13.51 ms. FAST-LIO core P95 was 27.38 ms;
independent consistency median 99.78%.

The first attempt `A-NX-anchor-1s-fault` is retained as failed input delivery,
not passed acceptance: FAST-LIO received only 328 scans / 6,547 IMUs before
exiting on an IMU source-time gap. Trace comparison found a 196.42 ms receiver
gap containing 38 samples present in the source bag, whose maximum IMU source
gap is 18.74 ms. The transport/scheduling cause is not established. Input checks
and FAST-LIO gap limits were not relaxed. The full isolated rerun supersedes
this incomplete test for the one-second hold acceptance.

Both runs and the build/test logs are copied under `log/fastlio-ndt/NX-evidence`.
B has not been rerun with the 1.0 s hold; its prior continuous-tracking failure
remains an unresolved result. Extending hold cannot make a persistently rejected
map match pass its translation/rotation/fitness admission gates.
