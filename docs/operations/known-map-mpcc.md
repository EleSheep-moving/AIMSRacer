<a id="已知地图上的-native-mpcc"></a>
# Native MPCC on a known map

**English** · [简体中文](known-map-mpcc.zh-CN.md)

`race.launch.py` starts the vehicle, NDT, localization monitor, and native MPCC together for known-map operation. FAST-LIO2 provides continuous local motion, EKF provides rear axle state, and NDT anchors this local motion to the saved map. See [architecture](../architecture.md) for frame and timestamp conventions.

<a id="地图和-bundle"></a>
## Map and bundle

Current example map: `~/maps/20260928_010503/map.pcd`. Deployment target for the current field-run bundle: `~/aimsracer-data/bundles/field-v35-20261010`. It must contain the complete `input_reference/`, `input_config.yaml`, source manifest, native manifest, generated C, and target-platform shared libraries. Files are copied during deployment; example paths do not mean that a new machine already has these data.

The reference's `metadata.json` binds the original PCD SHA-256; the bundle binds reference, configuration, and generated source; the native manifest binds target platform and dependency libraries. After changing map contents, reference, or configuration, prepare a new bundle offline according to [MPCC usage](../../src/controller/docs/usage.md). Historical evidence linking the saved map and reference is in the [map reference report](../reports/2026-09-28-map-reference.md). For a new reference, use the two recovery scripts in [tools/reference](../../tools/reference/README.md), then run prepare/export/build.

<a id="启动与初始化"></a>
## Launch and initialization

First source the current workspace and acados libraries according to the [bringup guide](bringup.md), then run:

```bash
ros2 launch aims_racer_system race.launch.py \
  map_file:="$HOME/maps/20260928_010503/map.pcd" \
  artifact_directory:="$HOME/aimsracer-data/bundles/field-v35-20261010" \
  record:=true
```

The default is to wait for manual initialization and enable. The pose in the initialization command is always **base_link's x y z roll pitch yaw in map**, with angles in radians; base_link is the rear axle midpoint here. Replace the following values with the vehicle's actual placement:

```bash
ros2 run aims_racer_system relocalize_known_map.py \
  "$HOME/maps/20260928_010503/map.pcd" --pose-frame base_link \
  --x 0 --y 0 --z 0 --roll 0 --pitch 0 --yaw 0
```

Alternatively, set RViz's fixed frame to `map` and use 2D Pose Estimate to publish `/initialpose`, with the arrow indicating base_link's position and direction. Do not directly use Livox point cloud coordinates or the front of the vehicle as the rear axle initial pose. The initialization tool waits for a new trusted epoch and checks the actual map identity and new localization state; after initialization, check the vehicle's placement in the map.

If the initial pose is known at launch, pass six finite values directly:

```bash
ros2 launch aims_racer_system race.launch.py \
  map_file:="$HOME/maps/20260928_010503/map.pcd" \
  artifact_directory:="$HOME/aimsracer-data/bundles/field-v35-20261010" \
  initial_pose:="0 0 0 0 0 0" record:=true
```

Omitting `initial_pose` waits for later tool or RViz input; the example zero pose applies only when the actual vehicle is at that map position.

<a id="启用一次自动启动与停止"></a>
## Enable, one-time auto start, and stop

Enable manually when localization has `ready=true`, state and actual command history are fresh, the vehicle is stationary, RC has selected autonomous speed mode, there is a unique `/drive` publisher, and the starting heading is within 30° of the reference tangent:

```bash
ros2 service call /mpcc/enable std_srvs/srv/SetBool '{data: true}'
```

With `auto_start:=true`, the native node waits for the same enable conditions above and starts only once after they are met. Native `auto_start_timeout` defaults to 60 s; explicit stop during waiting cancels the request. Timeout/rejection reasons appear in `/mpcc/status`. Completion or a fault does not trigger automatic re-enabling, and a fatal process does not restart automatically. For another lap, the operator must explicitly enable or restart.

```bash
ros2 service call /mpcc/enable std_srvs/srv/SetBool '{data: false}'
```

The default `repeat_laps=false` stops after one lap counted from the actual enable position. `repeat_laps:=true` continues lapping until the operator explicitly stops; it does not change the number of auto starts. Field stopping relies on RC/emergency stop; close launch after the vehicle is stationary.

<a id="定位的资格与限制"></a>
## Localization eligibility and limits

NDT consumes `/fastlio2/body_cloud` and uses EKF TF prediction at the scan stamp; it owns `map → odom`. Its 50 Hz TF may hold the last correction, so TF timestamps cannot establish that new registration has occurred. The monitor derives health from protocol-v1 anchors, epoch, sequence, map SHA, and source/receive freshness; the native controller also checks the trusted alignment payload. `map_valid` alone or fresh TF is insufficient to authorize motion.

Currently `enforce_corridor=false`; the 0.5 m width on each side in the reference file is a course model, not a real boundary measured from the map; footprint OCP/startup boundary checks are disabled. Curvature planning ay=1 still participates in speed planning; disabling the combined acceleration ellipse does not imply unlimited lateral tire capability. See the [field review](../reports/2026-10-10-mpcc-field-review.md) for existing field runs and unverified conditions.
