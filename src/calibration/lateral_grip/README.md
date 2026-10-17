# Fixed nominal-steering lateral grip calibration

ROS package `aims_lateral_grip`, source folder `src/calibration/lateral_grip`. This is an independent persistent C++ calibration node, deterministic core and C++ operators. It records left then right, forward-only speed steps at fixed signed steering `atan(wheelbase / nominal_radius)`. The radius is nominal: slip and dynamics change the actual path. Complete combined-slip experiments are excluded. No planner, controller or `mu_safe` settings are updated.

## Configure, launch, inspect, start, stop

Use the same sourced ROS/install environment and ROS_DOMAIN_ID as the intended calibration stack. The normal selector, wheel odometry, telemetry and RC must already be operating. The launch starts no hardware and no production controller. Do not run longitudinal and lateral calibration nodes together: they share `/calib/vesc_command`; simultaneous publishers refuse/abort this session.

Copy `config/disabled.yaml` to a run-specific absolute path. Explicitly select `physical_configuration_selected: true`, measured wheelbase, nominal radius, strictly increasing positive speed steps, speed and steering limits, maximum motor current, minimum battery voltage and maximum motor/FET temperature. Set `source_revision` to the actual checkout revision plus dirty-state description and choose a durable `output_root`. Empty `output_root` resolves to `$HOME/aimsracer-data/sessions/lateral-grip`. Defaults are disabled and cannot start movement. There are no validated physical example limits; synthetic test values are software fixtures only. Review freshness, final-command latency and measured standstill thresholds for the actual stack before physical acceptance.

```bash
source /opt/ros/humble/setup.bash
source /absolute/workspace/install/setup.bash
ros2 launch aims_lateral_grip lateral_grip_calib.launch.py config:=/absolute/run.yaml
# Compatibility entry point: ros2 launch aims_racer_system lateral_grip_calib.launch.py config:=/absolute/run.yaml
ros2 run aims_lateral_grip inspect
ros2 run aims_lateral_grip start
ros2 run aims_lateral_grip stop
```

`start` sends explicit `/calib/lateral/enable` SetBool true. Nothing automatically starts. RC must be fresh, unlocked and in calibration mode, and `/control/calibration_enabled` must remain fresh and true. RC channel profile defaults to `steering_ch1_throttle_ch3_aux_ch5_to_ch10`; `sequential_ch1_ch2` is supported. Authority Bool and CRSF channels lack source stamps: their recorded time is explicitly receipt ROS time, not invented sensor acquisition time.

The driver parameter `/vesc_driver_node/current_startup_enabled` must be verified false through its GetParameters service, and the check remains fresh during the trial. This prevents a hidden startup-current replacement of speed requests. Missing/true/wrong-typed/stale parameter results refuse/abort. `inspect` prints configuration validity, driver readiness, control source/receive ages (0=wheel, 1=telemetry, 2=RC, 3=authority, 4=final command), authority, final matching and exclusive publisher state.

Before accepting start, require fresh `base_link` wheel velocity, healthy motor telemetry, an exclusive command publisher on each of `/calib/vesc_command` and `/vesc/command`, and selector final commands matching recent requests within `max_command_latency`. The node sends zero-current requests while idle to establish that selector feedback. Only the selector publishes `/vesc/command`; calibration publishes `/calib/vesc_command` with the typed VescCommand speed/current contract. No Ackermann jerk encoding exists.

Each target ramps at `ramp_rate` m/s². Fresh wheel speed must remain within `speed_tolerance` for `settle_seconds`, then satisfy `hold_seconds` and at least `minimum_hold_samples` distinct wheel source timestamps before advancing. Leaving the tolerance resets settling and hold. `stage_timeout` aborts an unreachable/incomplete stage. Only HOLD acceleration samples enter per-stage G-G statistics. At the end of left/right stages, request zero current and wait for continuously fresh measured wheel standstill for `stop_seconds`; left/right switching never relies on a timer alone. `stop_timeout` aborts if it cannot establish standstill. Zero current means **coast**, not braking or confirmed standstill. Mandatory control freshness, authority, frame, clock, telemetry-limit, actual wheel speed limit, exclusive publisher or selector-command failures latch abort. Stop and completion consume the single session; restart the process for another explicitly enabled session. `use_sim_time: true` refuses driving. Runtime parameter changes are rejected to keep recorded configuration fixed.

Checked metadata/evidence/summary write or flush failures latch abort and prevent further speed requests. Ctrl-C closes/flushed evidence and writes `summary.json` with process shutdown or earlier terminal reason. Selector authority/watchdog remains necessary when a process terminates; final shutdown is not a physical stop guarantee. This software has not qualified a vehicle or tire grip.

## Evidence and offline analysis

`inspect` prints the session directory, terminal state, warning quality and config SHA256. Each process gets a unique directory with:

- `metadata.json`: canonical complete control parameters/hash, executable path/hash and declared source revision.
- `evidence.csv`: every independent incoming sample's source timestamp, ROS/steady receive times, local state/direction/stage, numeric values, quality and raw detail; command requests and final selected commands are separate rows. RC channels and telemetry current/avg_iq/voltage/ERPM/temperatures/fault are retained. Observer covariance and diagnostic status are retained. Actual observer diagnostics are subscribed at `/state_estimation/{rear_axle,center_of_mass}/status`; optional raw `/livox/imu` and `/livox/imu_bias_corrected` are recorded independently.
- `radius_vxr_diagnostics.csv`: offline nearest-source-time Vx/r radius and Vx*r comparisons, bounded to 25 ms and retaining each channel timestamp; diagnostic estimates only.
- `summary.json`: periodically atomically replaced and finalized on shutdown; reason is retained after abort/completion.

All six observer streams are recorded under `/state_estimation/{rear_axle,center_of_mass}/{velocity,acceleration,yaw_rate}` using TwistWithCovarianceStamped, AccelWithCovarianceStamped and Imu respectively. Missing/stale/nonfinite observer evidence produces WARN and recording continues; observer quality is not an all-output publication gate. Control-input faults still abort.

```bash
ros2 run aims_lateral_grip analyze /absolute/session-directory
# Without matplotlib: ... analyze /absolute/session-directory --no-plot
```

Analysis writes `stage_stats.csv`, `analysis.json` and `gg.png`. AX and AY always come from the same acceleration message. State/stage labels reflect callback receipt time; no latest-value snapshot claims source alignment. The plotted G-G envelope reflects sampled maneuver/observer evidence, not independently validated tire forces or an exhaustive friction boundary. Any Vx*r comparison is a diagnostic proxy, not acceleration truth. Software tests and synthetic isolated ROS tests prove contracts only, not real-car tracking, observer accuracy, nominal-radius tracking or tire-grip safety.

## Reproduce software checks

```bash
colcon build --packages-select aims_lateral_grip --cmake-args -DCMAKE_BUILD_TYPE=Release
colcon test --packages-select aims_lateral_grip --event-handlers console_direct+
colcon test-result --verbose
# Pure core without ROS:
c++ -std=c++17 -Wall -Wextra -Werror -Iinclude tests/test_core.cpp -o /tmp/lateral_core
/tmp/lateral_core
python3 tests/test_analysis.py
```

CMake process tests force ROS_DOMAIN_ID=227 and ROS_LOCALHOST_ONLY=1, with fake selector/RC/wheel/telemetry only. They must never be run against a live vehicle graph or domain 0.
