# Independent enhanced pure pursuit

[简体中文](README.zh-CN.md)

`aims_pp` runs the enhanced PP equations recovered from the calibration script
as a standalone controller. It needs NumPy, PyYAML and ROS 2 Humble. Reference
loading and control do not import CasADi, acados, SciPy or `aims_mpcc`.

The controller uses the real vehicle YAML geometry: the default field profile
has a 0.36 m wheelbase and 0.45 rad steering limit. Its recovered tuning is
`lookahead_gain=1`, `min_lookahead=0.3`, `max_lookahead=4.5`, lateral gain 1,
heading gain 0.1 and curvature feedforward gain 0.1. The steering limit moves
linearly from the vehicle limit at 4 m/s to 0.18 rad at 6 m/s.

For measured forward speed `v`, lookahead is `clip(abs(v)+0.3, 0.3, 4.5)` metres.
Steering is the sum of `atan(wheelbase * 2*y_lookahead/lookahead²)`, wrapped
heading error times 0.1, and reference curvature times 0.1, clipped to the
speed-dependent steering limit. Speed is the minimum of the frozen reference
speed, the launch `speed_limit` (default **1 m/s**) and the vehicle maximum.

```bash
ros2 launch aims_pp pp.launch.py \
  reference_directory:=/absolute/reference-directory \
  vehicle_config:=/absolute/vehicle.yaml \
  log_directory:=/absolute/new-run-directory \
  speed_limit:=1.0 repeat_laps:=false auto_start:=false
ros2 service call /pp/enable std_srvs/srv/SetBool '{data: true}'
ros2 service call /pp/enable std_srvs/srv/SetBool '{data: false}'
```

Launch arguments: `reference_directory`, `vehicle_config`, `pp_config` (blank
uses installed `config/pp.yaml`), `log_directory`, `speed_limit`, `repeat_laps`,
`auto_start` and `use_sim_time`. `auto_start` must be false: enable is explicit.
When logging is enabled, the directory must not contain an existing
`runtime.csv`; startup fails rather than overwriting earlier evidence.

The node publishes `/drive` as reliable `KeepLast(1)`
`AckermannDriveStamped` at 50 Hz on a steady-clock timer, with ROS publication time and `base_link`
frame. It also publishes `/pp/status` (`DiagnosticArray`, 10 Hz), a latched
`/pp/reference` (`Path`, `map`) and `/pp/lookahead_point` (`PointStamped`, `map`).
The `/pp/enable` service is `SetBool`. Startup, operator stop, fault and lap
completion all emit zero speed and zero steering. A repeated enable while
running is rejected so it cannot reset the lap association.

Reference directories must contain canonical `reference.json` with strictly
increasing `knots`, descending quintic `coefficients[6][interval][2]`, `length`,
`left_width`, `right_width`, `frame_id=map`, and a schema-1 frozen
`speed_profile` (`positions`, `speeds`). Metadata must identify a closed lap,
`source_frame=base_link`, matching map SHA-256 and complete matching vehicle
geometry (`wheelbase`, `rear_offset`, `half_length`, `front_extent`,
`rear_extent`, `half_width`). The coordinates describe the rear axle. Legacy
canonical bundle roots with this shape are accepted; the bundle's solver
library is never opened. Position, tangent and second derivative must agree
at every knot and closure. The loader checks finite values, nonzero sampled
tangents, width and speed compatibility before creating a drive publisher.

The trusted localization record comes from `/localization/status`, with the
single `aims_racer_system/localization` diagnostic record. The upstream
`/localization/anchor_status` supplies events to that monitor. Authorization
requires protocol 1, matching map hash and validity, a qualified atomic
`map_odom_*` transform bound to the epoch, anchor sequence and anchor source
stamp, anchor age at most 1 s in ROS and monotonic time, and heartbeat receipt
age at most 0.3 s. TF freshness is not used as localization quality evidence.
Publication continues when simulation time freezes, so monotonic source
freshness can still stop the controller.
The node also requires `/localization/map_sha256` and
`/localization/map_valid`, `odom/base_link` source and receipt ages at most
0.1 s, fresh `/control/autonomy_speed_enabled`, and a valid, fresh final
`/vesc/command` in speed mode. Nonunit quaternions, nonfinite inputs, future
stamps, discontinuous state/progress, forward heading mismatch, a second `/drive` publisher, an
epoch change or ROS clock rollback stop the run. Restored inputs never resume
it automatically; the operator must explicitly enable again.

Saved reference widths are nominal preparation widths and do not establish
measured track boundaries. The vehicle YAML's `enforce_corridor` controls the
body footprint gate; it defaults to false. Only an explicit true setting
stops the controller when its body footprint exceeds those widths.

One lap is measured from the initial projected rear axle position when enable
is accepted. Crossing the global reference seam immediately after enable is
not lap completion. `repeat_laps=true` permits repeated laps. The output
contains no solver scheduling, trajectory plan, or extra steering smoothing.

`runtime.csv` records callbacks and commands with ROS/monotonic times, source
age, callback duration, command gap, phase, speed, steering and lap progress.
Callbacks enqueue to a fixed 1024-row queue; a worker writes CSV. Queue drops
and writer errors appear in status.

The numeric APIs are `aims_pp.core.PPConfig`, `EnhancedPP`,
`aims_pp.reference.NumericReference`, and `aims_pp.tracking.LocalizationHealth`,
`TrackingInputs`, `PPTrackingRuntime`. Calibration can use
`TrackingInputs.localization_reason()` without asserting speed-mode authority,
and `PPTrackingRuntime.association()` for the shared forward check and optional
footprint corridor check controlled by `enforce_corridor`.
Its own final-command authority must still be checked by its owner.

Tests: `python3 -m pytest src/aims_pp/test -q`. ROS tests require a built Humble
workspace and an isolated `ROS_DOMAIN_ID`. The recovered-source parity test
uses a bundled verbatim numeric fixture and verifies its SHA/provenance; it
executes no hardware module and needs no external audit path. These tests establish
numeric and ROS interface behavior; they do not qualify ARM timing or real
vehicle tracking. The bounded coarse/Newton projector requires a path with
unambiguous local association: intersections or closely overlapping branches
need separate qualification. Stock Humble rclpy does not pass DDS message
information to callbacks, so final-command freshness uses the physical ROS
header and monotonic receipt; newer adapters providing message information
also check DDS source age.
