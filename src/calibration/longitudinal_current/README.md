# Longitudinal calibration

[简体中文](README.zh-CN.md)

`aims_calibration` measures longitudinal behavior through the existing vehicle command selector. It publishes only native `vesc_msgs/VescCommand` requests on `/calib/vesc_command`, at 50 Hz, stamped with current ROS time and `base_link`. Command and safety/status timers use steady time so they continue checking freshness when `/clock` pauses. The selector owns `/vesc/command`; this package never publishes `/drive` or the driver topic.

The executable and node-only launch are `longitudinal_calibration` and `longitudinal_calibration.launch.py`. The complete graph is `aims_racer_system/calibration.launch.py`. Package launch arguments: `config`, `reference_directory`, `vehicle_config`, `pp_config`, `log_directory`, `stage`, `steering_source`, `base_current_file`, and `auto_start`. Use the full system graph to configure vehicle, known-map localization, recording, and acceleration estimation. Defaults are disabled; explicit `/calib/enable` (`std_srvs/SetBool`) starts preparation. `auto_start` must be false; the node rejects true, requiring explicit service enable. `/calib/status` reports state, reason, measured current, requested current, and output directory.

## Packaged calibration commands

`longitudinal_calibration`, `inspect`, `start`, and `stop` are C++17 executables.
The calibration process runs the Python configuration/reference/provenance loaders once
at startup; that helper exits before the sustained ROS process starts. Its original
ROS parameter arguments are converted into typed native overrides, including empty
`currents` arrays. The 50 Hz control timer, source and steady receipt checks, explicit
enable preflight, PP association, and bounded evidence writer run in C++.
`baseline`, `plan`, and `results` retain the offline Python CLI; the original Python
node and state machine remain available as regression references. Runtime parameter
changes are rejected except `use_sim_time`; restart to load a new frozen configuration.
No temporary `field-tests` Python script, fixed PID or conversation authorization
string is required.

If the vehicle launch is already running, use only these commands:

```bash
ros2 run aims_calibration inspect
# Confirm pose/direction in RViz, unlock RC and select CH8 calibration; enable once:
ros2 run aims_calibration start
# To abort, lock with RC first; this leaves the vehicle launch and recorder running:
ros2 run aims_calibration stop
# Use the output_directory printed by inspect/start for offline acceptance:
ros2 run aims_calibration results /absolute/path/to/result-directory
```

The CLI and direct `/calib/enable` use the same node preflight: fresh observations
and selected authority, stationary wheel speed (absolute value <=0.1 m/s), a final
zero-speed request, disabled driver current-startup substitution, exclusive command
publishers and healthy logging. PP additionally requires matching trusted localization,
lateral distance <=0.25 m and heading error <=0.35 rad. ACC availability is checked
when the launch enables it. Start rechecks vehicle, PP, calibration, reference and
region file hashes. Correct a refused preflight and retry; an accepted start cannot
be repeated or reset. Completion, interruption or fault needs a fresh launch/session
for the next collection.

`start` submits one enable request. An uncertain response, lost monitor input or
Ctrl+C requests disable. It checks fresh wheel standstill continuously for 3 seconds
before reporting a stop. Zero current requests coasting, not guaranteed stopping;
unverified standstill explicitly requires operator intervention and never closes
the vehicle launch. `results` checks complete target coverage, configuration hash
and each record's `success` / `evidence_valid`. Empty, duplicate and partial runs
return nonzero exit codes.

In an NX graphical terminal, append `rviz:=true` to load the packaged `calibration.rviz` with the prior map and green reference. Software rendering is the default for the observed NX black-screen issue; `rviz_software_rendering:=false` selects GL rendering.

### Repository nine-speed preset

This preset is optional; default vehicle startup is unchanged. Site maps and exported
references remain data paths. All scripts, launch files, settings and documentation
are installed from the repository. The preset chooses packaged `speed_hold_1to5.yaml`,
`speed_hold_vehicle.yaml` and `speed_hold_pp.yaml`; no temporary vehicle/PP YAML is needed.

**Start this once only when no vehicle launch is running**. With an existing launch,
use the commands above instead:

```bash
ros2 launch aims_racer_system calibration.launch.py \
  profile:=speed_hold_1to5 steering_source:=pp \
  map_file:=/absolute/path/to/map.pcd \
  reference_directory:=/absolute/path/to/pp-routev2-straight-trials \
  record:=true longitudinal_acceleration:=true auto_start:=false
```

Nine targets 1.0, 1.5, ..., 5.0 m/s each settle for 1 second and accumulate 5 seconds
of eligible straight samples. Turns use 1 m/s recovery; turns and unstable intervals
add no acquisition time. `/calib/reference` is a latched `nav_msgs/Path`; add a Path
display in RViz with Fixed Frame `map`. NDT pose corrections retain `/initialpose`.
Use the printed result and bag session paths, not a fixed PID. After physical
standstill, Ctrl+C in the launch terminal and verify that session's `bag`.

## Stages and generated sweeps

| Stage | Packaged profile | Measurement |
| --- | --- | --- |
| A `speed_hold` | `speed_hold_1to5` | Actual cruising motor current at nine speeds, five cumulative straight seconds each |
| B `accel_interval` | `accel_sweep_1to5` | Adjacent +0.5 m/s intervals with currents `I0(start)+5k`, capped at 100 A |
| C `decel_current` | `decel_sweep_1to5` | Below-baseline positive current to start−0.5, then coast and negative current to 1 m/s |

Use the [B/C operator guide](../../../docs/operations/full-step-current.zh-CN.md) for baseline
export, plan preview, launch and result definitions. `baseline` and `plan` share
`cli.py` with the other four operator entries and run offline. B/C require an
explicit accepted nine-speed baseline; no session is selected automatically.

`baseline --iq-polynomial C0 C1 C2 C3` explicitly selects a cubic Iq planning
baseline over 1–5 m/s. Schema v3 stores `planning_model` alongside the unchanged
measured A points and source identities. B/C evaluate start speeds to freeze the
plan. Without that option, measured-point schema v2 remains available.

Each current item restores its own start speed, settles one second within ±0.2 m/s,
then applies native current mode. Region exit retries the same item at most three
times; PP curves retain 1 m/s recovery. Current tracking uses measured `avg_iq`.
B deviations remain diagnostic. C starts at 5 m/s and descends to 1.5 m/s;
a supported current-failure window or reverse motion during/after a real current
attempt skips every remaining item at that speed. Selected zero current and one
second of fresh nonnegative wheel observations precede the next speed group.
Skips retain trigger evidence and leave measurement coverage gaps. A speed timeout
alone does not skip. A fresh successful endpoint takes
priority; short responses can have unverified current tracking. Current bounds are
±100 A, speed bounds at most `min(config.max_speed, vehicle.max_speed, 5)` m/s.
B/C profiles freeze their complete structured plans and use schema v2; A retains v1.

Manual mode still accepts explicit positive B or negative C `currents` and a single
start/end interval. Its default empty list refuses current-stage startup. Pass
matching `stage` and `steering_source` to the node-only launch; the system launch
resolves packaged profiles with `stage:=auto`. Current startup substitution must be
false: the selected request is not the measured motor current.

## Nine-target cruising-current preset

`config/speed_hold_1to5.yaml` requests `[1.0, 1.5, 2.0, 2.5, 3.0, 3.5, 4.0, 4.5, 5.0]`, five effective source seconds per target, one repeat, one second of settling after each pause, and 1.0 m/s recovery outside trial regions. A target may span multiple straights; its five seconds are cumulative, not continuous.

Select `profile:=speed_hold_1to5` to use the packaged vehicle and PP YAML, or pass a separate vehicle YAML with `max_speed: 5.0`. A 4 m/s vehicle config rejects this sweep rather than skipping its last targets. The field patch removes the measured-overspeed stop; admitted hold samples still require the target tolerance and requested speeds remain capped at 5.0. Diagnostics expose current target, accumulated seconds, segment count, plan length and sampling mode. Permission, localization and evidence faults still invalidate the current target; a new launch/session starts again at the first target.

## Steering and trial windows

`steering_source: pp` uses `aims_pp`'s recovered enhanced PP implementation, geometry from `vehicle_config`, `NumericReference`, and the same qualified localization and map identity gates as standalone PP. Tuning defaults are lookahead gain 1, minimum 0.3 m, maximum 4.5 m, heading gain 0.1, and curvature feedforward gain 0.1. `pp_config` optionally overrides tuning. Geometry, forward heading, pose continuity, source stamps, health epoch, and map hash must agree. When the vehicle profile enables `enforce_corridor`, the shared PP association also checks the vehicle footprint against reference widths. PP follows the lap at `recovery_speed` outside marked straight windows and uses speed mode while preparing.

PP requires `reference_directory/reference.json` and an explicit `calibration_regions.json` exported by the editor. Region schema is `{schema_version: 1, reference_sha256: "<hash of exact reference.json bytes>", regions: [{id: "straight1", name: "straight 1", kind: "straight", start_s: 2.0, end_s: 10.0}]}`. Progress uses the reference knot/chord coordinate. Intervals include their start and exclude their end. `start_s > end_s` wraps over the closed lap seam; equal endpoints and overlapping regions are rejected. A manual straight mark does not verify available distance, curvature, tire grip, or safe current. Cumulative speed hold closes its current sample segment on exit or region change, then requires settling before starting another. Segments retain separate source timestamps and current statistics; turns and settling never contribute time. Continuous hold and current-response trials still reject incomplete trials on region exit.

`steering_source: rc` needs no localization or reference. Wheel odometry supplies velocity. The explicit default profile matches the vehicle launch: `steering_ch1_throttle_ch3_aux_ch5_to_ch10`, steering CH1, throttle CH3, lock CH5, ESC CH6, source CH7, calibration CH8, limit CH10. Values are 1-based. Center is 992, endpoints 172/1810, steering reversed, selector steering scale 0.4751 rad. Sampling is eligible only within the 50-count steering deadzone, with fresh valid RC, unlocked CH5 >= 992, and calibration CH8 > 992. A turn pauses cumulative hold, or invalidates a continuous hold/current-response trial, and returns to speed mode; the operator may steer while recovering. `sequential_ch1_ch2` is available only when the selector uses that explicit profile (steering CH2, lock CH3, calibration CH7). Confirm the selected profile before any current-stage test.

## Authority, freshness, and stops

While disabled at startup, fresh zero-speed requests let the selector establish calibration selection. Enabling waits for fresh `/control/calibration_enabled == true`, then requires independent RC validity, new wheel/current data, and a fresh final `/vesc/command` corresponding to a recently sent request. The selector restamps commands, so content is matched against a short request history rather than requiring the latest request timestamp. Only the active mode target is populated; positive and negative current both use `MODE_CURRENT`. `MODE_BRAKE_CURRENT` and `MODE_ACCELERATION` are never emitted.

Authority loss, RC lock/mode loss, source/receipt age over 0.2 s, invalid telemetry, foreign final command, map/localization withdrawal, backward ROS/source time, frozen ROS time, timer stall, or another calibration publisher latches `FAULT`, invalidates the current trial, and continuously requests zero motor current until a fresh session is explicitly enabled. Operator disable also invalidates the incomplete trial and requests zero motor current. A fresh-session enable establishes a new ROS-time baseline, clears cached observations and request history, and starts preparation. Authority, RC, wheel, and motor source observations must arrive after the new arm time, and a newly sent zero-speed request must appear in the final command before preparation can move. Runtime clock rollback still latches a fault; a fresh session can recover in the new clock domain without waiting for the old timestamp. It never resumes a partial interval. Stage completion requests zero speed. A zero-current request is a coast request, not a physical stop guarantee; the external selector and operator retain control.

## Evidence and metric definitions

Each node run creates a UTC-named directory below `log_directory` with `observations.csv`, `runtime.csv`, atomic `trials.json`, and `trials.csv` once a trial exists. JSON contains effective configuration, vehicle hash, reference and region hashes for PP, and the explicit RC profile. Effective PP values and file SHA-256 (when a tuning file is supplied), calibration YAML SHA-256 (through the package launch), and a canonical effective-configuration hash are retained. A bounded background queue writes evidence; overflow or writer errors invalidate acquisition and request zero current. If the just-completed boundary record cannot be enqueued, its trial success is revoked, `evidence_valid` is false, `mean_acceleration` is unavailable, and `response_completed` retains whether the response had crossed its endpoint. A pending result checkpoint retries once the queue can accept it. `trials.json` is the authoritative acceptance checkpoint: the CSV checkpoint completes first, and a writer error immediately attempts to publish rejected JSON results independently of the failed CSV path, including when no later result is queued. The CSV may retain an earlier checkpoint after a write failure; inspect JSON metadata and diagnostics for evidence validity. Writer failures also invalidate response records whose evidence checkpoint has not completed. JSON metadata retains evidence errors; persistent storage failure remains visible in diagnostics even if the disk cannot retain a new checkpoint. Diagnostics expose pending and dropped queue counts. `runtime.csv` retains ROS time, callback duration (through acquisition enqueue), timer interval, queue counts, and state for later joint-load analysis. It does not prove ARM timing. Raw observations include source wheel and telemetry stamps, wheel velocity, `state.avg_iq`, separate `state.current_motor`, `state.voltage_input`, electrical RPM (`state.speed`), requested and selected targets, authority, steering/window flags, and optional acceleration estimate, source stamp, and quality status. Missing or stale acceleration is recorded as unavailable and never replaces interval wheel-speed response.

Only finite `/sensors/core.state.avg_iq` supplies torque-baseline and current-tracking evidence. Missing Iq refuses acquisition. `current_motor` is an independent diagnostic; `current_input` is battery-side current. Neither replaces Iq or proves an achieved target. A row needs both source stamps to advance; timer repetitions never duplicate an observation. Stable and response windows use wheel source time. Cumulative hold integrates wheel-source differences only between consecutive unique admitted observations in the same segment, with both source gaps bounded by freshness. Duplicate telemetry and gaps between segments add no time. Current interval samples begin only when both physical source stamps are at or after the first selected-current command stamp (`first_current_ack_stamp`). That bound is latched once; later republished command stamps do not exclude valid delayed telemetry. Current interval response is `(measured end speed - measured start speed) / (end wheel stamp - start wheel stamp)` only for successful trials. Boundaries are the first eligible observed crossing; no interpolation is implied. Telemetry current and wheel velocity have separate stamps, so the CSV retains their offset. Stage A `mean_iq` and `std_iq` use admitted q-axis samples; motor statistics remain separate diagnostics. Cumulative hold reports effective sample time as `duration_s`, total first-to-last time as `wall_duration_s`, and individual `segments` with regions, wheel/telemetry source endpoints, durations, counts and current statistics. A current interval already crossed on its first selected-current sample is unsuccessful because its source-time duration cannot be measured. These statistics describe telemetry samples, with no claim of independent noise or bench-calibrated current accuracy.

Tests run in ROS Humble without hardware. Source and synthetic tests demonstrate software behavior; they do not establish physical stop distance, current accuracy, realtime guarantees, or safe field currents.


## Legacy B/C interval predictive exit (2026-10-15)

Only the legacy `accel_sweep_1to5` / `decel_sweep_1to5` interval profiles use `predictive_exit`; this paragraph does not apply to `full_range_step`. They use 2 m/s turn recovery. Pulse release first requests the frozen Stage A holding current at the endpoint, confirms selected command and two fresh actual IQ samples, then waits for a quiet speed hold before another trial. The default 140 ms exit budget plus source age is an assumption, not a hardware deadline. Infeasible or unsupported speed/current pairs are recorded without applying their pulse. Early pulse measurements (`pulse_mean_acceleration`) remain distinct from completed interval coverage.

The packaged response limits bind to the recorded A baseline; use `response_limits_file` for a regenerated matching model. IMU observer values are corroborative only. Neutral current does not guarantee physical standstill.

## Full-range constant-current acquisition

The additional `accel_full_step_1to5` and `decel_full_step_5to1` profiles hold
one requested current from an independently prepared start speed until a
measured wheel-speed endpoint or steady response. They retain truncated
attempts and measured-Iq handoff tails, using version 3 result evidence.
Endpoint prediction and historical-response-table refusal are disabled for this protocol.
See the [operator guide](../../../docs/operations/full-step-current.zh-CN.md).


## Native build and verification

Build this package with `ament_cmake` after its ROS message dependencies and the
native `aims_pp` library. JSON/OpenSSL development headers are required (Ubuntu:
`nlohmann-json3-dev` and `libssl-dev`). The original launch and executable names
are unchanged; the installed calibration executable is an ELF binary.

```bash
colcon build --packages-up-to aims_calibration
colcon test --packages-select aims_calibration --event-handlers console_direct+
colcon test-result --verbose
```

Registered tests include the C++ writer/watchdog contracts, native/Python public-API
trace parity, isolated ROS process/operator tests, and retained Python numerical,
configuration, planner, evidence, and acceptance regressions. Process tests exercise
explicit start, frozen ROS time, source/receipt expiry, duplicate publishers, writer
failure, original full-step YAML startup, and the C first-wheel trigger using synthetic
peers. They do not constitute vehicle or target-platform real-time acceptance.

Offline wheel-current tools are included: [model guide (Chinese)](docs/wheel-current-model.zh-CN.md). `wheel-current-fit` and `wheel-current-query` publish no commands and do not replace the selected model.
