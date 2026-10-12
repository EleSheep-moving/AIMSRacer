# Longitudinal acceleration runtime implementation plan

> **For agentic workers:** Use superpowers:subagent-driven-development for the node task and independent specification / quality reviews. The root coordinates launch, deployment and source preservation.

**Goal:** Prepare NX to record the selected wheel + IMU observer during two ordinary field laps without changing navigation EKF or vehicle commands.

**Authorization:** The user accepted the previous recommendation to merge experimental evidence, implement a separate runtime acceleration output, and field-test before default activation. Preserve and commit the already deployed rear-adapter source-time fixes first. Existing motor-command execution and navigation inputs remain outside this change.

**Architecture:** One canonical NumPy observer is installed with `aims_racer_system`. A dedicated rclpy process consumes source-stamped wheel speed and corrected rear IMU specific force. It publishes `AccelWithCovarianceStamped` with longitudinal body-speed derivative in linear.x and model covariance, plus a separate diagnostic status. It uses no LIO, TF, map, command or navigation-EKF subscriptions. Vehicle/race launch exposes opt-in `longitudinal_acceleration:=true`; launch-managed recording includes estimates and status. ROS callbacks are serialized within this small dedicated process to preserve actual receipt order.

**Tech stack:** ROS 2 Humble, rclpy, NumPy, geometry_msgs, diagnostic_msgs, existing ament_cmake_python installation. Existing Python and ROS dependencies already declared in package.xml.

## Approved runtime contract

- Input `/rear_axle/wheel_odom`: nav_msgs/Odometry, only twist.linear.x in child frame base_link. Input `/rear_axle/imu`: sensor_msgs/Imu in base_link, only valid linear_acceleration.x. Sensor-data QoS subscriptions accept current publishers. Ignore orientation entirely.
- Wheel values are ERPM-derived and already contain existing near-zero handling. Force already has unit/mount/lever correction; never rescale, reapply lever correction or remove gravity in this node.
- Fixed selected settings: jerk_psd=30, disturbance_psd=0.1, speed_variance=0.01, force_variance=16, use_imu=true, use_gravity=false; history 2s / 4096 observations. These are effective tuning values, not calibrated physical noise.
- Admit observations with exact integer source ns only after callback receipt. Emit after IMU callbacks, predict scalar state to current ROS publication time, and stamp that estimate time. Upstream source timestamps never change. Never forward-predict LIO pose/attitude.
- Require a valid wheel observation and a valid force observation before first output; distinguish initialization from force unavailable later. After initialization unavailable force is skipped, not inserted as zero; wheel/model estimate may continue on IMU callbacks, with warning diagnostics.
- Frames, finite scalars and time domains are validated. Unsupported/future timestamps are counted and not inserted. ROS clock reversal resets observer plus initialization flags/history, with a visible reset counter. No old data carried into a new playback epoch.
- Output `/state_estimation/longitudinal_acceleration`: geometry_msgs/AccelWithCovarianceStamped, frame base_link, linear.x=d(body_vx)/dt, covariance[0]=observer acceleration variance. Other means zero, other diagonal entries 1e6 to show unobserved components. This quantity is not full inertial vector acceleration; no automatic navigation EKF fusion.
- Output `/state_estimation/longitudinal_acceleration/status`: DiagnosticArray at 1Hz, describes initialization, source ages, force validity, model estimate/variance/disturbance, accepted/rejected/history counters and per-kind callback timing. Source age warnings are diagnostic only and never hard stop output. No falsely repeated IMU when input absent.
- No command publishers, TF, additional output smoothing, hardware drivers or parameter-dependent motor changes. B is opt-in in vehicle/race launch and standalone through `longitudinal_acceleration.launch.py`.

## Tasks

### 1. Preserve current source-time fixes

- [x] Hash-compare local/NX 16 pending files, save patch and tar under `sessions/2026-10-12/longitudinal-acc-runtime`.
- [x] Run existing NX package tests before committing; expected no failures. Actual 89 tests, 0 failures.
- [x] Commit source-time fixes as `ab0374a` on main; merge main into experiment branch before runtime implementation. Source changes are independent of acceleration model.

### 2. Shared observer and ROS wrapper (worker ownership)

Files: create `src/aims_racer_system/aims_racer_system/longitudinal_observer.py`, `longitudinal_acceleration.py`, `scripts/longitudinal_acceleration.py`, `tests/test_longitudinal_acceleration.py`; replace `verification/acceleration/observer.py` with a shim importing the same canonical core.

- [x] Add failing tests for public runtime module, ROS output, correct dimensions, source/estimate stamps, initialization, invalid force, wrong frames, late wheel update, old/future input and playback clock reversal. Test through real ROS messages/context; no hardware commands.
- [x] Preserve observer mathematics byte-for-byte apart from module location. Existing 27 numerical/pipeline tests must still pass.
- [x] Implement wrapper and test steady stationary force offset plus known acceleration. A real DDS test checks publications without LIO and verifies no TF or drive publisher.
- [x] Save failing and passing logs using the existing Docker image and source ROS environment. Commit owned files only after root verification/reviews.

### 3. Launch and recording (root ownership)

Files: CMakeLists.txt; new `params/longitudinal_acceleration.yaml`; new `launch/longitudinal_acceleration.launch.py`; vehicle.launch.py; race.launch.py; launch_support.py; test_current_launch.py; recording_qos.yaml if required.

- [x] Add launch regressions first: defaults off, explicit true includes exactly one observer, race propagates flag, standalone has no drivers/TF/controller, recording retains accel/status and original inputs.
- [x] Install script and add package pytest targets for core/runtime. YAML settings match frozen selected B configuration.
- [x] Standalone command: `ros2 launch aims_racer_system longitudinal_acceleration.launch.py` against already running vehicle. Integrated: existing race/vehicle command adds `longitudinal_acceleration:=true`; `record:=true` retains both new topics. Do not start standalone plus integrated instance simultaneously.
- [x] Preserve all existing vehicle parameters, RC authority, EKF, NDT and MPCC defaults exactly.

### 4. NX verification and merge

- [x] Independent spec review, then code-quality review; fix required findings with regression evidence.
- [x] Build/test aims_racer_system in existing desktop image with isolated outputs; full numerical/pipeline checks and installed launch arguments.
- [x] Sync source to an isolated NX workspace or approved main after verified merge. Preserve old NX pending files in a named stash/backup before fast-forward, verify restored canonical files identical.
- [x] Build/test NX with bounded workers; source installed module and check standalone launch arguments. Synthetic ROS tests use isolated localhost domain; no live drivers.
- [x] Replay corrected frozen force + wheel inputs through actual ROS wrapper at approximately real time on NX and compare against canonical receive/snapshot oracle at emitted timestamps. Record delivery, data age and callback timings; no claim of localization joint-load or field acceptance.
- [x] Commit/review/merge main, synchronize GitHub and NX as previously authorized, verify source SHA / branch / installed interfaces. Leave no verification ROS processes.

### 5. User-facing documentation

- [x] Add English/Chinese `docs/operations/longitudinal-acceleration.md` guides, index from README and docs index. Update package/script guides and architecture with exact output semantics and opt-in flag.
- [x] Add brief field steps: ordinary vehicle/race graph, observer enabled, recording enabled, static interval then two normal laps. Existing controller commands continue; acceleration remains observational.
- [x] Explicitly distinguish offline evidence, installed realtime readiness, real-time replay and pending field/load validation. Describe specific force f=a-g, conversion a=f+g and body-derivative rotation terms consistently.

## Verification commands

```bash
source /opt/ros/humble/setup.bash
python3 -m pytest -p no:cacheprovider verification/acceleration -q
colcon build --packages-select aims_racer_system --parallel-workers 1 --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
colcon test --packages-select aims_racer_system --event-handlers console_direct+
colcon test-result --verbose
ros2 launch aims_racer_system longitudinal_acceleration.launch.py --show-args
```

Evidence directory: `/home/elesheep/aimsracer-data/sessions/2026-10-12/longitudinal-acc-runtime`; NX equivalent under `/home/aims/aimsracer-data/sessions/2026-10-12/longitudinal-acc-runtime`. Actual launch/delivery results, source hashes and remaining field acceptance must be recorded before reporting readiness.

## Completion evidence

Implementation `ba81fc7` is merged to main and installed on NX. Desktop, isolated NX and NX main package checks each report 110 tests with zero errors/failures/skips. Both independent reviews passed. NX DDS replay received all 8,967 inputs and delivered all 7,170 outputs; scalar/covariance oracle error zero. The named NX stash preserves preceding source-time changes. Two field laps and localization joint-load evaluation remain the next vehicle acceptance step. See [runtime report](../reports/2026-10-12-longitudinal-acc-runtime.md).
