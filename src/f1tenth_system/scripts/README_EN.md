# Calibration Data Collection Tools (Longitudinal / Lateral)

This folder currently maintains two types of calibration tools:

- **Longitudinal calibration**: maps speed, current, acceleration, and deceleration for current control and speed planning.
- **Lateral grip / slip calibration**: drives fixed-radius circles with stepped speed targets to estimate centripetal acceleration, lateral force, effective friction, and slip onset.

Longitudinal calibration provides two data-collection approaches:

- **Localization-based (Pure Pursuit Auto Looping)**: requires stable odometry/localization. The vehicle follows a “stadium / figure-8” style trajectory automatically; Stage A/B/C collection runs on straights and curves are handled in speed mode. This is suitable for long, repeatable data collection, but typically requires a relatively large open area/track.
- **RC-intervention (Manual Steer, No Localization)**: does not rely on localization. The scripts publish longitudinal commands ("current sweep on straights / speed hold on turns"), while lateral control (steering) is provided by the RC transmitter. This is recommended when odometry drifts (e.g., fastlio/IMU issues) or when you do not have a large enough track.

Both approaches publish the same topic: `/calib/ackermann_cmd` (downstream must parse the `jerk` convention below).

---

## 0.1) Build & Environment Setup (Required)

These scripts are installed as executables via `ament_python`. After the first use or after script updates, rebuild:

```bash
cd ~/RallyCore
colcon build --packages-select f1tenth_system
```

Then source in the **current terminal** (pick the one matching your shell):

```bash
# bash
source install/setup.bash

# zsh
source install/setup.zsh
```

---

## 0.2) `/calib/ackermann_cmd` Convention (Downstream Must Follow)

This repository uses `AckermannDriveStamped.drive.jerk` as a "mode flag" to multiplex speed mode and current mode on the same topic:

- `AckermannDriveStamped.drive.jerk == 2.0`: **current mode**
  - `drive.acceleration` = current (A)
  - `drive.steering_angle` = steering angle (rad)
  - `drive.speed` not used (can be 0)
- `AckermannDriveStamped.drive.jerk == 0.0`: **speed mode**
  - `drive.speed` = target speed (m/s)
  - `drive.steering_angle` = steering angle (rad)
  - `drive.acceleration` can be 0

---

## 0.3) Lateral Grip / Slip Calibration: Fixed-Radius Speed Steps

Script: `src/f1tenth_system/scripts/lateral_grip_calib.py`

Purpose: actively drive fixed-radius circles, increase speed step by step, and estimate lateral capacity from **odometry speed + IMU yaw rate**:

- `ay_yaw = v_odom * yaw_rate_imu`
- `mu_y = abs(ay_yaw) / 9.81`
- `Fy = vehicle_mass * ay_yaw`
- `radius_est = v_odom / yaw_rate_imu`

Default vehicle mass is `vehicle_mass:=4.5` kg. The first version only tests fixed-radius circles; it does not scan a full race line. `/odom.angular.z` is diagnostic only because the current VESC odom yaw rate may be model-derived.

Subscribes:
- `/odom` (`nav_msgs/Odometry`, default speed source)
- `/livox/imu_ekf` (`sensor_msgs/Imu`, default yaw-rate / lateral-accel source)
- `/sensors/core` (`vesc_msgs/VescStateStamped`, telemetry and safety check)

Publishes:
- `/calib/ackermann_cmd` (`AckermannDriveStamped`, `jerk=0.0` speed mode)
- `/calib/lateral_status_text` (`visualization_msgs/Marker`, RViz status text)

Safety default: with `armed:=false`, the node publishes **no motion command**. It only waits for/checks topics and prints the configuration. Real vehicle runs must explicitly enable arming:

```bash
ros2 launch f1tenth_system lateral_grip_calib.launch.py \
   armed:=true \
   vehicle_mass:=4.5 \
   test_radius:=3.0 \
   speed_start:=0.5 \
   speed_end:=4.0 \
   speed_step:=0.25
```

Common parameters:

```bash
# topics / outputs
ros2 launch f1tenth_system lateral_grip_calib.launch.py \
   armed:=true \
   odom_topic:=/odom \
   imu_topic:=/livox/imu_ekf \
   vesc_topic:=/sensors/core \
   output_dir:=lateral_grip_run1

# If IMU signs are inverted, validate at low speed and then adjust signs.
ros2 launch f1tenth_system lateral_grip_calib.launch.py \
   armed:=true \
   imu_yaw_axis_sign:=-1.0 \
   imu_lateral_axis_sign:=1.0
```

Output files:
- `lateral_grip_samples.csv`: per-sample data.
- `lateral_grip_results.csv`: per direction/speed statistics, including `mu_y`, `Fy`, and radius error.
- `lateral_grip_summary.yaml`: `mu_left`, `mu_right`, `mu_safe`, and safe lateral-accel limit for later controller use.

Slip / limit detection:
- IMU yaw rate deviates from `v/R` for longer than the confirmation window.
- Estimated radius `radius_est` deviates from commanded radius.
- `mu_y` exceeds `mu_abort`.
- Speed keeps increasing but lateral acceleration stops increasing.
- odom/IMU/VESC timeout or user interruption.

Start with a low-speed sign check:

```bash
ros2 launch f1tenth_system lateral_grip_calib.launch.py \
   armed:=true \
   speed_end:=1.5 \
   test_radius:=3.0
```

Suggested rosbag recording:

```bash
ros2 bag record -o lateral_grip \
   /odom \
   /livox/imu_ekf \
   /sensors/core \
   /calib/ackermann_cmd \
   /calib/lateral_status_text \
   /tf /tf_static
```

---

## 1) Localization-based: Pure Pursuit Auto Loop Calibration (Requires `/odom`)

### 1.1 When to Use

- You have stable `nav_msgs/Odometry` (e.g., `/odom` from EKF / fastlio / wheel odometry fusion).
- You want repeatable long-duration collection.
- You have a relatively large open area/track (long straights + safe turn radius). Without space, the loop-based method becomes hard to run safely and cleanly.

### 1.2 Core Scripts

#### (1) `longitudinal_calib.py`: Auto loop + segmented current/speed calibration

Script: `src/f1tenth_system/scripts/longitudinal_calib.py`

Subscribes:
- `/odom` (`nav_msgs/Odometry`)
- `/vesc/sensors` (`vesc_msgs/VescStateStamped`, telemetry)

Publishes:
- `/calib/ackermann_cmd` (`ackermann_msgs/AckermannDriveStamped`)
- `/calib/current_trajectory` (`nav_msgs/Path`, generated global trajectory for RViz)
- `/calib/lookahead_point` (`geometry_msgs/PointStamped`, PP lookahead point for RViz)
- `/calib/status_text` (`visualization_msgs/Marker`, stage/speed/current status for RViz)

RViz: open `src/f1tenth_system/rviz/pp.rviz`; `Calib Trajectory`, `Calib Lookahead`, and `Calib Status` show the generated path, PP lookahead point, and live speed/current/stage text.

Recommended three-stage PP workflow:

```bash
# Stage A: hold speeds and measure mean current
ros2 run f1tenth_system longitudinal_calib.py --ros-args \
   -p workflow:=pp_speed_hold \
   -p speeds:="[1,2,3,4,5,6,7,8]" \
   -p hold_time_sec:=10.0 \
   -p output_path:=speed_hold_current_results.txt

# Stage B: current-step acceleration trials based on Stage-A current map
ros2 run f1tenth_system longitudinal_calib.py --ros-args \
   -p workflow:=pp_accel_interval \
   -p v_start:=1.0 -p v_end:=8.0 -p dv:=1.0 \
   -p base_current_file:=speed_hold_current_results.txt \
   -p current_step:=3.0 -p current_max:=80.0 \
   -p output_path:=speed_interval_accel_results.txt

# Stage C: negative-current deceleration trials
ros2 run f1tenth_system longitudinal_calib.py --ros-args \
   -p workflow:=pp_decel_current \
   -p v_start:=3.0 -p v_end:=8.0 -p dv:=1.0 \
   -p decel_low_speed:=1.0 \
   -p decel_current_step:=3.0 -p decel_current_min:=-20.0 \
   -p output_path:=decel_current_sweep_results.txt
```

Common parameters (examples):

```bash
# track geometry (start with larger radius/longer straights to reduce lateral disturbance)
ros2 param set /longitudinal_calib track_radius 3.0
ros2 param set /longitudinal_calib track_straight_length 10.0

# PP parameters (tune with pp_param_tuner first, then copy here)
ros2 param set /longitudinal_calib lookahead_gain 1.0
ros2 param set /longitudinal_calib min_lookahead 0.3
ros2 param set /longitudinal_calib max_lookahead 4.5
ros2 param set /longitudinal_calib lateral_error_gain 1.0
ros2 param set /longitudinal_calib heading_error_gain 0.1
ros2 param set /longitudinal_calib curvature_ff_gain 0.1

# PP high-speed steering limit (only limits PP-generated steering)
ros2 param set /longitudinal_calib max_steering_angle 0.35
ros2 param set /longitudinal_calib steering_limit_start_speed 4.0
ros2 param set /longitudinal_calib steering_limit_full_speed 6.0
ros2 param set /longitudinal_calib high_speed_max_steering_angle 0.18

# trajectory offsets w.r.t. localization frame (for on-site alignment)
# default true: place the trajectory local origin at the first odom (x,y), then apply x/y below as fine offsets
ros2 param set /longitudinal_calib use_first_odom_as_origin true
ros2 param set /longitudinal_calib traj_offset_x 0.0
ros2 param set /longitudinal_calib traj_offset_y 0.0
ros2 param set /longitudinal_calib traj_offset_yaw 0.0
```

Logic summary:
- Closed-loop “stadium / figure-8”: straights for collection, curves for speed-mode recovery/holding.
- By default, after the first odom message arrives, the trajectory local origin is translated to that odom `(x,y)`. `traj_offset_x/y/yaw` remain live fine-tuning offsets. Set `use_first_odom_as_origin:=false` to restore the old behavior where the trajectory origin is the odom origin.
- PP defaults now match `pp_param_tuner.py`: `lookahead_gain=1.0`, `max_lookahead=4.5`, and `heading_error_gain=0.1`. If the car oscillates above 2m/s, first watch `ld` in RViz/logs, then slightly increase `lookahead_gain/max_lookahead` or lower `heading_error_gain`.
- PP steering has a speed-dependent limit: `v<=4.0m/s` allows up to `0.35rad`, `4.0<v<6.0m/s` linearly tightens to `0.18rad`, and `v>=6.0m/s` stays at `0.18rad`. This applies only to `pp_speed_hold / pp_accel_interval / pp_decel_current`; RC/manual no-localization workflows are unchanged.
- `/calib/status_text` and node logs show raw steering, effective `steer_limit`, `ld`, and whether the command was `clipped`.
- **Stage A / `pp_speed_hold`**: for each speed point, stabilize first, then accumulate `hold_time_sec` of current samples only on straights.
- **Stage B / `pp_accel_interval`**: for each `v0→v1` interval, reset to `v0`, then run current-mode trials only on straights; entering a curve aborts the current trial and resets to `v0`.
- **Stage C / `pp_decel_current`**: stabilize at each target speed, then apply negative current only on straights; entering a curve aborts the current decel trial and recovers target speed.

The old `workflow:=pp_auto` continuous sweep behavior has been removed; PP collection now uses Stage A/B/C only. Stage B / `pp_accel_interval` uses `current_start_step_index` plus its own current-list index to control the starting current.

End behavior: when a workflow completes, Ctrl-C is pressed, or the node exits, it publishes repeated stop commands: first zero-current current-mode commands, then speed-mode commands with `speed=0`.

Suggested rosbag topics:

```bash
ros2 bag record -o pp_calib \
   /odom \
   /vesc/sensors \
   /calib/ackermann_cmd \
   /calib/current_trajectory \
   /calib/lookahead_point \
   /calib/status_text \
   /tf /tf_static
```

#### (2) `pp_param_tuner.py`: PP parameter tuning helper (optional but strongly recommended)

Script: `src/f1tenth_system/scripts/pp_param_tuner.py`

Note: default interface differs from the calibration node:
- Subscribes: `/odometry/filtered`
- Publishes: `/drive`
- Visualization: `/calib/current_trajectory`, `/calib/lookahead_point`, and `/calib/status_text`, matching `longitudinal_calib.py` and `src/f1tenth_system/rviz/pp.rviz`.
- Shutdown: on Ctrl-C / node exit, it publishes repeated stop commands with `speed=0`.

If your system uses `/odom` or needs output to `/ackermann_cmd`, use remap/bridge without changing code.

Quick run:

```bash
ros2 run f1tenth_system pp_param_tuner.py --ros-args -p target_speed:=2.0
```

The tuner uses the same high-speed steering limit defaults and supports live tuning:

```bash
ros2 param set /pp_param_tuner max_steering_angle 0.35
ros2 param set /pp_param_tuner steering_limit_start_speed 4.0
ros2 param set /pp_param_tuner steering_limit_full_speed 6.0
ros2 param set /pp_param_tuner high_speed_max_steering_angle 0.18
ros2 param set /pp_param_tuner use_first_odom_as_origin true
```

Remap examples:

```bash
# 1) Use /odom instead of /odometry/filtered
ros2 run f1tenth_system pp_param_tuner.py --ros-args \
   -r /odometry/filtered:=/odom

# 2) Publish to /ackermann_cmd instead of /drive
ros2 run f1tenth_system pp_param_tuner.py --ros-args \
   -r /drive:=/ackermann_cmd
```

---

## 2) RC-intervention Calibration (No Localization)

Goal: avoid dependence on localization when odometry drifts.

The recommended entrypoint is `longitudinal_calib.py`; select the no-localization collection path with `workflow`:
- `workflow:=speed_hold`: Stage A, hold speeds and measure mean current.
- `workflow:=accel_interval`: Stage B, run current-step trials for each speed interval.
- `workflow:=decel_current`: optional Stage C, negative-current decel trials.

RC intervention behavior:
- **Straight** (RC steering within deadzone): allow sampling/trials and optionally force `steering=0`.
- **Turning** (RC steering outside deadzone): pause sampling/trials and hold a safe speed in speed mode.
- **Steering**: controlled by RC; after returning straight, wait `post_turn_settle_sec` before resuming.

Output: `/calib/ackermann_cmd`

---

## 3) Recommended New Workflow: Stage A speed-hold current, then Stage B interval acceleration sweep

When the previous methods do not work well, use this two-stage workflow.

### 3.1 Stage A: hold speeds 1..8 m/s and measure mean phase current

Goal: on straight segments (`steering=0`), measure the mean current required to hold each speed.

Script: `src/f1tenth_system/scripts/longitudinal_calib.py`

Run example:

```bash
ros2 run f1tenth_system longitudinal_calib.py --ros-args \
   -p workflow:=speed_hold \
   -p speeds:="[1,2,3,4,5,6,7,8]" \
   -p hold_time_sec:=10.0 \
   -p use_rc_steering:=false \
   -p vesc_topic:=/sensors/core \
   -p output_path:=speed_hold_current_results.txt
```

If you must enable RC steering intervention:

```bash
ros2 run f1tenth_system longitudinal_calib.py --ros-args \
   -p workflow:=speed_hold \
   -p use_rc_steering:=true \
   -p rc_topic:=/rc/channels \
   -p rc_timeout_sec:=0.25 \
   -p post_turn_settle_sec:=0.8
```

Output:
- `speed_hold_current_results.txt`: mean current per speed (the default strategy is: wait until speed reaches target and remains stable for a while, then sample)
- optional `csv_path`: full time series for debugging

#### 3.1.1 Voltage / SOC Stratified Collection

`I0(v)` will not be perfect across all battery states. For the first voltage-aware workflow, run Stage A in three loaded-voltage bands and build a 2D baseline table:

```text
I0 = I0(v, V)
```

Use VESC `/sensors/core.state.voltage_input` during motion, not open-circuit resting voltage:

```text
High: voltage_input >= 15.8V
Mid:  15.4V <= voltage_input < 15.8V
Low:  15.0V <= voltage_input < 15.4V
```

Recommended collection:
- Stage A: run `2~6m/s` speed hold once in each High / Mid / Low voltage band to build `I0(v,V)`.
- Stage B: run `2->6m/s` accel interval in at least High / Mid bands; add Low only if it is safe for the battery and VESC.
- The low-voltage boundary must respect your battery and VESC safety limits; do not force Low-band data below a safe voltage.

Reference post-processing formulas:

```text
# Stage A: hold current per speed point / voltage band
I0(v_i, V_bin) = mean(I_q)
V_bin_mean(v_i) = mean(voltage_input)

# Stage B: each accel trial
a_fit = slope(linear_fit(t, v_odom))
I_q_mean = mean(state.avg_iq)       # fallback: state.current_motor
V_mean = mean(voltage_input)
I_net = I_q_mean - interp2d(I0_table, v_mid, V_mean)
v_mid = 0.5 * (v0 + v1)

# Calibrate net-current effectiveness
a_fit = I_net * (k0 + k1*(v_mid - v_ref) + k2*(V_mean - V_ref))
```

Recommended feed-forward inversion:

```text
I_cmd = I0(v,V) + a_ref / k_eff(v,V)
k_eff(v,V) = k0 + k1*(v - v_ref) + k2*(V - V_ref)
k_eff(v,V) = clamp(k_eff, k_min, k_max)
```

### 3.2 Stage B: for each speed interval v→v+1, sweep current steps to estimate acceleration

Goal: for each interval (1→2, 2→3, …), start near the Stage-A baseline current and increase by `current_step` until `current_max`, measure time-to-reach and estimate acceleration via $a=\Delta v / t$.

If the baseline current is already known to only hold speed and does not need to be repeated, set `current_start_step_index:=1` so each interval starts from `base_current + current_step`. The default `0` still starts from `base_current`.

Script: `src/f1tenth_system/scripts/longitudinal_calib.py`

Run example:

```bash
ros2 run f1tenth_system longitudinal_calib.py --ros-args \
   -p workflow:=accel_interval \
   -p v_start:=1.0 -p v_end:=8.0 -p dv:=1.0 \
   -p base_current_file:=speed_hold_current_results.txt \
   -p current_step:=3.0 -p current_start_step_index:=1 -p current_max:=80.0 \
   -p vesc_topic:=/sensors/core \
   -p odom_topic:=/odom \
   -p output_path:=speed_interval_accel_results.txt
```

If you must enable RC steering intervention and only run trials on straights:

```bash
ros2 run f1tenth_system longitudinal_calib.py --ros-args \
   -p workflow:=accel_interval \
   -p use_rc_steering:=true \
   -p rc_topic:=/rc/channels \
   -p rc_timeout_sec:=0.25 \
   -p post_turn_settle_sec:=0.8
```

Output:
- `speed_interval_accel_results.txt`: each line is `v0 v1 current_A t_sec accel_mps2 reached`

Notes:
- Before each trial, the script uses **speed mode** to bring the vehicle back to and stabilize at `v0`, then switches to **current mode** to time the acceleration to `v1`.
- Speed feedback comes from `odom.twist.twist.linear.x` (configured by `odom_topic`), and current feedback comes from VESC telemetry (configured by `vesc_topic`).

---

### 3.3 Offline Longitudinal Model Verification

Script: `src/f1tenth_system/scripts/longitudinal_model_verify.py`

The first-pass model intentionally keeps temperature out of the main fit and compares three net-current models:

```text
I_net = I_q_mean - I0(v0)

Model A: a = k * I_net
Model B: a = I_net * (k0 + k1*(v_mid - v_ref))
Model C: a = I_net * (k0 + k1*(v_mid - v_ref) + k2*(V_mean - V_ref))
```

`I_q_mean` prefers `state.avg_iq` and falls back to `state.current_motor`. Temperature is better treated as a later derating/limit diagnostic; voltage and duty are still exported to the verification table to check battery sag and duty saturation. The current verifier still uses the one-dimensional Stage-A `I0(v0)` baseline. Once enough High / Mid / Low voltage-stratified data exists, upgrade the baseline to:

```text
I_net = I_q_mean - I0(v_mid,V)
I_cmd = I0(v,V) + a_ref / k_eff(v,V)
```

CSV-only quick check:

```bash
python3 src/f1tenth_system/scripts/longitudinal_model_verify.py \
   --base speed_hold_current_results.txt \
   --samples speed_interval_accel_samples.csv \
   --valid-v-end 6.0 \
   --out-prefix longitudinal_model_verify_csv
```

Recommended bag-based check with measured `avg_iq / voltage / duty / accel_fit`:

```bash
python3 src/f1tenth_system/scripts/longitudinal_model_verify.py \
   --base speed_hold_current_results.txt \
   --bag /home/nuc/RallyCore/bag/pp_accel_interval2 \
   --valid-v-end 6.0 \
   --model-accel-max 2.5 \
   --out-prefix longitudinal_model_verify_bag
```

Recommended multi-bag voltage-modulation check:

```bash
python3 src/f1tenth_system/scripts/longitudinal_model_verify.py \
   --base speed_hold_current_results.txt \
   --bags /home/nuc/RallyCore/bag/pp_accel_interval1 /home/nuc/RallyCore/bag/pp_accel_interval2 \
   --valid-v-end 6.0 \
   --model-accel-max 2.5 \
   --out-prefix longitudinal_model_verify_pp_accel_1_2_voltage
```

Outputs:
- `*_trials.csv`: per-trial `bag_id`, `bag_path`, `v_mid_mps`, `iq_mean_a`, `i_net_a`, `voltage_mean_v`, `duty_max`, `accel_fit_mps2`, and reject reason.
- `*_model.json`: legacy per-speed/global `k`, `b`, `R2`, `RMSE`, plus aggregate Model A/B/C coefficients, `R2`, `RMSE`, voltage range, duty range, and Model-C-vs-Model-B RMSE improvement.

---

### 3.4 Offline Analysis Prompt (Stage A + B outputs)

Replace the file paths below with your actual generated outputs and send the prompt to your analysis assistant.

```text
You are a vehicle longitudinal dynamics / calibration engineer. Based on the two-stage data collection outputs, build a speed-segmented mapping from motor current to acceleration, and provide a deployable lookup/fit with quality diagnostics.

Input files (replace with real paths):
1) Stage A: speed_hold_current_results.txt
   - Format: speed_mps  mean_current_A  std_current_A  samples
2) Stage B: speed_interval_accel_results.txt
   - Format: v0  v1  current_A  t_sec  accel_mps2  reached
Optional:
- Stage A / Stage B csv_path (if available) for debugging and plotting.

Important collection assumptions:
- No sampling / no trials during turning (steering outside deadzone). After returning straight (inside deadzone), the scripts wait post_turn_settle_sec before resuming sampling/trials.
- Stage B: before each trial, the script stabilizes at v0 in speed mode, then switches to current mode and times acceleration to v1.
- Speed feedback is from /odom.twist.twist.linear.x; current feedback is from VESC telemetry (phase current / iq, etc.).

Tasks:
1) Read & sanity-check
   - Parse both txt files; list Stage-A speed points, range, missing/NaN.
   - Parse all Stage-B trials; compute reached=1 ratio; group by interval (v0->v1) and count samples.
   - Identify obvious outliers: t_sec<=0, invalid accel_mps2, duplicated/disordered current_A, etc.

2) Stage A: baseline current curve
   - Plot I_base(v)=mean_current_A vs v.
   - Smooth/interpolate I_base (e.g., linear interpolation or piecewise fit) and output a queryable function/table.
   - Use std and sample counts to comment on confidence.

3) Stage B: build current→accel model (piecewise by speed interval)
   For each interval (e.g., 1->2, 2->3, ...):
   - Use only reached=1 trials.
   - Scatter plot current_A vs accel_mps2.
   - Provide and compare two modeling options:
     A) Direct fit: a = k*I + b (linear regression)
     B) Subtract baseline: I_eff = I - I_base(v0), fit a = k*I_eff + b or a = k*I_eff.
   - Report k,b, R^2, RMSE, and whether there is nonlinearity/saturation at high current.

4) Deployable output (very important)
   - Generate a monotonic lookup table: for each interval, output a monotonic I->a table (apply monotonic regression if needed).
   - Or output a piecewise-linear parameter table: k,b per interval, with recommended valid ranges.
   - Provide an inversion recipe: given target accel a*, compute the needed current I*.
   - Recommend constraints: I_max, minimum usable current, timeout/unreached handling.

5) Quality diagnostics & next iteration suggestions
   - Which intervals lack data or are too noisy (recommend more repeats or longer reset/stable time).
   - Which current levels frequently timeout (suggest adjusting current_max, trial_timeout_sec, or dv).
   - Recommend practical ranges for post_turn_settle_sec, speed_tolerance, stable_required_sec.

Output requirements:
- List the key plots to generate and what each plot shows.
- Provide the final recommended tables/parameters clearly.
- If interpolation is used, specify method and boundary handling (what to do outside the speed range).
```

---

## IMPORTANT NOTE

This document is currently maintained mainly for the “two-stage workflow” in Section 3.
The old `speed_hold_current_logger.py`, `speed_interval_accel_sweep.py`, `manual_steer_speed_stages.py`, and `manual_steer_current_sweep_stages.py` scripts have been removed; their useful behavior is covered by `longitudinal_calib.py` with `workflow:=speed_hold/accel_interval/decel_current`.
Content below is supplemental recording / analysis guidance. When in doubt, the code and Section 3 take precedence.

---

## Appendix A) Rosbag Recording (Suggested)

Your current command:

```bash
ros2 bag record -o manual_control1 \
  /odom \
  /drive \
  /imu \
  /livox/imu \
  /sensors/servo_position_command \
  /ackermann_cmd \
  /sensors/core \
  /rc/channels \
  /calib/ackermann_cmd
```

Suggested additional topics (priority order):

1) **Actuator commands (VESC-side)**: to confirm what is actually executed (speed/current/duty) and detect overrides.
- `/commands/motor/speed`
- `/commands/motor/current`
- `/commands/motor/duty_cycle`
- `/commands/servo/position`

2) **TF** (if post-processing needs frame alignment or visualization)
- `/tf`
- `/tf_static`

3) **Clock** (if sim / /clock is used)
- `/clock`

---

## Appendix B) Rosbag Analysis Prompt

Replace `{bag_dir}` with your bag path.

```text
You are a vehicle longitudinal dynamics / calibration engineer. Based on one rosbag recorded during “manual steering intervention + multi-stage speed hold”, build a speed-binned longitudinal model and evaluate the relationship between motor current and acceleration.

Context:
- Upstream publishes /calib/ackermann_cmd (AckermannDriveStamped). In this scheme, drive.jerk=0.0 is used for speed mode.
- When RC steering is within deadzone: treat as straight (steering=0).
- When RC steering is outside deadzone: treat as turning (steering=RC).
- Motor telemetry is from /sensors/core (VescStateStamped), including speed, avg_iq, etc.

Data:
- rosbag directory: {bag_dir}

Please do:
1) Data inspection: list topics, message counts, approximate rates; check timestamps monotonicity and long gaps.
2) Alignment & derived quantities:
   - Cross-check vehicle speed from /sensors/core/state/speed vs /odom.twist.twist.linear.x (explain scaling/latency differences).
   - Compute acceleration from /odom speed (describe differencing, smoothing/low-pass, and any delay compensation).
3) Straight/turning segmentation:
   - Prefer using /rc/channels steering channel with the same deadzone rule as joystick_control_v2.
   - Backup: threshold on /calib/ackermann_cmd.drive.steering_angle.
4) Per-stage statistics:
   - Segment by target speed stages (or cluster by speed bins).
   - For each stage: mean speed, speed variance, mean current (or avg_iq), mean accel, accel noise level.
5) Modeling:
   - Provide at least one deployable model, e.g., a = k(v)*I + b(v) with piecewise-constant k,b by speed bins, or a lookup-table policy.
   - Report R^2 / RMSE and discuss major error sources (turning disturbance, slope, battery voltage, latency).
6) Quality diagnostics:
   - Check contamination: steering input present during supposed “straight” segments.
   - Check saturation, deadzone, or speed-loop oscillation.
7) Next collection suggestions:
   - Minimum straight duration per stage and recommended speed range.
   - Missing topics that would improve analysis.
8) Consider the case where duty cycle saturates at maximum, and propose calibration / control changes for that regime.

Output requirements:
- Provide the final recommended mapping (formula/parameters/table) and clearly specify all filtering and selection rules (thresholds, windows, filter params).
```

---

## Appendix C) How to Hook `/calib/ackermann_cmd` Into Your Stack

These nodes only publish `/calib/ackermann_cmd`. If your control chain uses a different command topic, use launch remap without modifying upstream/downstream code.

Example (remap to the actual command input):

```bash
ros2 run f1tenth_system longitudinal_calib.py \
  --ros-args -p workflow:=speed_hold \
  -r /calib/ackermann_cmd:=/ackermann_cmd
```
