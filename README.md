# 🏎️ AIMSRacer

## 🌟 Overview
AIMSRacer is a ROS 2 autonomous-racing stack and experimental vehicle platform for RoboRacer competition and high-performance autonomous-driving research. It supports reproducible work on state estimation, LiDAR/stereo perception, vehicle-dynamics calibration, trajectory tracking, motion planning, and learning-based racing.

The hardware setup and basic software framework were developed during my time as a Research Assistant at [ZJU FAST Lab](https://github.com/ZJU-FAST-Lab). I am deeply grateful to the wonderful people at ZJU FAST Lab for their invaluable support and guidance. The platform is now maintained in PolyU AIMS Lab as a dedicated RoboRacer vehicle for competition development and racing-focused research papers.

## 📦 Installation

I provide an installation guide [here](install.md) 📖, for ROS2 Humble on Ubuntu 22.04.

⚠️ May have some issues with the installation guide, please let me know if you have any questions.

## 🔨 Modification & Customization

### 🛠️ RoboRacer Platform
**💻 Computing Platform:** NVIDIA Jetson Orin NX 16 GB

**🏎️ Chassis:** KKPIT ZQR "Zhuque" 1/7-scale electric 4WD on-road chassis

**🔦 LiDAR:** Livox Mid-360 (main sensor)

**📷 Stereo Camera:** ZED 2i (CUDA/GPU-accelerated depth and perception)

**🧭 IMU:** fdilink Deta10 (optional, yaw estimation)

**🎮 Remote Controller:** RadioMaster Pocket ELRS version (much better than XBOX series controller) 

### ⚠️ KKPIT ZQR "Zhuque" Reverse-Transition Limitation

On this chassis, the initial transient when reverse is engaged produces an approximately **2 g vertical (Z-axis) vibration/shock**. This event can violate FAST-LIO2's IMU motion assumptions and cause the estimator to fail. Treat this as a chassis-specific operating limitation: avoid relying on FAST-LIO2 through a reverse transition, and stop or reinitialize localization before resuming autonomous operation if the event occurs.

### ⚙️ RoboRacer Drive Interface
The modified VESC interface is based on the VESC interface provided by Veddar VESC Interface. 

**Modifications:**
- ✨ Fixed odometry computation to eliminate speed delay when decelerating from high velocities to stop

### 🎛️ ackermann_mux
- ✅ Added scripts to process messages from ELRS driver and publish to `/teleop`

### 🗺️ Nav2 Parameters
- 📚 Configured based on [QUTMS_Driverless](https://github.com/QUT-Motorsport/QUTMS_Driverless)

## 🚀 Main Launch Files

### 🛡️ Vehicle-Safe ROS Network

Before vehicle bringup, keep the ROS2 control graph on localhost so Wi-Fi loss cannot break local control traffic:

```bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_LOCALHOST_ONLY=0
export CYCLONEDDS_URI=file:///home/nuc/cyclonedds.xml
```

`/home/nuc/cyclonedds.xml` should bind CycloneDDS to `lo`. The Livox driver still communicates with the Mid-360 on the wired `192.168.1.x` network through its own UDP sockets.

### ⚡ Quick Start (Recommended)
```bash
# 🔌 Hardware bringup (V3 - FAST-LIO2/EKF + ZED 2i neural-depth perception)
ros2 launch aims_racer_system base_orin_livox_bringup_v3.launch.py

# LiDAR-only alternative: V2 - FAST-LIO2 with EKF fusion
ros2 launch aims_racer_system base_orin_livox_bringup_v2.launch.py

# 🗺️ SLAM/Localization
ros2 launch aims_racer_system slam.launch.py

# 🧭 Navigation
ros2 launch aims_racer_system nav.launch.py
```

### 🏗️ Hardware Bringup Versions
| Version | Launch File | LIO Backend | Features |
|---------|-------------|-------------|----------|
| **🚀 V3 (Perception)** | `base_orin_livox_bringup_v3.launch.py` | **FAST-LIO2 + EKF** | V2 control/localization • ZED 2i `NEURAL_LIGHT` RGB/depth/IMU • ZED tracking and dynamic TF disabled |
| **✨ V2 (Recommended)** | `base_orin_livox_bringup_v2.launch.py` | **FAST-LIO2** | 🎯 Integrated control (joystick_v2) • 🔋 Speed/current/duty modes • 🤖 EKF fusion |
| 📦 V1 (Legacy) | `base_orin_livox_bringup.launch.py` | **FAST-LIO2** | 🔀 Separate mux node • 🏛️ Traditional architecture • 🤖 EKF fusion |

**⚙️ V2 Advantages:** Single control node • Built-in arbitration • Current control support • Easier debugging • Proven odometry accuracy

**📷 V3 ZED policy:** ZED 2i runs at 30 Hz with `NEURAL_LIGHT`, RGB, depth, and IMU telemetry. ZED positional tracking, map/odom TF, object detection, body tracking, and point-cloud publication are disabled by default so FAST-LIO2/EKF remain the only vehicle-localization chain. After measuring the camera installation, provide its transform explicitly, for example:

```bash
ros2 launch aims_racer_system base_orin_livox_bringup_v3.launch.py \
  zed_publish_base_tf:=true \
  zed_tf_x:=<x_m> zed_tf_y:=<y_m> zed_tf_z:=<z_m> \
  zed_tf_roll:=<roll_rad> zed_tf_pitch:=<pitch_rad> zed_tf_yaw:=<yaw_rad>
```

📚 **Calibration docs (maintained):** [src/aims_racer_system/scripts/README_EN.md](src/aims_racer_system/scripts/README_EN.md)

## 🔧 Calibration & Tuning

✅ The longitudinal calibration workflow is documented and maintained in:

- [src/aims_racer_system/scripts/README_EN.md](src/aims_racer_system/scripts/README_EN.md)

It includes:
- `/calib/ackermann_cmd` jerk convention (speed/current)
- `/calib/current_trajectory`, `/calib/lookahead_point`, and `/calib/status_text` RViz topics
- Localization-based (Pure Pursuit) and RC-intervention data collection
- Recommended **three-stage PP workflow** (Stage A speed-hold current + Stage B interval accel sweep + Stage C decel sweep)


### ⚡ Longitudinal Calibration (Recommended)

#### Option A: Localization-based (Pure Pursuit)

This option requires stable odometry and a relatively large open area/track (long straights and safe turn radius), because the vehicle needs enough space to run repeatable loops.

See [src/aims_racer_system/scripts/README_EN.md](src/aims_racer_system/scripts/README_EN.md) → Localization-based (Pure Pursuit).

```bash
ros2 run aims_racer_system longitudinal_calib.py --ros-args \
    -p workflow:=pp_speed_hold \
    -p speeds:="[1,2,3,4,5,6,7,8]" \
    -p hold_time_sec:=10.0 \
    -p output_path:=speed_hold_current_results.txt

ros2 run aims_racer_system longitudinal_calib.py --ros-args \
    -p workflow:=pp_accel_interval \
    -p v_start:=1.0 -p v_end:=8.0 -p dv:=1.0 \
    -p base_current_file:=speed_hold_current_results.txt \
    -p current_step:=3.0 -p current_max:=80.0 \
    -p output_path:=speed_interval_accel_results.txt

ros2 run aims_racer_system longitudinal_calib.py --ros-args \
    -p workflow:=pp_decel_current \
    -p v_start:=3.0 -p v_end:=8.0 -p dv:=1.0 \
    -p decel_low_speed:=1.0 \
    -p decel_current_step:=3.0 -p decel_current_min:=-20.0 \
    -p output_path:=decel_current_sweep_results.txt
```

#### Option B (Recommended): RC-intervention (No localization) — Two-stage workflow

This is the recommended workflow when odometry/localization is unstable, or when you do not have a large enough calibration track.

Stage A (speed hold → mean current):

```bash
ros2 run aims_racer_system longitudinal_calib.py --ros-args \
    -p workflow:=speed_hold \
    -p speeds:="[1,2,3,4,5,6,7,8]" \
    -p hold_time_sec:=10.0 \
    -p use_rc_steering:=false \
    -p vesc_topic:=/sensors/core \
    -p output_path:=speed_hold_current_results.txt
```

Stage B (per speed interval → accel vs current, speed from `/odom`):

```bash
ros2 run aims_racer_system longitudinal_calib.py --ros-args \
    -p workflow:=accel_interval \
    -p v_start:=1.0 -p v_end:=8.0 -p dv:=1.0 \
    -p base_current_file:=speed_hold_current_results.txt \
    -p current_step:=3.0 -p current_max:=80.0 \
    -p use_rc_steering:=false \
    -p vesc_topic:=/sensors/core \
    -p odom_topic:=/odom \
    -p output_path:=speed_interval_accel_results.txt
```

If you need RC steering intervention during data collection, enable it and the scripts will:
- NOT sample / NOT run trials during turning (out of deadzone)
- Wait `post_turn_settle_sec` after returning straight before resuming

```bash
ros2 run aims_racer_system longitudinal_calib.py --ros-args \
    -p workflow:=speed_hold \
    -p use_rc_steering:=true -p rc_topic:=/rc/channels -p rc_timeout_sec:=0.25 \
    -p post_turn_settle_sec:=0.8

ros2 run aims_racer_system longitudinal_calib.py --ros-args \
    -p workflow:=accel_interval \
    -p use_rc_steering:=true -p rc_topic:=/rc/channels -p rc_timeout_sec:=0.25 \
    -p post_turn_settle_sec:=0.8
```

📊 Analysis prompt templates live in: [src/aims_racer_system/scripts/README_EN.md](src/aims_racer_system/scripts/README_EN.md).

## 🏗️ Architecture: V1 vs V2

### 📐 System Architecture Diagrams

**✨ V2 Architecture (Integrated Control):**
```
┌─────────────┐
│  RC Remote  │──┐
└─────────────┘  │
┌─────────────┐  │
│ Nav2 /drive │──┤
└─────────────┘  │    ┌──────────────────┐    ┌──────────┐
┌─────────────┐  ├───▶│ joystick_v2.py   │───▶│   VESC   │
│  PP Tuner   │──┤    │ (Integrated Mux) │    │  Driver  │
└─────────────┘  │    └──────────────────┘    └──────────┘
┌─────────────┐  │
│ Calibration │──┘
└─────────────┘
```

**📦 V1 Architecture (Mux-based):**
```
┌─────────────┐    ┌──────────────┐    ┌─────────────┐    ┌──────────┐
│  RC Remote  │───▶│ joystick.py  │───▶│  ackermann  │───▶│   VESC   │
└─────────────┘    └──────────────┘    │     mux     │    │  Driver  │
┌─────────────┐                        │  (Priority) │    └──────────┘
│ Nav2 /drive │───────────────────────▶│             │
└─────────────┘                        └─────────────┘
```

### 🔄 Feature Comparison

| Feature | V2 ✨ (Recommended) | V1 📦 (Legacy) |
|---------|----|----|
| 🎮 **Control Nodes** | 1 (joystick_v2) | 2 (joystick + mux) |
| ⚡ **ESC Modes** | Speed/Current/Duty | Speed only |
| 🔬 **Calibration Support** | ✅ Built-in `/calib/*` | ❌ Not supported |
| 🎚️ **Command Arbitration** | Built-in logic | External mux node |
| 🐛 **Debugging** | Easier (single node) | Complex (multiple nodes) |
| 📡 **Input Topics** | `/drive`, `/calib/ackermann_cmd` | `/teleop`, `/drive` |
| 📤 **Output Topic** | `/ackermann_drive` | `/ackermann_drive` |
| 🎛️ **Mode Switching** | RC channel 10 | YAML config |

### 🎯 When to Use Each Version

**Use V2 if:**
- ✅ Running calibration experiments
- ✅ Need current/duty control modes
- ✅ Want simplified debugging
- ✅ Prefer integrated control logic

**Use V1 if:**
- 📦 Only need Nav2 navigation (speed control)
- 📦 Require strict priority-based arbitration
- 📦 Legacy system compatibility

### 📊 V2 Control Flow Details

```python
# joystick_v2.py simplified logic
def control_logic():
    if rc_channel_10 == HIGH:
        mode = "teleop"  # RC manual control
    elif calibration_active:
        mode = "calib"   # Calibration mode
    else:
        mode = "nav"     # Navigation mode
    
    if esc_mode == "CURRENT":
        command.current = target_current  # For calibration
    elif esc_mode == "SPEED":
        command.speed = target_speed      # For navigation
    elif esc_mode == "DUTY":
        command.duty_cycle = target_duty  # For advanced control
```

### 🔌 ROS2 Topic Interface

**V2 Subscribed Topics:**
- `/drive` (AckermannDriveStamped) - Navigation commands
- `/calib/ackermann_cmd` (AckermannDriveStamped) - Calibration commands
- `/joy` (Joy) - RC remote input

**V2 Published Topics:**
- `/ackermann_drive` (AckermannDriveStamped) - Final command to VESC

**Message Fields Usage:**
```yaml
AckermannDriveStamped:
  drive:
    steering_angle: float    # Steering angle in radians
    speed: float             # Target speed (m/s) for SPEED mode
    acceleration: float      # Current (A) for CURRENT mode
    jerk: float             # Mode flag: 0=teleop, 1=curve, 2=straight
    steering_angle_velocity: # Unused
```

---

## 🙏 Acknowledgement
This project would not be possible without the use of multiple great open-sourced code bases as listed below:

- 🏎️ [ForzaETH Race Stack](https://github.com/ForzaETH/race_stack)
- 🏁 [QUTMS_Driverless](https://github.com/QUT-Motorsport/QUTMS_Driverless)
- 🎯 [Original upstream system package](https://github.com/f1tenth/f1tenth_system)
- 📡 [ros2_crsf_receiver](https://github.com/AndreyTulyakov/ros2_crsf_receiver.git)
- 🔀 [ackermann_mux](https://github.com/z1047941150/ackermann_mux.git)
- ⚡ [Veddar VESC Interface](https://github.com/f1tenth/vesc)
- 🗺️ [FAST-LIO2_ROS2](https://github.com/liangheming/FASTLIO2_ROS2.git)

##### 🏛️ Hardware and basic software were developed at FAST Lab, Zhejiang University.
##### 🎓 Currently pursuing MPhil at PolyU AIMS Lab, with ongoing development in progress.

---

## 🚀 Future Work
- 🏁 Add racing-line estimation and track-boundary perception
- 🛣️ Add racing-aware trajectory planning and optimization
- ✅ ~~Add current&acceleration calibration and control module~~ (Completed ✨)
- 🎮 Use a racing simulator, such as Isaac Lab or a dedicated track simulator
- 🤖 Use RL for racing-policy and lap-time optimization
- 🗺️ Integrate additional LIO backends (LVI-SAM, DLIO, etc.)

