# AIMSRacer

A ROS 2 Humble autonomous-racing stack and experimental RoboRacer vehicle at
PolyU AIMS Lab, covering localization, calibration, perception and MPCC control.
The vehicle uses an Orin NX 16 GB, KKPIT ZQR 1/7-scale chassis, Livox MID360,
ZED 2i and RadioMaster Pocket ELRS remote.

## Start here

| Task | Guide |
| --- | --- |
| Install the Orin vehicle computer | [Orin installation](docs/installation.md) |
| Install or run an Orin / NUC vehicle computer | [Vehicle-computer deployment](docs/deployment/README.md) |
| Run numerical or Gazebo MPCC simulation on a workstation | [`mpcc-sim` simulation guide](https://github.com/EleSheep-moving/AIMSRacer/blob/mpcc-sim/docs/simulation/README.md) |
| Understand frames, sensor fusion and command routing | [Architecture](docs/architecture.md) |
| Start V2 or V3 | [Vehicle bringup](docs/operations/bringup.md) |
| Record a bag | [Recording](docs/operations/recording.md) |
| Use a saved PGO map for MPCC | [Known-map workflow](docs/operations/known-map-mpcc.md) |
| Prepare and run MPCC | [MPCC usage](src/controller/docs/usage.md) |
| Learn the MPCC code | [Implementation](src/controller/docs/implementation.md) |
| Check readiness before an autonomous lap | [Vehicle checklist](docs/operations/vehicle-checklist.md) |
| Collect calibration data | [中文](src/aims_racer_system/docs/calibration.md) / [English](src/aims_racer_system/docs/calibration.en.md) |

V2 supplies LiDAR localization, rear-axle EKF and RC/VESC control. V3 adds ZED
perception. MPCC currently follows a recorded closed lap from disk; live local
trajectory topic input is not implemented. Its low-speed prototype still requires
measured geometry and vehicle validation.

## Measured vehicle response

The 2026-09-28 V2 calibration bag was recorded on the Orin NX/MID360 car with
the current VESC gain of 3465 ERPM/(m/s). All control messages used **speed
mode**. These are offline observations, not results from an MPCC-controlled
lap. Most moving samples used about 0.85–1.15 m/s wheel speed.

| Test and conditions | Observed response |
| --- | --- |
| Speed scale: 412 approximately straight samples at 0.4–1.4 m/s | Source-time-aligned LIO/wheel forward-speed ratio median **1.005**; retain the 3465 gain. |
| Straight speed-mode starts: 11 events from rest | The 200 Hz raw IMU detected sustained forward acceleration after **45–60 ms** (median 52 ms). A separate command-to-wheel fit gave about **40 ms delay + 0.16 s response constant**; MPCC does not model these separately. |
| Steering: ten neutral-to-±0.475 rad steps at 0.88–1.03 m/s | Raw-IMU yaw rate first responded after median **52 ms**; command-to-50%/90% yaw response was **109/159 ms**. |

The steering times describe command-to-vehicle **yaw response**, not measured
front-wheel or servo motion: `/sensors/servo_position_command` echoes the
command and no steering-angle encoder was recorded. Fitting yaw rate divided
by measured forward speed to the MPCC's single-lag steering model gave
`steering_tau: 0.0806 s` using wheel speed or `0.0798 s` using LIO speed, so
the configured **0.08 s** is retained for this roughly 1 m/s speed-mode region.
It absorbs unmodelled command-to-yaw delay; it is neither the first-response
delay nor a physical servo constant. `understeer_coefficient: 0` remains a
low-speed assumption, not measured zero slip. See the
[speed-mode calibration](docs/reports/2026-09-28-speed-mode-calibration.md) for
selection, timing and estimator-age limits, and the
[earlier mixed-mode analysis](docs/reports/2026-09-28-vehicle-response.md) for
the original speed-scale correction. Both bags are local artifacts, not part
of the source checkout.

One closed lap has also been recovered from the mapping CSV and matched to
the saved PGO map poses. The configured footprint is 620 × 320 mm, with 100 mm
behind the rear axle, and the course model is 1.0 m wide with a centered path.
It remains a **candidate reference** until the physical envelope, track clearance
and live map relocalization are checked; see the
[map-reference report](docs/reports/2026-09-28-map-reference.md). The initial
MPCC cruise target is now **1.0 m/s**, consistent with the bag's operating
speed and this candidate's curvature under the configured 1 m/s² lateral
acceleration limit.

## Documentation ownership

System contracts and whole-vehicle procedures live in `docs/`. Package-specific
usage and implementation live beside their code. Each topic has one maintained
body; other pages link to it. Dated [reports](docs/reports/README.md) retain their
experiment conditions and are not current operating instructions. Submodule
sources and documentation are maintained upstream and must not be edited locally.

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
