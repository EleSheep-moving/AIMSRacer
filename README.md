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
| Prepare and run MPCC | [MPCC usage](src/controller/docs/usage.md) |
| Learn the MPCC code | [Implementation](src/controller/docs/implementation.md) |
| Check readiness before an autonomous lap | [Vehicle checklist](docs/operations/vehicle-checklist.md) |
| Collect calibration data | [中文](src/aims_racer_system/docs/calibration.md) / [English](src/aims_racer_system/docs/calibration.en.md) |

V2 supplies LiDAR localization, rear-axle EKF and RC/VESC control. V3 adds ZED
perception. MPCC currently follows a recorded closed lap from disk; live local
trajectory topic input is not implemented. Its low-speed prototype still requires
measured geometry and vehicle validation.

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
