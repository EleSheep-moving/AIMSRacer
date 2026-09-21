# AIMSRacer system integration

This package provides vehicle launch files, configuration, rear-axle adapters,
and calibration tools. V2 supplies LiDAR/EKF and RC/VESC; V3 adds ZED perception.

- [Vehicle bringup](../../docs/operations/bringup.md)
- [Frames, sensor fusion and control chain](../../docs/architecture.md)
- [Recording](../../docs/operations/recording.md)
- Calibration: [中文](docs/calibration.md) / [English](docs/calibration.en.md)

## Build and verify

After completing [installation](../../docs/installation.md), run from the workspace root:

```bash
/usr/bin/python3 -m colcon build --symlink-install --packages-select aims_racer_system
source install/setup.bash
ROS_DOMAIN_ID=219 ROS_LOCALHOST_ONLY=1 OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 \
  /usr/bin/python3 -m pytest -q src/aims_racer_system/tests
```

Tests require the EKF and VESC converter packages already built. They start
adapters, EKF and converter with synthetic input in an isolated ROS domain; they
do not exercise FAST-LIO scan matching or actual vehicle motion. Use a test
terminal with DDS settings compatible with localhost and distinct from the
vehicle's ROS domain.
