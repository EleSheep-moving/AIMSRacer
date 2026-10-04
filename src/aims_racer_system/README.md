# AIMSRacer system integration

This package provides vehicle launch files, configuration, rear-axle adapters,
and calibration tools. V2 supplies LiDAR/EKF and RC/VESC; V3 adds ZED perception.

The package uses `ament_cmake` with `ament_cmake_python`: the high-rate
`imu_to_rear_axle` adapter is C++/Eigen; the remaining adapters and calibration
tools retain their Python executables. IMU topics, parameters, source timestamps,
gravity-retaining specific force, and unavailable-data flags are preserved.

- [Vehicle bringup](../../docs/operations/bringup.md)
- [Frames, sensor fusion and control chain](../../docs/architecture.md)
- [Recording](../../docs/operations/recording.md)
- Calibration: [中文](docs/calibration.md) / [English](docs/calibration.en.md)

## LIO output budget

FAST-LIO publishes odometry before optional output work. Point-cloud outputs check
their enable switch and subscriber count before allocating, transforming or
serializing points. The shipped RViz configurations disable point-cloud displays.
LIO path publication is disabled by default; if enabled it is limited to 1 Hz and
1000 poses. Cloud publisher queues retain only two messages instead of 10000.

`fastlio_rear.yaml` retains `publish_body_cloud: true` for ICP/PGO, RViz,
and scan/map consistency diagnostics at the original scan rate and density.
The consistency worker pairs `/fastlio2/body_cloud` with `/fastlio2/lio_odom`
by identical scan-end timestamps, samples at most 2000 finite points at 2 Hz,
and transforms them with the raw LIO pose followed by the held map correction.
It never uses EKF TF to reconstruct the original LIO alignment and never gates
map TF or MPCC.

`publish_world_cloud: false` is the default. Host launch files remap the optional
world output to `/fastlio2/visualization/world_cloud`; it is for inspecting a scan
already transformed by raw LIO, with no localization/control subscriber. When
enabled in the main-repository LIO YAML at startup, it is capped at 5 Hz and
2000 points. The scan timestamp is preserved. Output parameters are read-only
at startup; restart LIO to apply changes. Neither the estimator input nor its
internal map depends on publishing these clouds. No relay or duplicate alias
is started for the renamed output.

For odometry alone, both cloud outputs may be disabled in the LIO YAML. Keep
body cloud enabled for ICP/PGO and consistency diagnostics. RViz can display
body cloud through the public EKF-owned TF tree; that view uses the EKF pose,
while the optional world view uses the raw LIO pose. See the centralized
[topic contracts](../../docs/architecture.md#topics-and-compensation) for
publishers, consumers, frames, timestamps and diagnostic-only outputs.
The [LIO/EKF findings](../../docs/reports/2026-10-04-lio-delay.md) record the
corrected matching Jacobian, remaining iteration-cap wiring issue and the
reason for the current 2 s EKF history window. Rebuild the FAST-LIO fork to
deploy its source changes; publishing a commit does not update a local binary.

## Build and verify

After completing [installation](../../docs/installation.md), run from the workspace root:

```bash
/usr/bin/python3 -m colcon build --symlink-install --packages-select aims_racer_system
source install/setup.bash
```

Local integration verification requires the EKF and VESC converter packages already built. It starts
adapters, EKF and converter with synthetic input in an isolated ROS domain; they
do not exercise FAST-LIO scan matching or actual vehicle motion. Use a test
terminal with DDS settings compatible with localhost and distinct from the
vehicle's ROS domain.

Local C++ tests compared 244 events captured from the former Python implementation,
including rotated mounting, interpolated LIO anchors, zero/invalid covariance,
gyro gaps, out-of-order timestamps and expired attitude. The additional C++
test sources, fixtures and benchmark program remain local and are not included
in this publication. The benchmark measures numerical helpers only; it does
not measure ROS/DDS or the live process CPU load.
In a local Release-build comparison with 400 gyro samples at 200 Hz, propagating
100 ms of attitude history took about 0.00147 ms median in C++, versus 1.07 ms
in the former Python helper. This is a numerical-helper benchmark, not a
measurement of the entire node's CPU reduction.
