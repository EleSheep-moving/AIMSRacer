# AIMSRacer MPCC numerical ROS simulation

`aims_mpcc_sim` verifies the deployed ROS control path with an independent,
real-time lagged kinematic bicycle plant:

```text
aims_mpcc -> /drive -> joystick selector -> /ackermann_cmd -> VESC converter
     ^                                                          |
     +-------------- /odometry/filtered <- numerical plant <---+
```

The plant publishes rear-axle `odom/base_link` odometry and the V2 RC profile.
It consumes the actual ERPM and servo outputs of `vesc_ackermann`; it does not
call the MPCC dynamics or solver directly.

Run the local Docker validation from the repository root:

```bash
docker build -f src/aims_mpcc_sim/docker/Dockerfile -t aimsracer-mpcc:ros-sim-local .
mkdir -p src/aims_mpcc_sim/results
docker run --rm --network none -e ROS_DOMAIN_ID=83 \
  -v "$PWD/src/aims_mpcc_sim/results:/results" aimsracer-mpcc:ros-sim-local \
  bash -lc 'source /opt/ros/humble/setup.bash && source /ws/install/setup.bash && \
  ros2 run aims_mpcc_sim run_acceptance /results/acceptance-$(date -u +%Y%m%dT%H%M%SZ)'
```

Each output directory contains `summary.json`, `trajectory.csv`, `tracking.png`,
the prepared reference, controller telemetry and the launch log. The runner
requires a COMPLETE lap, zero solve deadline misses, RMS cross-track error at
most 0.10 m, maximum cross-track error at most 0.25 m and finish error at most
0.20 m.

This validates the ROS topic/process/control path and the specified numerical
model. It does not validate FAST-LIO/EKF runtime behavior, tire forces, actuator
calibration, Orin timing, braking distance or real-vehicle safety.
