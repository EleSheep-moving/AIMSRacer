# AIMSRacer Gazebo MPCC validation

This package runs the existing ROS control path in Gazebo Fortress:

`aims_mpcc -> /drive -> RC selector -> /ackermann_cmd -> vesc_ackermann -> VESC-to-Gazebo bridge -> Gazebo Ackermann/contact vehicle -> physical odometry -> /odometry/filtered`.

The model origin is the rear axle, so the bridged odometry follows the MPCC state convention. The model uses a synthetic `0.36 m` wheelbase and `0.26 m` track width. Those values are intentionally separate from the unverified real-vehicle profile.

Build and run only inside the local Docker image:

```sh
docker build -f src/aims_gazebo_sim/docker/Dockerfile -t aimsracer-mpcc:gazebo-local .
docker run --rm --net=host -e ROS_DOMAIN_ID=42 -v "$PWD/src/aims_gazebo_sim/results:/results" aimsracer-mpcc:gazebo-local \
  bash -lc 'source /opt/ros/humble/setup.bash && source /ws/install/setup.bash && ros2 run aims_gazebo_sim run_acceptance /results/circle'
```

Use `--track figure_eight --radius 6 --waist-ratio 0.3` to exercise the wider double-lobe reference. `results/` and the Dockerfile are machine-local and ignored by Git.

The acceptance result proves that this ROS graph can close the loop with the configured Gazebo vehicle, Gazebo clock, physical wheel contacts, VESC conversion, and source MPCC runtime. It does not calibrate real tires, drivetrain dynamics, localization, or real-car safety.

The vehicle SDF follows the interface and parameter layout of Gazebo's Apache-2.0 Ackermann example: <https://github.com/gazebosim/gz-sim/blob/gz-sim6/examples/worlds/ackermann_steering.sdf>.
