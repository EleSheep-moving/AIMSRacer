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

For an interactive Gazebo window plus RViz2, run this from an X11 desktop. It creates a timestamped fixture under `results/`, prepares the solver cache, enables MPCC automatically, and completes one 8-shaped lap:

```sh
XAUTHORITY=/run/user/$(id -u)/gdm/Xauthority
docker run --rm --net=host --gpus all \
  -e ROS_DOMAIN_ID=42 -e DISPLAY=:0 -e QT_X11_NO_MITSHM=1 -e XAUTHORITY=/root/.Xauthority -e AIMS_MPCC_CACHE_DIR=/results/ccache \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw -v "$XAUTHORITY:/root/.Xauthority:ro" \
  -v "$PWD/src/aims_gazebo_sim/results:/results" \
  aimsracer-mpcc:gazebo-local bash -lc 'source /opt/ros/humble/setup.bash; source /ws/install/setup.bash; ros2 run aims_gazebo_sim run_visual'
```

The initial invocation compiles the reference-specific solver cache. Later runs of the same reference reuse `/results/ccache`.

The acceptance result proves that this ROS graph can close the loop with the configured Gazebo vehicle, Gazebo clock, physical wheel contacts, VESC conversion, and source MPCC runtime. It does not calibrate real tires, drivetrain dynamics, localization, or real-car safety.

The vehicle SDF follows the interface and parameter layout of Gazebo's Apache-2.0 Ackermann example: <https://github.com/gazebosim/gz-sim/blob/gz-sim6/examples/worlds/ackermann_steering.sdf>.
