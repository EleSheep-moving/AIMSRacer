# AIMSRacer Gazebo MPCC validation

This package runs the existing ROS control path in Gazebo Fortress:

`aims_mpcc -> /drive -> RC selector -> /ackermann_cmd -> vesc_ackermann -> VESC-to-Gazebo bridge -> Gazebo Ackermann/contact vehicle -> physical odometry -> /odometry/filtered`.

The model origin is the rear axle, so the bridged odometry follows the MPCC state convention. The model uses a synthetic `0.36 m` wheelbase and `0.26 m` track width. Those values are intentionally separate from the unverified real-vehicle profile.

## Version baseline

The validated container is Ubuntu 22.04.5, ROS 2 Humble, and Gazebo Fortress
(Ignition Gazebo `6.18.0`). Its ROS-Gazebo packages are `ros_gz 0.244.26`:
`ros-humble-ros-gz 0.244.26-1jammy.20260908.015605`,
`ros-humble-ros-gz-bridge 0.244.26-1jammy.20260907.225444`, and
`ros-humble-ros-gz-sim 0.244.26-1jammy.20260907.235058`. The MPCC Python
runtime uses CasADi `3.7.2`, NumPy `1.26.4`, SciPy `1.15.3`, and Matplotlib
`3.8.4`.

## Native source build (without Docker)

Use Ubuntu 22.04 with ROS 2 Humble. Fortress is the ROS-supported Gazebo pair
for Humble; do not mix the Humble `ros-gz` packages with a newer Gazebo release.
After installing ROS Humble Desktop and initializing `rosdep`, clone the project
with the VESC and CRSF submodules, then install only the dependencies used by
this simulation:

```sh
git clone --recurse-submodules git@github.com:EleSheep-moving/AIMSRacer.git AIMSRacer
cd AIMSRacer

sudo apt update
sudo apt install \
  ros-humble-ros-gz ros-humble-ros-gz-sim ros-humble-ros-gz-bridge ros-humble-rviz2 \
  ccache gcc python3-numpy python3-scipy python3-yaml python3-matplotlib python3-pip
/usr/bin/python3 -m pip install --user --no-deps casadi==3.7.2

source /opt/ros/humble/setup.bash
rosdep install --ignore-src -r -y --rosdistro humble \
  --from-paths src/controller src/aims_mpcc_sim src/aims_gazebo_sim \
  src/ackermann_mux src/ros2_crsf_receiver/crsf_receiver_msg \
  src/vesc/vesc_msgs src/vesc/vesc_ackermann
colcon build --symlink-install --packages-up-to aims_gazebo_sim
source install/setup.bash

export AIMS_MPCC_CACHE_DIR="$PWD/.cache/aims_mpcc"
mkdir -p "$AIMS_MPCC_CACHE_DIR"
ros2 run aims_gazebo_sim run_visual --output-root "$PWD/src/aims_gazebo_sim/results"
```

Run these commands as the user who installed CasADi, with Python user-site
packages enabled. The repository-wide installation guide provides the ROS
Humble and `rosdep` bootstrap steps: [`docs/installation.md`](../../docs/installation.md).

## Docker build and run

```sh
docker build -f src/aims_gazebo_sim/docker/Dockerfile -t aimsracer-mpcc:gazebo-local .
docker run --rm --net=host -e ROS_DOMAIN_ID=42 -v "$PWD/src/aims_gazebo_sim/results:/results" aimsracer-mpcc:gazebo-local \
  bash -lc 'source /opt/ros/humble/setup.bash && source /ws/install/setup.bash && ros2 run aims_gazebo_sim run_acceptance /results/circle'
```

Use `--track figure_eight --radius 6 --waist-ratio 0.3` to exercise the wider double-lobe reference. `results/` and the Dockerfile are machine-local and ignored by Git.

For an interactive Gazebo window plus RViz2, run this from an X11 desktop. It creates a timestamped fixture under `results/`, prepares the solver cache, enables MPCC automatically, and completes one 8-shaped lap. The NVIDIA driver capabilities are explicit because `--gpus all` alone mounts CUDA/NVML by default but not the OpenGL/X11 libraries needed by Gazebo and RViz:

```sh
XAUTHORITY=/run/user/$(id -u)/gdm/Xauthority
docker run --rm --net=host --gpus all \
  --tmpfs /tmp/runtime-root:rw,mode=700 \
  -e ROS_DOMAIN_ID=42 -e DISPLAY=:0 -e QT_X11_NO_MITSHM=1 -e XAUTHORITY=/root/.Xauthority \
  -e XDG_RUNTIME_DIR=/tmp/runtime-root -e NVIDIA_DRIVER_CAPABILITIES=compute,utility,graphics,display \
  -e AIMS_MPCC_CACHE_DIR=/results/ccache \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw -v "$XAUTHORITY:/root/.Xauthority:ro" \
  -v "$PWD/src/aims_gazebo_sim/results:/results" \
  aimsracer-mpcc:gazebo-local bash -lc 'source /opt/ros/humble/setup.bash; source /ws/install/setup.bash; ros2 run aims_gazebo_sim run_visual'
```

The initial invocation compiles the reference-specific solver cache. Later runs of the same reference reuse `/results/ccache`.

No physical joystick is required. `vesc_gazebo_bridge` publishes synthetic CRSF
channels selecting autonomous speed control. Before those channels and the first
MPCC `/drive` command arrive, `joystick_control` can report that it is waiting
for RC or navigation input; those startup notices are limited to once per second
and do not disable the later automatic MPCC enable.

The acceptance result proves that this ROS graph can close the loop with the configured Gazebo vehicle, Gazebo clock, physical wheel contacts, VESC conversion, and source MPCC runtime. It does not calibrate real tires, drivetrain dynamics, localization, or real-car safety.

The vehicle SDF follows the interface and parameter layout of Gazebo's Apache-2.0 Ackermann example: <https://github.com/gazebosim/gz-sim/blob/gz-sim6/examples/worlds/ackermann_steering.sdf>.
