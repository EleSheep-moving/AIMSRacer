# MPCC simulation host

Run numerical and Gazebo validation on an x86-64 Ubuntu 22.04 workstation with
ROS 2 Humble. The vehicle computers are intentionally excluded: the Orin and
NUC use their native deployment guides on the `feat/aims-mpcc` branch.

This branch provides two equivalent host setup paths. Choose one per workspace:
native source build or Docker. Both build the same controller, RC selector,
VESC conversion and simulator packages. They do not install vehicle drivers or
send commands to a real vehicle.

## 1. Obtain the simulation branch

```bash
git clone --branch mpcc-sim --recurse-submodules \
  git@github.com:EleSheep-moving/AIMSRacer.git AIMSRacer
cd AIMSRacer
git submodule update --init --recursive
```

Use `git switch mpcc-sim && git submodule update --init --recursive` when
updating an existing clone. The commands below assume the repository root is the
current directory.

## 2. Native Ubuntu setup and execution

Install [ROS 2 Humble Desktop](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html)
and initialize `rosdep` on the workstation first. Confirm the host baseline:

```bash
. /etc/os-release
test "$VERSION_ID" = 22.04
source /opt/ros/humble/setup.bash
ros2 --version
```

Install the simulation dependencies, resolve package dependencies, and build:

```bash
sudo apt update
sudo apt install \
  ccache gcc g++ python3-colcon-common-extensions python3-pip \
  ros-humble-ackermann-msgs ros-humble-ament-cmake-auto \
  ros-humble-diagnostic-updater ros-humble-tf2-geometry-msgs \
  ros-humble-ros-gz ros-humble-ros-gz-sim ros-humble-ros-gz-bridge ros-humble-rviz2 \
  python3-matplotlib
/usr/bin/python3 -m pip install --user --no-deps casadi==3.7.2

rosdep install --ignore-src -r -y --rosdistro humble --from-paths \
  src/ackermann_mux src/ros2_crsf_receiver/crsf_receiver_msg \
  src/vesc/vesc_msgs src/vesc/vesc_ackermann src/controller \
  src/aims_mpcc_sim src/aims_gazebo_sim
colcon build --symlink-install --packages-up-to aims_gazebo_sim
source install/setup.bash
```

### Numerical acceptance

This is the fastest closed-loop check. It uses an independent lagged kinematic
bicycle plant and writes a self-contained result directory.

```bash
mkdir -p results
ROS_DOMAIN_ID=83 ros2 run aims_mpcc_sim run_acceptance \
  "$PWD/results/numerical-$(date -u +%Y%m%dT%H%M%SZ)" \
  --track figure_eight --radius 6.0 --waist-ratio 0.3
```

Read `summary.json`; a successful run reports `"status": "PASS"`. The result
also contains the reference, controller telemetry, trajectory, plot and launch
log.

### Gazebo acceptance

Gazebo Fortress is launched headlessly and supplies the vehicle contact model,
VESC-to-Gazebo bridge and simulated clock.

```bash
mkdir -p results
ROS_DOMAIN_ID=83 ros2 run aims_gazebo_sim run_acceptance \
  "$PWD/results/gazebo-$(date -u +%Y%m%dT%H%M%SZ)" \
  --track figure_eight --radius 6.0 --waist-ratio 0.3
```

### Gazebo and RViz2 window

Run this from an X11 desktop with a working NVIDIA driver. It prepares the
figure-eight reference, starts Gazebo and RViz2, then enables MPCC automatically.

```bash
mkdir -p results
export AIMS_MPCC_CACHE_DIR="$PWD/results/ccache"
ros2 run aims_gazebo_sim run_visual --output-root "$PWD/results" \
  --track figure_eight --radius 6.0 --waist-ratio 0.3
```

## 3. Docker setup and execution

Install [Docker Engine for Ubuntu](https://docs.docker.com/engine/install/ubuntu/)
and ensure the current user can run `docker` without `sudo`. Verify the engine
before building:

```bash
docker version
docker run --rm hello-world
```

Build the simulation image. Build it on the same architecture that will run it;
the documented validation image was built on x86-64 Ubuntu 22.04.

```bash
docker build -f docker/simulation/Dockerfile -t aimsracer-mpcc:sim-humble .
mkdir -p results
```

### Numerical acceptance in Docker

The numerical run has no external ROS or graphics dependency, so it uses an
isolated Docker network.

```bash
docker run --rm --network none -e ROS_DOMAIN_ID=83 \
  -v "$PWD/results:/results" \
  aimsracer-mpcc:sim-humble \
  ros2 run aims_mpcc_sim run_acceptance \
    "/results/numerical-$(date -u +%Y%m%dT%H%M%SZ)" \
    --track figure_eight --radius 6.0 --waist-ratio 0.3
```

### Headless Gazebo acceptance in Docker

```bash
docker run --rm --network none -e ROS_DOMAIN_ID=83 \
  -v "$PWD/results:/results" \
  aimsracer-mpcc:sim-humble \
  ros2 run aims_gazebo_sim run_acceptance \
    "/results/gazebo-$(date -u +%Y%m%dT%H%M%SZ)" \
    --track figure_eight --radius 6.0 --waist-ratio 0.3
```

### Gazebo and RViz2 window in Docker

Run this from an X11 desktop. It passes the display server and NVIDIA graphics
capabilities into the container; no physical joystick is required.

```bash
XAUTHORITY=/run/user/$(id -u)/gdm/Xauthority
docker run --rm --net=host --gpus all \
  --tmpfs /tmp/runtime-root:rw,mode=700 \
  -e ROS_DOMAIN_ID=83 -e DISPLAY=:0 -e QT_X11_NO_MITSHM=1 \
  -e XAUTHORITY=/root/.Xauthority -e XDG_RUNTIME_DIR=/tmp/runtime-root \
  -e NVIDIA_DRIVER_CAPABILITIES=compute,utility,graphics,display \
  -e AIMS_MPCC_CACHE_DIR=/results/ccache \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  -v "$XAUTHORITY:/root/.Xauthority:ro" \
  -v "$PWD/results:/results" \
  aimsracer-mpcc:sim-humble \
  ros2 run aims_gazebo_sim run_visual --output-root /results \
    --track figure_eight --radius 6.0 --waist-ratio 0.3
```

The first run compiles a reference-specific solver cache in `results/ccache`.
Later runs of the same reference reuse it.

## Evidence boundary

Numerical `PASS` proves the ROS process/topic chain and the specified independent
plant close a complete lap within its acceptance limits. Gazebo `PASS` adds the
configured vehicle model, contact physics, simulated clock and VESC conversion.
Neither result calibrates real tires, drivetrain dynamics, localization,
onboard-computer timing or real-vehicle safety.
