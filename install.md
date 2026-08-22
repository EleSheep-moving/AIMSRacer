# AIMSRacer Installation Guide

This guide installs AIMSRacer and its build dependencies on ROS 2 Humble.
The base workspace procedure was verified on 2026-08-08. The ZED ROS 2 source set
was pinned and its CUDA requirements were checked on the target below on
2026-08-11; installing the native ZED SDK is still a required host step:

| Item | Target environment |
| --- | --- |
| Computer | NVIDIA Jetson Orin NX 16 GB (AArch64) |
| OS | Ubuntu 22.04 (Jammy) |
| Jetson Linux | L4T 36.5.2 (JetPack 6.2.2 family) |
| ROS | ROS 2 Humble |
| CUDA / cuDNN / TensorRT | CUDA 12.6 / cuDNN 9 / TensorRT 10 |
| ZED SDK | 5.4.1 for L4T 36.5 |
| ZED ROS 2 sources | wrapper/examples 5.4.1, messages 5.3.0, description 0.1.5 |
| Compiler | GCC 11 |
| CMake | 4.4.2 |
| AIMSRacer branch | `agent/vehicle-dynamics-calibration-wip` |

The commands below use Tsinghua TUNA for Ubuntu, ROS 2, and rosdep, and
Aliyun for Python packages where practical. Git repositories are cloned from
their upstream GitHub URLs.

> [!IMPORTANT]
> Do not blindly run a full `apt upgrade` on a Jetson. It can replace packages
> supplied by JetPack. This guide updates only the package indexes and the
> `systemd`/`udev` packages required before installing ROS 2 Humble.

## Contents

1. [Preflight checks](#1-preflight-checks)
2. [Clone AIMSRacer](#2-clone-aimsracer)
3. [Configure domestic mirrors](#3-configure-domestic-mirrors)
4. [Install ROS 2 and system packages](#4-install-ros-2-and-system-packages)
5. [Configure rosdep](#5-configure-rosdep)
6. [Build native dependencies](#6-build-native-dependencies)
7. [Build the Livox underlay](#7-build-the-livox-underlay)
8. [Create the Python environment](#8-create-the-python-environment)
9. [Build AIMSRacer](#9-build-aimsracer)
10. [Persist the shell environment](#10-persist-the-shell-environment)
11. [Hardware configuration](#11-hardware-configuration)
12. [Verification](#12-verification)
13. [Troubleshooting](#13-troubleshooting)

## 1. Preflight checks

The tested path requires Ubuntu 22.04. On Jetson, the architecture must be
`arm64`:

```bash
. /etc/os-release
printf 'Ubuntu codename: %s\n' "$VERSION_CODENAME"
dpkg --print-architecture
uname -m
tr -d '\0' </proc/device-tree/model
head -n 1 /etc/nv_tegra_release
nvpmodel -q
nvcc --version
df -h "$HOME"
```

Expected values are `jammy`, `arm64`, `aarch64`, an Orin NX model, L4T
`R36 ... REVISION: 5.2`, and CUDA 12.6. Keep at least 25 GB of free disk space
for ROS, the ZED SDK, PCL, native dependencies, and build artifacts. Do not use
an x86-64 Ubuntu ZED installer on Jetson: the installer must match the L4T
release printed above.

Install the bootstrap tools:

```bash
sudo apt update
sudo apt install -y \
  ca-certificates \
  curl \
  git \
  gnupg2 \
  locales \
  software-properties-common

sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8
```

## 2. Clone AIMSRacer

The URL shown in a GitHub browser includes `/tree/<branch>`, but that browser
URL cannot be passed directly to `git clone`. Clone the repository and select
the branch explicitly:

```bash
cd "$HOME"
git clone \
  --branch agent/vehicle-dynamics-calibration-wip \
  --recurse-submodules \
  git@github.com:EleSheep-moving/AIMSRacer.git

cd "$HOME/AIMSRacer"
git submodule update --init --recursive
git status --short --branch
git submodule status
```

The ZED 2/2i integration is split into four pinned upstream submodules:

| Path | Version | Purpose |
| --- | --- | --- |
| `src/zed-ros2-wrapper` | 5.4.1 | `zed_components`, launch files, and CUDA/ZED SDK integration |
| `src/zed-ros2-interfaces` | 5.3.0 | `zed_msgs` messages and services |
| `src/zed-ros2-description` | 0.1.5 | ZED 2/2i URDF, xacro, and meshes |
| `src/zed-ros2-examples` | 5.4.1 | RViz, IPC, CUDA, and optional NITROS examples |

The Git superproject records the exact commit of every submodule. Do not clone
another copy of these repositories inside `src`, and do not replace the pinned
commits with each repository's moving default branch.

If the repository already exists, update it without overwriting local work:

```bash
cd "$HOME/AIMSRacer"
git fetch origin
git submodule update --init --recursive
```

## 3. Configure domestic mirrors

### 3.1 Ubuntu Ports mirror (Jetson/AArch64)

Back up the current source list first. Ordinary Jammy repositories are moved
to TUNA; the official Ubuntu Ports server is retained for security updates.
JetPack/NVIDIA entries under `/etc/apt/sources.list.d/` are not changed.

```bash
sudo cp --preserve=all \
  /etc/apt/sources.list \
  "/etc/apt/sources.list.aimsracer-backup-$(date +%Y%m%d-%H%M%S)"

sudo sed -i -E \
  '/^[[:space:]]*deb .*jammy-security/! s#https?://ports\.ubuntu\.com/ubuntu-ports/?#https://mirrors.tuna.tsinghua.edu.cn/ubuntu-ports/#g' \
  /etc/apt/sources.list

sudo apt update
```

This replacement is for `arm64`. An x86-64 machine uses TUNA's regular
Ubuntu mirror instead of `ubuntu-ports`.

### 3.2 ROS 2 mirror and signing key

Use TUNA for the ROS 2 package repository. Obtain the small signing key from
the official ROS distribution repository; some `github-raw` mirror paths for
`ros.key` return HTTP 404.

```bash
curl -fsSL --retry 3 \
  https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  -o /tmp/aimsracer-ros.key

sudo install -d -m 0755 /usr/share/keyrings
sudo gpg --dearmor --batch --yes \
  --output /usr/share/keyrings/ros-archive-keyring.gpg \
  /tmp/aimsracer-ros.key
rm -f /tmp/aimsracer-ros.key

ARCH="$(dpkg --print-architecture)"
echo "deb [arch=${ARCH} signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] https://mirrors.tuna.tsinghua.edu.cn/ros2/ubuntu jammy main" \
  | sudo tee /etc/apt/sources.list.d/ros2.list >/dev/null

sudo apt update
```

## 4. Install ROS 2 and system packages

Update `systemd` and `udev` before ROS, then install the tested dependency set.
Use exact package names instead of broad wildcards so the result is
reproducible.

```bash
sudo apt install -y systemd udev

sudo apt install -y \
  build-essential \
  cmake \
  git \
  gstreamer1.0-gl \
  gstreamer1.0-libav \
  gstreamer1.0-plugins-bad \
  libasio-dev \
  libboost-system-dev \
  libboost-thread-dev \
  libeigen3-dev \
  libfmt-dev \
  libfuse2 \
  libgstreamer1.0-dev \
  nlohmann-json3-dev \
  libomp-dev \
  libpcl-dev \
  libusb-1.0-0-dev \
  libxcb-cursor-dev \
  libxkbcommon-x11-0 \
  libyaml-cpp-dev \
  ninja-build \
  pkg-config \
  python3-colcon-common-extensions \
  python3-dev \
  python3-pip \
  python3-rosdep \
  python3-vcstool \
  python3-venv \
  rapidjson-dev \
  ros-dev-tools \
  ros-humble-ackermann-msgs \
  ros-humble-ament-cmake-clang-format \
  ros-humble-asio-cmake-module \
  ros-humble-backward-ros \
  ros-humble-compressed-depth-image-transport \
  ros-humble-compressed-image-transport \
  ros-humble-desktop \
  ros-humble-depthimage-to-laserscan \
  ros-humble-diagnostic-aggregator \
  ros-humble-diagnostic-updater \
  ros-humble-geographic-msgs \
  ros-humble-grid-map-rviz-plugin \
  ros-humble-gtsam \
  ros-humble-image-transport-plugins \
  ros-humble-nav2-bringup \
  ros-humble-navigation2 \
  ros-humble-nmea-msgs \
  ros-humble-pcl-conversions \
  ros-humble-pcl-ros \
  ros-humble-plotjuggler-ros \
  ros-humble-rmw-cyclonedds-cpp \
  ros-humble-robot-localization \
  ros-humble-rqt \
  ros-humble-slam-toolbox \
  ros-humble-tf-transformations \
  ros-humble-theora-image-transport \
  ros-humble-udp-msgs \
  ros-humble-xacro
```

Two package-name details matter on Jammy:

- Use `rapidjson-dev`, not `librapidjson-dev`.
- Use `ros-humble-gtsam`, not Ubuntu's unavailable `libgtsam-dev`.

### 4.1 Install the CUDA-enabled ZED SDK on Orin NX

`zed_components` is not a CPU-only camera driver. Its build requires both
`find_package(ZED REQUIRED)` and `find_package(CUDAToolkit REQUIRED)`, and the
ZED SDK performs stereo depth, positional tracking, and AI inference on the
Orin GPU. The source packages under `src` therefore cannot replace the native
ZED SDK.

First verify that the JetPack CUDA stack is complete:

```bash
command -v nvcc
nvcc --version
ldconfig -p | grep -E 'libcuda\.so|libcudnn|libnvinfer' | head
```

For the verified L4T 36.5.x target, download the matching Jetson installer.
The versioned endpoint prevents a later SDK release from silently changing the
build ABI:

```bash
ZED_SDK_INSTALLER="$HOME/Downloads/ZED_SDK_Tegra_L4T36.5_v5.4.1.zstd.run"

curl --http1.1 -fL \
  --retry 3 \
  --retry-all-errors \
  https://download.stereolabs.com/zedsdk/5.4.1/l4t36.5/jetsons \
  -o "$ZED_SDK_INSTALLER"

chmod +x "$ZED_SDK_INSTALLER"
"$ZED_SDK_INSTALLER" -- silent skip_python
```

Keep the SDK tools: `ZED_Diagnostic` and `ZED_Explorer` are used for USB,
camera, CUDA, and sensor checks. `skip_python` only omits the optional PyZED
binding; AIMSRacer uses the C++ SDK. After installation, open a new terminal
and verify the development files:

```bash
test -f /usr/local/zed/zed-config.cmake
test -f /usr/local/zed/include/sl/Camera.hpp
ldconfig -p | grep libsl_zed
```

If the first CMake configure reports `Specify CUDA_TOOLKIT_ROOT_DIR`, the
JetPack development metapackages are incomplete. Install them, then re-run the
checks above:

```bash
sudo apt install nvidia-jetpack nvidia-jetpack-dev
```

Do not install desktop CUDA or the x86-64 ZED SDK over JetPack. JetPack already
provides the CUDA driver/runtime combination supported by this Orin image.
The core `zed_components` target and the `zed_rgb_convert`/
`zed_aruco_localization` examples link to `CUDA::cudart`; the default
`NEURAL_LIGHT` depth mode also runs in the ZED SDK on the GPU. NVIDIA Isaac ROS
NITROS is a separate, optional GPU-buffer transport layer. It is not required
for CUDA depth or tracking, and is intentionally not mixed into this base ROS
Humble installation because its release must match a specific Isaac ROS image.

Load ROS in the current terminal:

```bash
source /opt/ros/humble/setup.bash
```

## 5. Configure rosdep

Configure both the rosdep source list and distribution index to use TUNA:

```bash
sudo install -d -m 0755 /etc/ros/rosdep/sources.list.d
sudo curl -fsSL --retry 3 \
  https://mirrors.tuna.tsinghua.edu.cn/github-raw/ros/rosdistro/master/rosdep/sources.list.d/20-default.list \
  -o /etc/ros/rosdep/sources.list.d/20-default.list

export ROSDISTRO_INDEX_URL=https://mirrors.tuna.tsinghua.edu.cn/rosdistro/index-v4.yaml
rosdep update --rosdistro humble

cd "$HOME/AIMSRacer"
rosdep install \
  --from-paths src \
  --ignore-src \
  --rosdistro humble \
  --skip-keys "GTSAM livox_ros_driver2 serial scout_description" \
  -r -y
```

The skipped keys are built in later sections or supplied by
`ros-humble-gtsam`. `scout_description` is referenced only by one optional
upstream ZED robot-integration example; it is not used by AIMSRacer or by the
ZED 2i camera node.

## 6. Build native dependencies

AIMSRacer needs CppLinuxSerial, Sophus, and Livox-SDK2. Keep their source trees
outside the ROS workspace:

```bash
DEPS_DIR="$HOME/aimsracer_deps"
mkdir -p "$DEPS_DIR"
cd "$DEPS_DIR"

test -d CppLinuxSerial/.git || \
  git clone --depth 1 https://github.com/gbmhunter/CppLinuxSerial.git

test -d serial-ros2/.git || \
  git clone --depth 1 https://github.com/RoverRobotics-forks/serial-ros2.git

test -d Sophus/.git || \
  git clone --depth 1 --branch 1.22.10 \
    https://github.com/strasdat/Sophus.git

test -d Livox-SDK2/.git || \
  git clone --depth 1 https://github.com/Livox-SDK/Livox-SDK2.git
```

Build and install CppLinuxSerial:

```bash
cmake \
  -S "$DEPS_DIR/CppLinuxSerial" \
  -B "$DEPS_DIR/CppLinuxSerial/build" \
  -DBUILD_TESTS=OFF \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_INSTALL_PREFIX=/usr/local
cmake --build "$DEPS_DIR/CppLinuxSerial/build" --parallel 2
sudo cmake --install "$DEPS_DIR/CppLinuxSerial/build"
```

Build Sophus. `CMAKE_POLICY_VERSION_MINIMUM` is required when the host has
CMake 4, while remaining harmless on older supported CMake versions:

```bash
cmake \
  -S "$DEPS_DIR/Sophus" \
  -B "$DEPS_DIR/Sophus/build" \
  -DBUILD_SOPHUS_EXAMPLES=OFF \
  -DBUILD_SOPHUS_TESTS=OFF \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_INSTALL_PREFIX=/usr/local \
  -DCMAKE_POLICY_VERSION_MINIMUM=3.5 \
  -DSOPHUS_INSTALL=ON \
  -DSOPHUS_USE_BASIC_LOGGING=ON
cmake --build "$DEPS_DIR/Sophus/build" --parallel 2
sudo cmake --install "$DEPS_DIR/Sophus/build"
```

Build Livox-SDK2 with the same CMake compatibility setting:

```bash
cmake \
  -S "$DEPS_DIR/Livox-SDK2" \
  -B "$DEPS_DIR/Livox-SDK2/build" \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_INSTALL_PREFIX=/usr/local \
  -DCMAKE_POLICY_VERSION_MINIMUM=3.5
cmake --build "$DEPS_DIR/Livox-SDK2/build" --parallel 2
sudo cmake --install "$DEPS_DIR/Livox-SDK2/build"
sudo ldconfig
```

Warnings emitted by RapidJSON while compiling Livox-SDK2 are non-fatal if the
build exits successfully.

## 7. Build the Livox underlay

`livox_ros_driver2` must be built in a separate underlay. Its `build.sh`
selects the correct ROS 2 package metadata, so do not replace this step with a
plain standalone CMake build.

```bash
mkdir -p "$HOME/livox_ws/src"
cd "$HOME/livox_ws/src"

test -d livox_ros_driver2/.git || \
  git clone --depth 1 \
    https://github.com/Livox-SDK/livox_ros_driver2.git

if [ ! -e serial-ros2 ]; then
  ln -s "$HOME/aimsracer_deps/serial-ros2" serial-ros2
fi

source /opt/ros/humble/setup.bash
cd "$HOME/livox_ws/src/livox_ros_driver2"
./build.sh humble
```

Verify and load the underlay:

```bash
source "$HOME/livox_ws/install/setup.bash"
ros2 pkg prefix livox_ros_driver2
ros2 pkg prefix serial
```

## 8. Create the Python environment

Use a virtual environment for map preprocessing and calibration analysis.
The explicit list below avoids ambiguity in older requirements files.

```bash
cd "$HOME/AIMSRacer"
python3 -m venv .venv

.venv/bin/python -m pip install \
  --index-url https://mirrors.aliyun.com/pypi/simple/ \
  --upgrade pip setuptools wheel

.venv/bin/python -m pip install \
  --index-url https://mirrors.aliyun.com/pypi/simple/ \
  "numpy>=1.17.3,<1.25" \
  "open3d==0.18.0" \
  matplotlib \
  "scipy<1.16" \
  PyYAML \
  pillow \
  pandas \
  scikit-learn

.venv/bin/python -m pip check
```

If `python3 -m venv` reports that `ensurepip` is unavailable, reinstall the
Jammy venv package and retry:

```bash
sudo apt install --reinstall python3-venv
```

Do not activate `.venv` while building the ROS workspace. ROS 2 Humble is
installed for the system Python and should be built in that environment.

## 9. Build AIMSRacer

Always source the ROS installation and Livox underlay, in that order:

```bash
source /opt/ros/humble/setup.bash
source "$HOME/livox_ws/install/setup.bash"

cd "$HOME/AIMSRacer"
colcon build \
  --symlink-install \
  --parallel-workers 2 \
  --packages-skip zed_debug \
  --cmake-args -DCMAKE_BUILD_TYPE=Release
```

Two parallel workers are a conservative default for Jetson and avoid memory
pressure while compiling PCL- and ZED-heavy packages on the Orin NX 16 GB. Use
`--parallel-workers 1` if the compiler is killed by the out-of-memory manager.
`zed_debug` is an upstream internal-development executable; the production
`zed_components`, `zed_wrapper`, `zed_msgs`, and examples are still built.

CMake policy warnings such as `CMP0148`, `CMP0167`, and warnings from PCL,
Eigen, or upstream LIO code are expected with CMake 4. They do not indicate a
failure when colcon ends with no failed packages. With the pinned ZED sources,
`colcon list` reports 42 packages; the command above intentionally skips only
`zed_debug`.

Load the completed overlay:

```bash
source "$HOME/AIMSRacer/install/setup.bash"
```

## 10. Persist the shell environment

Add each line once. The source order is important:

```bash
grep -qxF 'source /opt/ros/humble/setup.bash' "$HOME/.bashrc" || \
  echo 'source /opt/ros/humble/setup.bash' >> "$HOME/.bashrc"

grep -qxF 'source "$HOME/livox_ws/install/setup.bash"' "$HOME/.bashrc" || \
  echo 'source "$HOME/livox_ws/install/setup.bash"' >> "$HOME/.bashrc"

grep -qxF 'source "$HOME/AIMSRacer/install/setup.bash"' "$HOME/.bashrc" || \
  echo 'source "$HOME/AIMSRacer/install/setup.bash"' >> "$HOME/.bashrc"

grep -qxF 'export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp' "$HOME/.bashrc" || \
  echo 'export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp' >> "$HOME/.bashrc"

grep -qxF 'export ROS_LOCALHOST_ONLY=0' "$HOME/.bashrc" || \
  echo 'export ROS_LOCALHOST_ONLY=0' >> "$HOME/.bashrc"

grep -qxF 'export ROSDISTRO_INDEX_URL=https://mirrors.tuna.tsinghua.edu.cn/rosdistro/index-v4.yaml' "$HOME/.bashrc" || \
  echo 'export ROSDISTRO_INDEX_URL=https://mirrors.tuna.tsinghua.edu.cn/rosdistro/index-v4.yaml' >> "$HOME/.bashrc"
```

Open a new terminal, or source the three setup files directly in the current
terminal. Do not add a `CYCLONEDDS_URI` until the intended DDS network
interface is known.

## 11. Hardware configuration

These steps are machine-specific and are intentionally separate from software
installation.

### 11.1 Serial permissions and udev rules

Inspect the supplied rules before installing them:

```bash
cd "$HOME/AIMSRacer"
ls -l rules
sed -n '1,200p' rules/*.rules
```

The ELRS rule targets the Jetson Orin 40-pin header UART (`ttyTHS1`) and
creates `/dev/ttyELRS`. The VESC rule matches its USB vendor/product IDs and
creates `/dev/ttyVESC`.

Install and activate the rules:

```bash
sudo install -m 0644 rules/*.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules
sudo udevadm trigger
sudo usermod -aG dialout "$USER"
```

Log out and back in after changing group membership. Remove `brltty` only if
logs show that it is claiming the actual VESC/RC serial adapter; do not
remove it unconditionally.

### 11.2 Livox network

Configure the wired interface with a static IPv4 address in the same subnet
as the Mid-360, then update:

```text
~/livox_ws/src/livox_ros_driver2/config/MID360_config.json
```

The host IP and LiDAR IP must match the physical network. Do not copy example
addresses without checking the device label and current network configuration.

### 11.3 CycloneDDS

The software installation only selects CycloneDDS as the RMW implementation.
Binding CycloneDDS to Wi-Fi, Ethernet, or loopback is deployment-specific. A
wrong `CYCLONEDDS_URI` can prevent local nodes from discovering each other, so
configure it only after deciding whether remote ROS participants are required.

### 11.4 ZED 2i USB and Orin performance checks

Connect the ZED 2i directly to an Orin USB 3 port. Hubs and USB 2 fallback can
cause dropped frames even when the ROS node builds correctly:

```bash
lsusb | grep -i '2b03\|stereolabs'
lsusb -t
nvpmodel -q
```

The camera path in `lsusb -t` should report SuperSpeed (`5000M` or faster), not
`480M`. The verified machine uses `MAXN_SUPER`; choose a lower power mode only
after profiling camera, LiDAR, navigation, temperature, and battery limits
together. Adequate cooling is required for sustained neural depth processing.

## 12. Verification

Run these commands in a fresh terminal.

### 12.1 Package and dependency checks

```bash
printf 'ROS_DISTRO=%s RMW=%s\n' "$ROS_DISTRO" "$RMW_IMPLEMENTATION"

cd "$HOME/AIMSRacer"
rosdep check \
  --from-paths src \
  --ignore-src \
  --skip-keys "GTSAM livox_ros_driver2 serial scout_description"

.venv/bin/python -m pip check
sudo apt-get check
```

Expected results include `All system dependencies have been satisfied` and
`No broken requirements found`.

### 12.2 ROS package checks

```bash
ros2 pkg list | grep -E \
  '^(ackermann_mux|crsf_receiver|aims_racer_system|fastlio2|hba|livox_ros_driver2|localizer|pgo|point_lio|vesc|wheeltec_n100_imu|zed_components|zed_description|zed_display_rviz2|zed_msgs|zed_ros2|zed_wrapper)$'

ros2 pkg executables aims_racer_system
ros2 pkg executables fastlio2
ros2 pkg executables point_lio
ros2 pkg prefix zed_components
ros2 pkg prefix zed_wrapper
```

### 12.3 Safe launch parsing

`--show-args` parses the launch file without starting vehicle hardware:

```bash
ros2 launch aims_racer_system \
  base_orin_livox_bringup_v2.launch.py --show-args

ros2 launch aims_racer_system \
  lateral_grip_calib.launch.py --show-args

ros2 launch zed_wrapper \
  zed_camera.launch.py camera_model:=zed2i --show-args
```

The lateral calibration launch defaults to `armed:=false`. Keep it disarmed
until odometry, IMU, VESC telemetry, steering direction, and emergency stop
have been verified on the physical vehicle.

### 12.4 ZED 2i CUDA/GPU runtime check

Start the ZED 2i with the GPU neural-depth path. `NEURAL_LIGHT` is the preferred
starting profile when LiDAR localization and navigation share the Orin NX; it
is GPU accelerated while leaving more headroom than `NEURAL` or `NEURAL_PLUS`:

```bash
ros2 launch zed_wrapper zed_camera.launch.py \
  camera_model:=zed2i \
  camera_name:=zed2i \
  param_overrides:='depth.depth_mode:=NEURAL_LIGHT;depth.point_cloud_freq:=10.0'
```

In a second terminal, verify the effective parameter and data topics:

```bash
source /opt/ros/humble/setup.bash
source "$HOME/livox_ws/install/setup.bash"
source "$HOME/AIMSRacer/install/setup.bash"

ros2 param get /zed2i/zed_node depth.depth_mode
ros2 topic hz /zed2i/zed_node/depth/depth_registered
ros2 topic hz /zed2i/zed_node/point_cloud/cloud_registered
tegrastats
```

The launch log must contain `ZED SDK running on GPU #0`; the parameter must not
be `NONE`, and `tegrastats` should show non-zero `GR3D_FREQ` while depth data is
being consumed. This proves the native CUDA/ZED path is active. NITROS is not
the proof of GPU depth: if Isaac ROS is later installed, the wrapper detects it
at build time and can additionally use NITROS for GPU-buffer transport.

### 12.5 Repository check

```bash
cd "$HOME/AIMSRacer"
git status --short --branch
git submodule status
```

With `--symlink-install`, some versions of the ROS Python installation macros
may change only the executable bits of files under
`src/aims_racer_system/scripts/`. If `git diff --summary` confirms there are no
content changes, restore the original permissions with:

```bash
chmod 644 \
  src/aims_racer_system/scripts/__init__.py \
  src/aims_racer_system/scripts/lateral_grip_calib.py \
  src/aims_racer_system/scripts/longitudinal_calib.py \
  src/aims_racer_system/scripts/navThroughPoses.py \
  src/aims_racer_system/scripts/navWithAviod.py
```

## 13. Troubleshooting

### `curl: (22) ... 404` followed by `gpg: no valid OpenPGP data found`

The ROS signing-key URL is wrong or a raw-GitHub mirror does not carry that
file. Use the official key URL from [section 3.2](#32-ros-2-mirror-and-signing-key),
while keeping the ROS package repository on TUNA. Do not pipe an unchecked
HTTP response directly into GPG.

### CMake 4 rejects an old `cmake_minimum_required`

For Sophus and Livox-SDK2, include:

```bash
-DCMAKE_POLICY_VERSION_MINIMUM=3.5
```

Policy/deprecation warnings during the AIMSRacer build are normally harmless;
an `error:` line or a non-zero colcon summary is not.

### `Could not find livox_ros_driver2`

The underlay was not sourced:

```bash
source /opt/ros/humble/setup.bash
source "$HOME/livox_ws/install/setup.bash"
```

Then rebuild AIMSRacer.

### `Could not find GTSAM`

```bash
sudo apt install ros-humble-gtsam
source /opt/ros/humble/setup.bash
```

### `Could not find ZED` or `ZEDConfig.cmake`

The Jetson ZED SDK development files are missing or the wrong installer was
used. For L4T 36.5.x, repeat [section 4.1](#41-install-the-cuda-enabled-zed-sdk-on-orin-nx)
with the L4T 36.5 Jetson installer, then verify:

```bash
test -f /usr/local/zed/zed-config.cmake
test -f /usr/local/zed/include/sl/Camera.hpp
```

Do not work around this by removing `find_package(ZED REQUIRED)`; that would
remove the SDK path that provides GPU depth and tracking.

### `Specify CUDA_TOOLKIT_ROOT_DIR` while building `zed_components`

Confirm that JetPack's compiler and development packages are present:

```bash
command -v nvcc
nvcc --version
sudo apt install nvidia-jetpack nvidia-jetpack-dev
```

Delete only the failed ZED package build directories before retrying:

```bash
cd "$HOME/AIMSRacer"
rm -rf build/zed_components install/zed_components
colcon build \
  --symlink-install \
  --parallel-workers 1 \
  --packages-up-to zed_ros2 zed_display_rviz2 \
  --packages-skip zed_debug \
  --cmake-args -DCMAKE_BUILD_TYPE=Release
```

### ZED 2i opens at USB 2 speed or drops frames

Check the physical topology with `lsusb -t`. Move the camera to a direct USB 3
port and replace the cable if its branch reports `480M`. Reducing ROS publish
rates does not repair a USB 2 link.

### `No module named rclpy`

Build and launch ROS nodes outside `.venv`, after sourcing ROS Humble. Use
`.venv` only for preprocessing and offline analysis unless a script explicitly
documents otherwise.

### Compiler killed or Jetson becomes unresponsive

Retry with one worker:

```bash
cd "$HOME/AIMSRacer"
colcon build \
  --symlink-install \
  --parallel-workers 1 \
  --packages-skip zed_debug \
  --cmake-args -DCMAKE_BUILD_TYPE=Release
```

### Serial device is missing

```bash
lsusb
ls -l /dev/ttyELRS /dev/ttyVESC 2>/dev/null
readlink -f /dev/ttyELRS
id
journalctl -k --since '5 minutes ago'
```

Confirm the udev IDs and `dialout` membership before changing permissions.

### Restore the original Ubuntu source list

List the backups created in section 3, choose the correct timestamp, and copy
that explicit file back:

```bash
ls -1 /etc/apt/sources.list.aimsracer-backup-*
sudo cp /etc/apt/sources.list.aimsracer-backup-YYYYMMDD-HHMMSS \
  /etc/apt/sources.list
sudo apt update
```

## Updating and rebuilding

Preserve local work before updating. Then:

```bash
cd "$HOME/AIMSRacer"
git status --short --branch
git pull --ff-only
git submodule update --init --recursive

source /opt/ros/humble/setup.bash
source "$HOME/livox_ws/install/setup.bash"
colcon build \
  --symlink-install \
  --parallel-workers 2 \
  --packages-skip zed_debug \
  --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

See [README.md](README.md) for vehicle operation, mapping, localization, and
calibration workflows.
