#!/usr/bin/env bash
# Desktop replay only. Uses pinned sources and a message-only Livox package.
set -euo pipefail
repo_dir="$(git -C "$(dirname "$0")" rev-parse --show-toplevel)"
replay_dir="${1:-$repo_dir/log/wheel-imu-ndt}"
bag_dir="${2:-/home/elesheep/AIMSRacerBag}"
container_name="${NDT_REPLAY_CONTAINER:-aimsracer-ndt-replay}"
image_name="${NDT_REPLAY_IMAGE:-aimsracer-ndt:humble-replay}"
mkdir -p "$replay_dir/deps" "$replay_dir/ws" "$replay_dir/image"
replay_dir="$(realpath "$replay_dir")"
bag_dir="$(realpath "$bag_dir")"
checkout() {
    local directory="$replay_dir/deps/$1" url="$2" revision="$3"
    if [[ ! -d "$directory/.git" ]]; then
        git clone "$url" "$directory"
        git -C "$directory" checkout --detach "$revision"
    elif [[ "$(git -C "$directory" rev-parse HEAD)" != "$revision" ]]; then
        echo "Refusing to replace existing dependency at $directory (different revision)." >&2
        exit 1
    fi
}
checkout lidar_localization_ros2 https://github.com/rsasaki0109/lidar_localization_ros2.git 5f795a6cd886a20ade4175cb70bde630ac9ec785
checkout ndt_omp_ros2 https://github.com/rsasaki0109/ndt_omp_ros2.git 63bf15b965b71d3a53db1757abe8e31b6114372a
checkout livox_driver_source https://github.com/Livox-SDK/livox_ros_driver2.git 21445540f0d100dc86a7e6df312dd70bbdb4afdf
patch_file="$repo_dir/src/aims_racer_system/replay/ndt-full-rotation-diagnostic.patch"
if git -C "$replay_dir/deps/lidar_localization_ros2" apply --check "$patch_file" 2>/dev/null; then
    git -C "$replay_dir/deps/lidar_localization_ros2" apply "$patch_file"
elif ! git -C "$replay_dir/deps/lidar_localization_ros2" apply --reverse --check "$patch_file"; then
    echo 'NDT checkout does not match the documented diagnostic patch.' >&2
    exit 1
fi
git -C "$replay_dir/deps/lidar_localization_ros2" diff HEAD --binary > "$replay_dir/observed-ndt.patch"
cmp "$patch_file" "$replay_dir/observed-ndt.patch" || { echo 'Unexpected NDT source changes.' >&2; exit 1; }
for dependency in lidar_localization_ros2 ndt_omp_ros2 livox_driver_source; do
    [[ -z "$(git -C "$replay_dir/deps/$dependency" ls-files --others --exclude-standard)" ]] || { echo "Unexpected untracked dependency files: $dependency" >&2; exit 1; }
    if [[ "$dependency" != lidar_localization_ros2 ]]; then
        git -C "$replay_dir/deps/$dependency" diff HEAD --exit-code
    fi
done
mkdir -p "$replay_dir/deps/livox_ros_driver2/msg"
cp "$replay_dir/deps/livox_driver_source/msg/CustomMsg.msg" "$replay_dir/deps/livox_driver_source/msg/CustomPoint.msg" "$replay_dir/deps/livox_ros_driver2/msg/"
cat > "$replay_dir/deps/livox_ros_driver2/CMakeLists.txt" <<'CMAKE'
cmake_minimum_required(VERSION 3.8)
project(livox_ros_driver2)
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME} "msg/CustomPoint.msg" "msg/CustomMsg.msg" DEPENDENCIES std_msgs)
ament_export_dependencies(rosidl_default_runtime)
ament_package()
CMAKE
cat > "$replay_dir/deps/livox_ros_driver2/package.xml" <<'XML'
<?xml version="1.0"?>
<package format="3"><name>livox_ros_driver2</name><version>1.2.4</version>
<description>Replay-only exact Livox interfaces; no hardware driver.</description>
<maintainer email="noreply@example.com">AIMSRacer</maintainer><license>MIT</license>
<buildtool_depend>ament_cmake</buildtool_depend><buildtool_depend>rosidl_default_generators</buildtool_depend>
<depend>std_msgs</depend><exec_depend>rosidl_default_runtime</exec_depend>
<member_of_group>rosidl_interface_packages</member_of_group>
<export><build_type>ament_cmake</build_type></export></package>
XML
cat > "$replay_dir/image/Dockerfile" <<'DOCKER'
FROM ros:humble
SHELL ["/bin/bash", "-o", "pipefail", "-c"]
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential cmake git libeigen3-dev libpcl-dev libgtest-dev \
    python3-pytest python3-scipy python3-yaml python3-matplotlib \
    ros-humble-robot-localization ros-humble-pcl-conversions \
    ros-humble-tf2-eigen ros-humble-tf2-geometry-msgs ros-humble-tf2-sensor-msgs \
    ros-humble-ament-cmake-pytest ros-humble-ament-cmake-gtest \
    && rm -rf /var/lib/apt/lists/*
ENV MPLBACKEND=Agg PYTHONDONTWRITEBYTECODE=1
WORKDIR /ws
CMD ["bash"]
DOCKER
docker build -t "$image_name" "$replay_dir/image"
if docker inspect "$container_name" >/dev/null 2>&1; then
    echo "Existing container $container_name: use it if its mounts match, or choose NDT_REPLAY_CONTAINER. No automatic replacement." >&2
    exit 1
fi
docker run -d --name "$container_name" --network host --ipc host \
    -e "ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-193}" -e ROS_LOCALHOST_ONLY=1 \
    -e OMP_NUM_THREADS=2 -e OPENBLAS_NUM_THREADS=1 -e PYTHONDONTWRITEBYTECODE=1 \
    -v "$repo_dir:/repo" -v "$replay_dir/deps:/deps:ro" -v "$replay_dir/ws:/ws" \
    -v "$bag_dir:/bags:ro" "$image_name" sleep infinity
docker exec "$container_name" bash -lc '
    source /opt/ros/humble/setup.bash
    export MAKEFLAGS=-j2 CMAKE_BUILD_PARALLEL_LEVEL=2
    cd /ws
    colcon build --base-paths /deps/livox_ros_driver2 /deps/ndt_omp_ros2 /deps/lidar_localization_ros2 \
        --packages-up-to lidar_localization_ros2 livox_ros_driver2 --parallel-workers 1 \
        --cmake-args -DBUILD_TESTING=OFF -DCMAKE_BUILD_TYPE=Release
    source install/setup.bash
    colcon build --base-paths /repo/src/aims_racer_system --packages-select aims_racer_system \
        --parallel-workers 1 --cmake-args -DBUILD_TESTING=ON -DCMAKE_BUILD_TYPE=Release
'
echo "Replay environment ready: $container_name. Source /opt/ros/humble/setup.bash and /ws/install/setup.bash inside it."
