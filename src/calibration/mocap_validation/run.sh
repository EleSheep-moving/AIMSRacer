#!/usr/bin/env bash
# Same entry on host (inside ROS Humble Docker) and NX. No vehicle start/control.
set -euo pipefail
mocap_group=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
mocap_workspace=$(cd -- "$mocap_group/../../.." && pwd)
set +u
source /opt/ros/humble/setup.bash
source "$mocap_workspace/install/setup.bash"
set -u
export OPENBLAS_NUM_THREADS=${OPENBLAS_NUM_THREADS:-1}
exec ros2 run aims_mocap_validation mocap "$@"
