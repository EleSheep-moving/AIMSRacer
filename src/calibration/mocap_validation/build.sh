#!/usr/bin/env bash
set -euo pipefail
mocap_group=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
mocap_workspace=$(cd -- "$mocap_group/../../.." && pwd)
cd "$mocap_workspace"
exec colcon build --packages-select vrpn vrpn_mocap aims_mocap_validation \
  --parallel-workers 1 --cmake-args -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_POLICY_VERSION_MINIMUM=3.5 \
  "-DCMAKE_PROJECT_INCLUDE=$mocap_group/cmake/threads-compat.cmake" "$@"
