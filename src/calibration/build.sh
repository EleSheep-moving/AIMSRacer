#!/usr/bin/env bash
set -euo pipefail
calibration_group=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
calibration_workspace=$(cd -- "$calibration_group/../.." && pwd)
cd "$calibration_workspace"
exec colcon build --packages-up-to aims_calibration aims_lateral_grip aims_mocap_validation \
  --parallel-workers 1 --cmake-args -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_POLICY_VERSION_MINIMUM=3.5 \
  "-DCMAKE_PROJECT_INCLUDE=$calibration_group/mocap_validation/cmake/threads-compat.cmake" "$@"
