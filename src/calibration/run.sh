#!/usr/bin/env bash
set -euo pipefail
calibration_group=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
calibration_workspace=$(cd -- "$calibration_group/../.." && pwd)
usage() {
  echo 'Usage: src/calibration/run.sh current|lateral|mocap COMMAND [arguments]' >&2
}
if [[ $# -lt 2 ]]; then usage; exit 2; fi
calibration_tool=$1
calibration_command=$2
shift 2
set +u
source /opt/ros/humble/setup.bash
source "$calibration_workspace/install/setup.bash"
set -u
case "$calibration_tool" in
  mocap) exec "$calibration_group/mocap_validation/run.sh" "$calibration_command" "$@" ;;
  current)
    if [[ "$calibration_command" == launch ]]; then
      exec ros2 launch aims_racer_system calibration.launch.py "$@"
    fi
    case "$calibration_command" in inspect|start|stop|baseline|plan|results|wheel-current-fit|wheel-current-query)
      exec ros2 run aims_calibration "$calibration_command" "$@" ;; esac ;;
  lateral)
    if [[ "$calibration_command" == launch ]]; then
      exec ros2 launch aims_lateral_grip lateral_grip_calib.launch.py "$@"
    fi
    case "$calibration_command" in inspect|start|stop|analyze)
      exec ros2 run aims_lateral_grip "$calibration_command" "$@" ;; esac ;;
esac
usage
exit 2
