#!/usr/bin/env bash
# Launch only; /calib/enable remains an explicit operator action.
set -eo pipefail
calibration_source_dir=$(cd -- "$(dirname -- "$(readlink -f -- "${BASH_SOURCE[0]}")")" && pwd)
source "$calibration_source_dir/field_env.bash"
case "${1:-}" in
  B|b) calibration_profile=accel_full_step_1to5 ;;
  C|c) calibration_profile=decel_full_step_5to1 ;;
  *) echo 'Usage: run_full_step.sh B|C [additional ros2 launch arguments]' >&2; exit 2 ;;
esac
shift
: "${AIMS_MAP:?Set AIMS_MAP to the field map}"
: "${AIMS_REFERENCE:?Set AIMS_REFERENCE to the qualified reference directory}"
: "${AIMS_BASELINE:?Set AIMS_BASELINE to the frozen Stage-A baseline}"
exec ros2 launch aims_racer_system calibration.launch.py \
  profile:="$calibration_profile" base_current_file:="$AIMS_BASELINE" \
  steering_source:=pp map_file:="$AIMS_MAP" reference_directory:="$AIMS_REFERENCE" \
  record:=true longitudinal_acceleration:=false auto_start:=false rviz:=true "$@"
