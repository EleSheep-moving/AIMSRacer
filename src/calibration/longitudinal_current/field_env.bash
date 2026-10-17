# Source in every field terminal. This file does not enable a controller.
calibration_source_dir=$(cd -- "$(dirname -- "$(readlink -f -- "${BASH_SOURCE[0]}")")" && pwd)
calibration_main_workspace=$(cd -- "$calibration_source_dir/../../.." && pwd)
source /opt/ros/humble/setup.bash
source "$calibration_main_workspace/install/setup.bash"
calibration_site_file=${AIMS_CALIBRATION_SITE_ENV:-$HOME/.config/aimsracer/calibration-site.bash}
if [[ -f "$calibration_site_file" ]]; then source "$calibration_site_file"; fi
unset calibration_source_dir calibration_main_workspace calibration_site_file
