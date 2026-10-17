# Source this file in a bash or zsh field terminal; no nodes are started.
if [ -n "${BASH_VERSION:-}" ]; then
  aims_state_env_source=${BASH_SOURCE[0]}
  aims_state_setup_suffix=bash
elif [ -n "${ZSH_VERSION:-}" ]; then
  aims_state_env_source=${(%):-%x}
  aims_state_setup_suffix=zsh
else
  echo 'Source state_observer_env.bash from bash or zsh.' >&2
  return 2
fi
aims_state_field_dir=$(dirname -- "$(readlink -f -- "$aims_state_env_source")")
export AIMS_STATE_WORKSPACE=$(cd -- "$aims_state_field_dir/../../.." && pwd)
source "/opt/ros/humble/setup.$aims_state_setup_suffix"
source "$AIMS_STATE_WORKSPACE/install/setup.$aims_state_setup_suffix"
export AIMS_VICON_TOOLS="$AIMS_STATE_WORKSPACE/verification/vicon_state"
export AIMS_VICON_CONFIG=${AIMS_VICON_CONFIG:-$AIMS_STATE_WORKSPACE/src/calibration/mocap_validation/aims_mocap_validation/aims_mocap_validation/config.template.json}
aims_state_site_file=${AIMS_STATE_SITE_ENV:-$HOME/.config/aimsracer/state-observer-site.bash}
if [ -f "$aims_state_site_file" ]; then source "$aims_state_site_file"; fi
unset aims_state_env_source aims_state_setup_suffix aims_state_field_dir aims_state_site_file
