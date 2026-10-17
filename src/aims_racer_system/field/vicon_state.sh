#!/usr/bin/env bash
# Observer/acquisition entry only; does not enable vehicle control.
set -eo pipefail
aims_field_dir=$(dirname -- "$(readlink -f -- "${BASH_SOURCE[0]}")")
source "$aims_field_dir/state_observer_env.bash"
exec python3 "$AIMS_VICON_TOOLS/run.py" "$@"
