#!/usr/bin/env bash
set -euo pipefail

# Start known_map_localization.launch.py beside V2/V3 first. An explicit map
# avoids silently selecting the wrong recording from ~/maps/.
if (( $# == 0 )); then
  echo "Usage: $0 /absolute/path/to/map.pcd [--x X --y Y --z Z --yaw YAW]" >&2
  exit 2
fi

exec ros2 run aims_racer_system relocalize_known_map.py "$@"
