#!/usr/bin/env bash
# Prepared host ROS Humble container; the main workspace is mounted at /work.
set -euo pipefail
mocap_container=${MOCAP_CONTAINER:-aims-mocap-main-20261018}
mocap_docker_flags=(-i)
if [[ -t 0 && -t 1 ]]; then mocap_docker_flags=(-it); fi
exec docker exec "${mocap_docker_flags[@]}" \
  -e ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}" \
  -e ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-0}" \
  "$mocap_container" /work/src/calibration/mocap_validation/run.sh "$@"
