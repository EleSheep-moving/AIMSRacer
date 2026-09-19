#!/bin/bash
set -euo pipefail
cd "$(dirname "$0")/../../.."
docker build -f src/controller/docker/Dockerfile -t aimsracer-mpcc:humble .
results_dir="${MPCC_RESULTS_DIR:-$PWD/src/controller/results/$(date -u +%Y%m%dT%H%M%SZ)}"
mkdir -p "$results_dir"
docker image inspect aimsracer-mpcc:humble > "$results_dir/image.json"
docker run --rm --network none -e ROS_DOMAIN_ID=83 aimsracer-mpcc:humble \
    python3 -m pytest -q /ws/src/controller/test | tee "$results_dir/unit-tests.txt"
docker run --rm --network none aimsracer-mpcc:humble python3 -m pip freeze > "$results_dir/python-packages.txt"
for scenario in nominal odom_drop manual rc_loss clock_reset solver_stall solver_crash disable; do
    docker run --rm --network none -e ROS_DOMAIN_ID=83 -v "$results_dir:/results" \
        aimsracer-mpcc:humble python3 -m aims_mpcc.integration --scenario "$scenario" --output "/results/$scenario"
done
docker run --rm --network none -e ROS_DOMAIN_ID=83 -v "$results_dir:/results" \
    aimsracer-mpcc:humble python3 -m aims_mpcc.integration --clockwise --speed-tau .3 --steer-tau .22 --output /results/clockwise_mismatch
docker run --rm --network none -e ROS_DOMAIN_ID=83 -v "$results_dir:/results" \
    aimsracer-mpcc:humble python3 -m aims_mpcc.integration --odom-delay .06 --output /results/delayed_odometry
printf 'Results: %s\n' "$results_dir"
