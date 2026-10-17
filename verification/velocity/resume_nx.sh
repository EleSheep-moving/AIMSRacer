#!/usr/bin/env bash
# Reproduce in the dedicated observation workspace; never replace vehicle install.
set -euo pipefail
if [[ $# != 1 ]]; then
  echo 'Usage: VELOCITY_PYTHON=/path/to/scientific/python3 bash verification/velocity/resume_nx.sh /new/external/result-dir' >&2
  exit 2
fi
vx_repo=$(cd "$(dirname "$0")/../.." && pwd)
vx_data=${VELOCITY_DATA_ROOT:-/home/elesheep/aimsracer-data/experiments/vx-observer-20261015}
vx_python=${VELOCITY_PYTHON:-python3}
vx_ssh=${VELOCITY_NX_SSH:-aims-nx}
vx_result=$1
ssh -o ConnectTimeout=5 "$vx_ssh" 'test "$(uname -m)" = aarch64'
mkdir "$vx_result"
ssh "$vx_ssh" 'mkdir -p /home/aims/aimsracer-vx-observer-20261015/src/aims_mpcc_rt /home/aims/aimsracer-vx-observer-20261015/verification /home/aims/aimsracer-vx-observer-20261015/inputs /home/aims/aimsracer-vx-observer-20261015/baseline'
rsync -a --exclude=__pycache__ "$vx_repo/src/aims_racer_system/" "$vx_ssh:/home/aims/aimsracer-vx-observer-20261015/src/aims_racer_system/"
rsync -a "$vx_repo/src/aims_mpcc_rt/launch/" "$vx_ssh:/home/aims/aimsracer-vx-observer-20261015/src/aims_mpcc_rt/launch/"
rsync -a --exclude=__pycache__ --exclude='*.tmp*' "$vx_repo/verification/" "$vx_ssh:/home/aims/aimsracer-vx-observer-20261015/verification/"
for vx_name in B C mpcc; do
  rsync -a "$vx_data/accepted/offline/$vx_name/events.txt" "$vx_ssh:/home/aims/aimsracer-vx-observer-20261015/inputs/$vx_name.txt"
  rsync -a "$vx_data/accepted/native-x86/$vx_name.csv" "$vx_ssh:/home/aims/aimsracer-vx-observer-20261015/baseline/$vx_name.csv"
done
rsync -a "$vx_data/inputs/mpcc/input.npz" "$vx_ssh:/home/aims/aimsracer-vx-observer-20261015/inputs/mpcc.npz"
ssh "$vx_ssh" bash -s > "$vx_result/nx.log" 2>&1 <<'REMOTE'
set -eo pipefail
source /opt/ros/humble/setup.bash
cd /home/aims/aimsracer-vx-observer-20261015
vx_run="runs/$(date -u +%Y%m%dT%H%M%S)"
mkdir -p "$vx_run"
printf '%s\n' "$vx_run" > latest-run.txt
uname -a > "$vx_run/host.txt"
cmake -S src/aims_racer_system -B build-final -DCMAKE_INSTALL_PREFIX="$PWD/install-final" -DBUILD_TESTING=ON
cmake --build build-final -j2
cmake --install build-final
source install-final/share/aims_racer_system/local_setup.bash
export PYTHONPATH="$PWD/src/aims_racer_system:${PYTHONPATH:-}"
ctest --test-dir build-final --output-on-failure -j1 | tee "$vx_run/ctest.log"
for vx_name in B C mpcc; do
  build-final/velocity_trace "inputs/$vx_name.txt" > "$vx_run/$vx_name.csv"
  /usr/bin/python3 verification/velocity/compare.py "baseline/$vx_name.csv" "$vx_run/$vx_name.csv" > "$vx_run/$vx_name-parity.json"
done
export ROS_DOMAIN_ID=200 ROS_LOCALHOST_ONLY=1 OPENBLAS_NUM_THREADS=1 ROS_LOG_DIR="$PWD/$vx_run/ros-logs"
/usr/bin/python3 verification/velocity/ros_replay.py --input inputs/mpcc.npz --expected baseline/mpcc.csv --executable "$PWD/build-final/vx_observer" --output "$vx_run/ros-replay"
sha256sum build-final/vx_observer build-final/velocity_trace > "$vx_run/binary-sha256.txt"
sha256sum /home/aims/aimsracer-data/sessions/2026-10-15/022604_194643_race/bag/bag_0.db3 /home/aims/aimsracer-data/sessions/2026-10-14/233659-B-full/bag/bag_0.db3 /home/aims/aimsracer-data/sessions/2026-10-15/015017-C-resume-minus60A-odom30cm/bag/bag_0.db3 > "$vx_run/bag-sha256.txt"
REMOTE
rsync -a "$vx_ssh:/home/aims/aimsracer-vx-observer-20261015/runs/" "$vx_result/runs/"
echo "NX evidence: $vx_result"
