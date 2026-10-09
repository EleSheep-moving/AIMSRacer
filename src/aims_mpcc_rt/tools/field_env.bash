# Source from a bash terminal on the tested NX. This file only sets environment;
# it does not launch nodes, enable control, change clocks or publish commands.
if [ -z "${BASH_VERSION:-}" ]; then
  printf 'Run bash before sourcing field_env.bash.\n' >&2
  return 2
fi
if [[ "${BASH_SOURCE[0]}" == "$0" ]]; then
  printf 'Use source field_env.bash to configure this terminal.\n' >&2
  exit 2
fi

export MPCC_REPO
MPCC_REPO="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../../.." && pwd -P)"
export MPCC_DATA_ROOT=/home/aims/aimsracer-data
export MPCC_QUAL_ROOT="$MPCC_DATA_ROOT/experiments/mpcc-acados-runtime/repair-final-20261009"
export MPCC_INTERFACE_ROOT="$MPCC_DATA_ROOT/experiments/mpcc-acados-runtime/remove-shadow-20261010"
export MPCC_MAP=/home/aims/maps/20260928_010503/map.pcd
export MPCC_BUNDLE05="$MPCC_QUAL_ROOT/bundles/qual-v2-field-v05-n10"
export MPCC_BUNDLE10="$MPCC_QUAL_ROOT/bundles/qual-v2-field-v10-n10"

source /opt/ros/humble/setup.bash || return
source /home/aims/AIMSRacer/install/setup.bash || return
source /home/aims/AIMSRacer-fastlio-ndt/log/fastlio-ndt/install/ndt_omp_ros2/share/ndt_omp_ros2/package.bash || return
source /home/aims/AIMSRacer-fastlio-ndt/log/fastlio-ndt/install/lidar_localization_ros2/share/lidar_localization_ros2/package.bash || return
source "$MPCC_QUAL_ROOT/ros-install/local_setup.bash" || return
source "$MPCC_INTERFACE_ROOT/ros-install/local_setup.bash" || return
export LD_LIBRARY_PATH="$MPCC_DATA_ROOT/experiments/mpcc-acados-runtime/deps/acados/install-runtime/lib:${LD_LIBRARY_PATH:-}"
export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=0
