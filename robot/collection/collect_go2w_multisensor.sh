#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

source /opt/ros/foxy/setup.bash
if [[ -f "${HOME}/cyclonedds_ws/install/local_setup.bash" ]]; then
  source "${HOME}/cyclonedds_ws/install/local_setup.bash"
fi

export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
unset CYCLONEDDS_URI

mkdir -p "${HOME}/go2w_dataset/bags"
exec python3 "${SCRIPT_DIR}/collect_go2w_multisensor.py" "$@"
