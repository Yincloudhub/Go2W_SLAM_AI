#!/usr/bin/env bash
set -euo pipefail

# Robot-side runtime startup wrapper for the operator panel.
# It starts/checks PTP, LiDAR, SLAM, unified D435 perception, PerceptionContext,
# and the structured gateway probe. It does not send navigation, relocation,
# mapping, or chassis motion commands.

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
PYTHON_BIN="${GO2W_PYTHON:-python3}"
export GO2W_START_XT16_GEOMETRY="${GO2W_START_XT16_GEOMETRY:-1}"
export GO2W_XT16_GEOMETRY_CALIBRATED="${GO2W_XT16_GEOMETRY_CALIBRATED:-auto}"
export GO2W_START_STEREO_DEPTH="${GO2W_START_STEREO_DEPTH:-1}"

forwarded_args=()
include_slam_stack=1
for arg in "$@"; do
  case "${arg}" in
    --supervised-engineering-release)
      export GO2W_XT16_SUPERVISED_RELEASE=1
      ;;
    --no-slam-stack)
      include_slam_stack=0
      forwarded_args+=("${arg}")
      ;;
    *)
      forwarded_args+=("${arg}")
      ;;
  esac
done

if [[ "${include_slam_stack}" == "1" ]] &&
   [[ "${GO2W_AUTO_START_XT16_PTP:-1}" == "1" ]] &&
   ! bash "${REPO_ROOT}/scripts/go2w_xt16_ptp.sh" check >/dev/null 2>&1; then
  echo "XT16 PTP is not healthy; starting the local PTP manager (sudo may prompt)."
  if [[ "${EUID}" -eq 0 ]]; then
    bash "${REPO_ROOT}/scripts/go2w_xt16_ptp.sh" start
  elif [[ -t 0 ]]; then
    sudo bash "${REPO_ROOT}/scripts/go2w_xt16_ptp.sh" start
  else
    sudo -n bash "${REPO_ROOT}/scripts/go2w_xt16_ptp.sh" start
  fi
fi

exec "${PYTHON_BIN}" "${REPO_ROOT}/scripts/go2w_startup_supervisor.py" \
  --run \
  --start-slam-script "${REPO_ROOT}/scripts/start_go2w_slam_stack.sh" \
  --gateway-client "${GO2W_GATEWAY_CLIENT:-${REPO_ROOT}/robot/slam_gateway_refactor/build/slam_llm_command_client}" \
  --interface "${GO2W_NETWORK_INTERFACE:-eth0}" \
  "${forwarded_args[@]}"
