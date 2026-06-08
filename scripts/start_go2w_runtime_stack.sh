#!/usr/bin/env bash
set -euo pipefail

# Robot-side runtime startup wrapper for the operator panel.
# It starts/checks only LiDAR, SLAM, and the structured gateway probe. It does
# not send navigation, relocation, mapping, or chassis motion commands.

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
PYTHON_BIN="${GO2W_PYTHON:-python3}"
export GO2W_START_XT16_GEOMETRY="${GO2W_START_XT16_GEOMETRY:-1}"
export GO2W_XT16_GEOMETRY_CALIBRATED="${GO2W_XT16_GEOMETRY_CALIBRATED:-0}"
export GO2W_START_STEREO_DEPTH="${GO2W_START_STEREO_DEPTH:-0}"

exec "${PYTHON_BIN}" "${REPO_ROOT}/scripts/go2w_startup_supervisor.py" \
  --run \
  --start-slam-script "${REPO_ROOT}/scripts/start_go2w_slam_stack.sh" \
  --gateway-client "${GO2W_GATEWAY_CLIENT:-${REPO_ROOT}/robot/slam_gateway_refactor/build/slam_llm_command_client}" \
  --interface "${GO2W_NETWORK_INTERFACE:-eth0}" \
  "$@"
