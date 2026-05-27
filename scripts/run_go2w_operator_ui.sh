#!/usr/bin/env bash
set -euo pipefail

# One-command robot-side operator UI launcher.
# Run this from an SSH terminal on the GO2W/NX. It builds the C++ panel if needed,
# starts/checks the LiDAR driver and SLAM stack, then opens the terminal UI.

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
BUILD_DIR="${GO2W_CPP_BUILD_DIR:-${REPO_ROOT}/cpp/build}"
CURRENT_NODE="${GO2W_CURRENT_NODE:-initial_point}"
GATEWAY_CLIENT="${GO2W_GATEWAY_CLIENT:-/home/unitree/slam_gateway_refactor/build/slam_llm_command_client}"
NETWORK_INTERFACE="${GO2W_NETWORK_INTERFACE:-eth0}"

if [[ ! -x "${BUILD_DIR}/go2w_operator_panel" ]]; then
  if ! command -v cmake >/dev/null 2>&1; then
    echo "missing cmake and ${BUILD_DIR}/go2w_operator_panel is not built" >&2
    exit 2
  fi
  cmake -S "${REPO_ROOT}/cpp" -B "${BUILD_DIR}"
  cmake --build "${BUILD_DIR}" -j"$(nproc 2>/dev/null || echo 2)"
fi

exec "${BUILD_DIR}/go2w_operator_panel" \
  --repo-root "${REPO_ROOT}" \
  --gateway-client "${GATEWAY_CLIENT}" \
  --interface "${NETWORK_INTERFACE}" \
  --current-node "${CURRENT_NODE}" \
  --ensure-slam-on-start \
  "$@"
