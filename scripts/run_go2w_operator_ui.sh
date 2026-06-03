#!/usr/bin/env bash
set -euo pipefail

# One-command robot-side operator UI launcher.
# Run this from an SSH terminal on the GO2W/NX. It builds the C++ panel if needed,
# starts/checks the LiDAR driver and SLAM stack, then opens the terminal UI.

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
BUILD_DIR="${GO2W_CPP_BUILD_DIR:-${REPO_ROOT}/cpp/build}"
CURRENT_NODE="${GO2W_CURRENT_NODE:-initial_point}"
REGISTRY_PATH="${GO2W_REGISTRY:-${REPO_ROOT}/configs/maps/go2w_real_site_map_registry.json}"
MAP_ID="${GO2W_MAP_ID:-go2w_real_site}"
GATEWAY_CLIENT="${GO2W_GATEWAY_CLIENT:-/home/unitree/slam_gateway_refactor/build/slam_llm_command_client}"
NETWORK_INTERFACE="${GO2W_NETWORK_INTERFACE:-eth0}"
START_SLAM_SCRIPT="${GO2W_START_SLAM_SCRIPT:-${REPO_ROOT}/scripts/start_go2w_slam_stack.sh}"
START_RVIZ2_SCRIPT="${GO2W_START_RVIZ2_SCRIPT:-${REPO_ROOT}/scripts/start_go2w_rviz2.sh}"
LLM_HTTP_ARGS=()
if [[ -n "${GO2W_LLM_HTTP_URL:-}" ]]; then
  LLM_HTTP_ARGS+=(--llm-http-url "${GO2W_LLM_HTTP_URL}")
  LLM_HTTP_ARGS+=(--llm-http-model "${GO2W_LLM_HTTP_MODEL:-local}")
fi

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
  --registry "${REGISTRY_PATH}" \
  --map-id "${MAP_ID}" \
  --gateway-client "${GATEWAY_CLIENT}" \
  --start-slam-script "${START_SLAM_SCRIPT}" \
  --start-rviz2-script "${START_RVIZ2_SCRIPT}" \
  --interface "${NETWORK_INTERFACE}" \
  --current-node "${CURRENT_NODE}" \
  --ensure-slam-on-start \
  "${LLM_HTTP_ARGS[@]}" \
  "$@"
