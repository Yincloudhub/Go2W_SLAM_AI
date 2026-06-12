#!/usr/bin/env bash
set -euo pipefail

# Robot-side browser UI launcher. It builds the C++ operator panel if needed and
# starts a thin Web UI that delegates commands back to that panel.

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
BUILD_DIR="${GO2W_CPP_BUILD_DIR:-${REPO_ROOT}/cpp/build}"
PANEL_BIN="${GO2W_OPERATOR_PANEL_BIN:-${BUILD_DIR}/go2w_operator_panel}"
CURRENT_NODE="${GO2W_CURRENT_NODE:-initial_point}"
REGISTRY_PATH="${GO2W_REGISTRY:-${REPO_ROOT}/configs/maps/go2w_real_site_map_registry.json}"
MAP_ID="${GO2W_MAP_ID:-go2w_real_site}"
GATEWAY_CLIENT="${GO2W_GATEWAY_CLIENT:-${REPO_ROOT}/robot/slam_gateway_refactor/build/slam_llm_command_client}"
NETWORK_INTERFACE="${GO2W_NETWORK_INTERFACE:-eth0}"
START_SLAM_SCRIPT="${GO2W_START_SLAM_SCRIPT:-${REPO_ROOT}/scripts/start_go2w_slam_stack.sh}"
START_RVIZ2_SCRIPT="${GO2W_START_RVIZ2_SCRIPT:-${REPO_ROOT}/scripts/start_go2w_rviz2.sh}"
WEB_HOST="${GO2W_WEB_HOST:-127.0.0.1}"
WEB_PORT="${GO2W_WEB_PORT:-8765}"
export GO2W_STEREO_SUMMARY_PATH="${GO2W_STEREO_SUMMARY_PATH:-${REPO_ROOT}/artifacts/stereo_depth_summary.json}"
export GO2W_LIDAR_GEOMETRY_SUMMARY_PATH="${GO2W_LIDAR_GEOMETRY_SUMMARY_PATH:-${REPO_ROOT}/artifacts/lidar_geometry_summary.json}"
export GO2W_COLLECTION_STATUS_PATH="${GO2W_COLLECTION_STATUS_PATH:-${HOME}/go2w_dataset/collection_status.json}"
export GO2W_STEREO_SAFETY_STALE_MS="${GO2W_STEREO_SAFETY_STALE_MS:-1000}"
export GO2W_START_XT16_GEOMETRY="${GO2W_START_XT16_GEOMETRY:-1}"
export GO2W_XT16_GEOMETRY_CALIBRATED="${GO2W_XT16_GEOMETRY_CALIBRATED:-0}"
export GO2W_START_STEREO_DEPTH="${GO2W_START_STEREO_DEPTH:-0}"

if [[ ! -x "${PANEL_BIN}" ]]; then
  if ! command -v cmake >/dev/null 2>&1; then
    echo "missing cmake and ${PANEL_BIN} is not built" >&2
    exit 2
  fi
  cmake -S "${REPO_ROOT}/cpp" -B "${BUILD_DIR}"
  cmake --build "${BUILD_DIR}" -j"$(nproc 2>/dev/null || echo 2)"
fi

LLM_HTTP_ARGS=()
if [[ -n "${GO2W_LLM_HTTP_URL:-}" ]]; then
  LLM_HTTP_ARGS+=(--llm-http-url "${GO2W_LLM_HTTP_URL}")
  LLM_HTTP_ARGS+=(--llm-http-model "${GO2W_LLM_HTTP_MODEL:-local}")
fi

exec "${PYTHON:-python3}" "${REPO_ROOT}/scripts/go2w_operator_web.py" \
  --repo-root "${REPO_ROOT}" \
  --panel-bin "${PANEL_BIN}" \
  --registry "${REGISTRY_PATH}" \
  --map-id "${MAP_ID}" \
  --gateway-client "${GATEWAY_CLIENT}" \
  --start-slam-script "${START_SLAM_SCRIPT}" \
  --start-rviz2-script "${START_RVIZ2_SCRIPT}" \
  --interface "${NETWORK_INTERFACE}" \
  --current-node "${CURRENT_NODE}" \
  --host "${WEB_HOST}" \
  --port "${WEB_PORT}" \
  --ensure-slam-on-start \
  "${LLM_HTTP_ARGS[@]}" \
  "$@"
