#!/usr/bin/env bash
set -euo pipefail

# Start RViz2 as a detached visual diagnostics process. This script does not
# publish motion commands or call Unitree APIs.

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
LOG_DIR="${GO2W_RVIZ2_LOG_DIR:-${REPO_ROOT}/artifacts/rviz2}"
RVIZ2_BIN="${GO2W_RVIZ2_BIN:-rviz2}"
RVIZ2_CONFIG="${GO2W_RVIZ2_CONFIG:-}"

mkdir -p "${LOG_DIR}"

if [[ -z "${DISPLAY:-}" && -z "${WAYLAND_DISPLAY:-}" ]]; then
  if [[ -S /tmp/.X11-unix/X0 ]]; then
    export DISPLAY="${GO2W_RVIZ2_DISPLAY:-:0}"
    if [[ -z "${XAUTHORITY:-}" && -f "${HOME}/.Xauthority" ]]; then
      export XAUTHORITY="${HOME}/.Xauthority"
    fi
    echo "rviz2_display_auto=${DISPLAY}"
    if [[ -n "${XAUTHORITY:-}" ]]; then
      echo "rviz2_xauthority=${XAUTHORITY}"
    fi
  else
    echo "DISPLAY/WAYLAND_DISPLAY is not set; enable X11 forwarding or run from a desktop session." >&2
    exit 3
  fi
fi

set +u
source /opt/ros/foxy/setup.bash >/dev/null 2>&1 || true
if [[ -f "${REPO_ROOT}/install/setup.bash" ]]; then
  source "${REPO_ROOT}/install/setup.bash" >/dev/null 2>&1 || true
fi
set -u

if ! command -v "${RVIZ2_BIN}" >/dev/null 2>&1; then
  echo "rviz2 executable not found: ${RVIZ2_BIN}" >&2
  exit 2
fi

args=()
if [[ -n "${RVIZ2_CONFIG}" ]]; then
  args+=("-d" "${RVIZ2_CONFIG}")
fi

log_file="${LOG_DIR}/rviz2_$(date +%Y%m%d_%H%M%S).log"
nohup "${RVIZ2_BIN}" "${args[@]}" >"${log_file}" 2>&1 &
pid="$!"

echo "rviz2_started=true"
echo "pid=${pid}"
echo "log=${log_file}"
