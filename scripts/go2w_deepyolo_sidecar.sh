#!/usr/bin/env bash
set -euo pipefail

# Optional robot-side DeepYOLO service. This sidecar never starts SLAM, sends
# navigation commands, or becomes a dependency of the deterministic runtime.

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
SRC_DIR="${GO2W_DEEPYOLO_SRC_DIR:-/home/unitree/librealsense/examples/DeepYolo_test}"
HEADLESS_BIN="${GO2W_DEEPYOLO_HEADLESS_BIN:-${SRC_DIR}/go2w_headless/build/yolo_test_realsense_headless}"
SERVICE_DIR="${GO2W_DEEPYOLO_SERVICE_DIR:-${REPO_ROOT}/artifacts/deepyolo_service}"
STREAM_DIR="${GO2W_DEEPYOLO_INPUT_DIR:-${SERVICE_DIR}/jsonl}"
SUMMARY_PATH="${GO2W_DEEPYOLO_SUMMARY_PATH:-${REPO_ROOT}/artifacts/vision_semantic_summary.json}"
DETECTOR_PID_FILE="${SERVICE_DIR}/detector.pid"
BRIDGE_PID_FILE="${SERVICE_DIR}/bridge.pid"
DETECTOR_LOG="${SERVICE_DIR}/detector.log"
BRIDGE_LOG="${SERVICE_DIR}/bridge.log"
NICE_LEVEL="${GO2W_DEEPYOLO_NICE_LEVEL:-5}"
RETAIN_STREAMS="${GO2W_DEEPYOLO_RETAIN_STREAMS:-4}"

mkdir -p "${SERVICE_DIR}" "${STREAM_DIR}"

read_pid() {
  local pid_file="$1"
  [[ -f "${pid_file}" ]] || return 1
  tr -dc '0-9' < "${pid_file}"
}

pid_matches() {
  local pid="$1"
  local expected="$2"
  [[ -n "${pid}" && -r "/proc/${pid}/cmdline" ]] || return 1
  tr '\0' ' ' < "/proc/${pid}/cmdline" | grep -F -- "${expected}" >/dev/null
}

is_running() {
  local pid_file="$1"
  local expected="$2"
  local pid
  pid="$(read_pid "${pid_file}" 2>/dev/null || true)"
  [[ -n "${pid}" ]] && kill -0 "${pid}" 2>/dev/null && pid_matches "${pid}" "${expected}"
}

prune_streams() {
  STREAM_DIR="${STREAM_DIR}" RETAIN_STREAMS="${RETAIN_STREAMS}" python3 - <<'PY'
import os
from pathlib import Path

stream_dir = Path(os.environ["STREAM_DIR"]).resolve()
retain = max(1, int(os.environ["RETAIN_STREAMS"]))
for path in sorted(stream_dir.glob("semantic_stream_*.jsonl"), key=lambda p: p.stat().st_mtime_ns, reverse=True)[retain:]:
    resolved = path.resolve()
    if resolved.parent != stream_dir:
        raise SystemExit(f"refusing to prune outside stream dir: {resolved}")
    resolved.unlink()
PY
}

stop_one() {
  local label="$1"
  local pid_file="$2"
  local expected="$3"
  local pid
  pid="$(read_pid "${pid_file}" 2>/dev/null || true)"
  if [[ -z "${pid}" ]]; then
    echo "${label}=stopped"
    return 0
  fi
  if ! kill -0 "${pid}" 2>/dev/null; then
    rm -f "${pid_file}"
    echo "${label}=stopped stale_pid=${pid}"
    return 0
  fi
  if ! pid_matches "${pid}" "${expected}"; then
    echo "${label}=pid_mismatch pid=${pid}; refusing to signal" >&2
    return 1
  fi
  kill "${pid}"
  for _ in $(seq 1 30); do
    if ! kill -0 "${pid}" 2>/dev/null; then
      rm -f "${pid_file}"
      echo "${label}=stopped pid=${pid}"
      return 0
    fi
    sleep 0.1
  done
  echo "${label}=still_running pid=${pid}; send SIGKILL manually after inspection" >&2
  return 1
}

print_status() {
  local detector_pid bridge_pid
  detector_pid="$(read_pid "${DETECTOR_PID_FILE}" 2>/dev/null || true)"
  bridge_pid="$(read_pid "${BRIDGE_PID_FILE}" 2>/dev/null || true)"
  if is_running "${DETECTOR_PID_FILE}" "${HEADLESS_BIN}"; then
    echo "detector=running pid=${detector_pid}"
  else
    echo "detector=stopped"
  fi
  if is_running "${BRIDGE_PID_FILE}" "deepyolo_semantic_bridge.py"; then
    echo "bridge=running pid=${bridge_pid}"
  else
    echo "bridge=stopped"
  fi
  echo "stream_dir=${STREAM_DIR}"
  echo "summary_path=${SUMMARY_PATH}"
  if [[ -f "${SUMMARY_PATH}" ]]; then
    SUMMARY_PATH="${SUMMARY_PATH}" python3 - <<'PY'
import json
import os
from pathlib import Path

data = json.loads(Path(os.environ["SUMMARY_PATH"]).read_text(encoding="utf-8"))
keys = ("source_status", "stale", "effective_action", "recommended_action", "dominant_class", "object_count")
print("summary=" + " ".join(f"{key}={data.get(key)}" for key in keys))
PY
  fi
}

case "${1:-status}" in
  build)
    exec bash "${SCRIPT_DIR}/build_deepyolo_headless.sh"
    ;;
  start)
    if [[ ! -x "${HEADLESS_BIN}" ]]; then
      echo "missing headless binary: ${HEADLESS_BIN}" >&2
      echo "run: bash scripts/go2w_deepyolo_sidecar.sh build" >&2
      exit 2
    fi
    if is_running "${DETECTOR_PID_FILE}" "${HEADLESS_BIN}" || is_running "${BRIDGE_PID_FILE}" "deepyolo_semantic_bridge.py"; then
      echo "DeepYOLO sidecar is already partially or fully running; inspect status first" >&2
      print_status
      exit 1
    fi
    prune_streams
    : > "${DETECTOR_LOG}"
    : > "${BRIDGE_LOG}"
    nohup nice -n "${NICE_LEVEL}" env \
      GO2W_DEEPYOLO_OUTPUT_DIR="${STREAM_DIR}" \
      "${HEADLESS_BIN}" > "${DETECTOR_LOG}" 2>&1 < /dev/null &
    echo "$!" > "${DETECTOR_PID_FILE}"
    nohup nice -n "${NICE_LEVEL}" env \
      GO2W_DEEPYOLO_INPUT_DIR="${STREAM_DIR}" \
      GO2W_DEEPYOLO_SUMMARY_PATH="${SUMMARY_PATH}" \
      bash "${SCRIPT_DIR}/start_go2w_deepyolo_bridge.sh" > "${BRIDGE_LOG}" 2>&1 < /dev/null &
    echo "$!" > "${BRIDGE_PID_FILE}"
    sleep 1
    print_status
    ;;
  stop)
    stop_one "bridge" "${BRIDGE_PID_FILE}" "deepyolo_semantic_bridge.py"
    stop_one "detector" "${DETECTOR_PID_FILE}" "${HEADLESS_BIN}"
    prune_streams
    ;;
  restart)
    "$0" stop
    exec "$0" start
    ;;
  status)
    print_status
    ;;
  *)
    echo "usage: $0 {build|start|status|stop|restart}" >&2
    exit 2
    ;;
esac
