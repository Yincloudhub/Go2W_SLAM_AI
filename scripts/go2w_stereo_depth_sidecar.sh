#!/usr/bin/env bash
set -euo pipefail

# Lightweight D435 ROI summary producer used by the motion safety gate.
# It keeps only one compact JSON value and never publishes raw frames.

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
SERVICE_DIR="${GO2W_STEREO_SERVICE_DIR:-${REPO_ROOT}/artifacts/stereo_depth_service}"
SUMMARY_PATH="${GO2W_STEREO_SUMMARY_PATH:-${REPO_ROOT}/artifacts/stereo_depth_summary.json}"
PID_FILE="${SERVICE_DIR}/producer.pid"
LOG_FILE="${SERVICE_DIR}/producer.log"
PYTHON_BIN="${GO2W_PYTHON:-python3}"
NICE_LEVEL="${GO2W_STEREO_NICE_LEVEL:-5}"
LOOP_INTERVAL_S="${GO2W_STEREO_LOOP_INTERVAL_S:-0.5}"
SAFETY_STALE_MS="${GO2W_STEREO_SAFETY_STALE_MS:-1000}"
INPUT_FPS="${GO2W_STEREO_INPUT_FPS:-15}"
FRAMES_PER_SAMPLE="${GO2W_STEREO_FRAMES_PER_SAMPLE:-1}"

mkdir -p "${SERVICE_DIR}"

read_pid() {
  [[ -f "${PID_FILE}" ]] || return 1
  tr -dc '0-9' < "${PID_FILE}"
}

pid_matches() {
  local pid="$1"
  [[ -n "${pid}" && -r "/proc/${pid}/cmdline" ]] || return 1
  tr '\0' ' ' < "/proc/${pid}/cmdline" | grep -F -- "realsense_depth_summary.py" >/dev/null
}

is_running() {
  local pid
  pid="$(read_pid 2>/dev/null || true)"
  [[ -n "${pid}" ]] && kill -0 "${pid}" 2>/dev/null && pid_matches "${pid}"
}

summary_health() {
  SUMMARY_PATH="${SUMMARY_PATH}" SAFETY_STALE_MS="${SAFETY_STALE_MS}" "${PYTHON_BIN}" - <<'PY'
import json
import os
import time
from pathlib import Path

path = Path(os.environ["SUMMARY_PATH"])
max_age_ms = max(1, int(os.environ["SAFETY_STALE_MS"]))
if not path.exists():
    print(f"summary_health=missing path={path}")
    raise SystemExit(1)
data = json.loads(path.read_text(encoding="utf-8"))
timestamp_ms = int(data.get("timestamp_ms") or 0)
age_ms = max(0, int(time.time() * 1000) - timestamp_ms) if timestamp_ms else -1
roi = data.get("roi_confidence") if isinstance(data.get("roi_confidence"), dict) else {}
fresh = timestamp_ms > 0 and age_ms <= max_age_ms and not bool(data.get("stale", False))
print(
    "summary_health="
    + ("ok" if fresh else "stale")
    + f" age_ms={age_ms} stale_ms={max_age_ms}"
    + f" front_m={data.get('front_clearance_m')} left_m={data.get('left_clearance_m')} right_m={data.get('right_clearance_m')}"
    + f" roi_front={roi.get('front')} roi_left={roi.get('left')} roi_right={roi.get('right')}"
)
raise SystemExit(0 if fresh else 1)
PY
}

print_status() {
  local pid
  pid="$(read_pid 2>/dev/null || true)"
  if is_running; then
    echo "stereo_depth=running pid=${pid}"
  else
    echo "stereo_depth=stopped"
  fi
  echo "summary_path=${SUMMARY_PATH}"
  summary_health || true
}

stop_sidecar() {
  local pid
  pid="$(read_pid 2>/dev/null || true)"
  if [[ -z "${pid}" ]]; then
    echo "stereo_depth=stopped"
    return 0
  fi
  if ! kill -0 "${pid}" 2>/dev/null; then
    rm -f "${PID_FILE}"
    echo "stereo_depth=stopped stale_pid=${pid}"
    return 0
  fi
  if ! pid_matches "${pid}"; then
    echo "stereo_depth=pid_mismatch pid=${pid}; refusing to signal" >&2
    return 1
  fi
  kill "${pid}"
  for _ in $(seq 1 30); do
    if ! kill -0 "${pid}" 2>/dev/null; then
      rm -f "${PID_FILE}"
      echo "stereo_depth=stopped pid=${pid}"
      return 0
    fi
    sleep 0.1
  done
  echo "stereo_depth=still_running pid=${pid}; inspect manually" >&2
  return 1
}

start_sidecar() {
  if is_running; then
    echo "stereo_depth=already_running"
    print_status
    return 0
  fi
  : > "${LOG_FILE}"
  nohup nice -n "${NICE_LEVEL}" "${PYTHON_BIN}" "${REPO_ROOT}/scripts/realsense_depth_summary.py" \
    --output "${SUMMARY_PATH}" \
    --fps "${INPUT_FPS}" \
    --frames "${FRAMES_PER_SAMPLE}" \
    --loop-interval-s "${LOOP_INTERVAL_S}" \
    --max-samples 0 \
    --quiet \
    > "${LOG_FILE}" 2>&1 < /dev/null &
  echo "$!" > "${PID_FILE}"
  sleep 2
  if ! is_running || ! summary_health; then
    echo "stereo_depth=start_failed" >&2
    tail -n 40 "${LOG_FILE}" >&2 || true
    stop_sidecar || true
    return 1
  fi
  print_status
}

case "${1:-status}" in
  start)
    start_sidecar
    ;;
  stop)
    stop_sidecar
    ;;
  restart)
    stop_sidecar
    start_sidecar
    ;;
  restart-if-stale)
    if is_running && summary_health >/dev/null; then
      echo "restart=not_needed"
      print_status
    else
      echo "restart=needed"
      stop_sidecar
      start_sidecar
    fi
    ;;
  status)
    print_status
    ;;
  health)
    is_running
    summary_health
    ;;
  *)
    echo "usage: $0 {start|stop|restart|restart-if-stale|status|health}" >&2
    exit 2
    ;;
esac
