#!/usr/bin/env bash
set -euo pipefail

# Competition D435 service: one RGBD owner with independent depth and YOLO reducers.

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
SRC_DIR="${GO2W_DEEPYOLO_SRC_DIR:-/home/unitree/librealsense/examples/DeepYolo_test}"
HEADLESS_BIN="${GO2W_DEEPYOLO_HEADLESS_BIN:-${SRC_DIR}/go2w_headless/build/yolo_test_realsense_headless}"
SERVICE_DIR="${GO2W_D435_SERVICE_DIR:-${REPO_ROOT}/artifacts/d435_perception_service}"
STREAM_DIR="${GO2W_D435_STREAM_DIR:-${SERVICE_DIR}/jsonl}"
DEPTH_PACKET="${GO2W_D435_DEPTH_PACKET_PATH:-${SERVICE_DIR}/depth_packet.json}"
SUMMARY_PATH="${GO2W_D435_SUMMARY_PATH:-${REPO_ROOT}/artifacts/d435_perception_summary.json}"
DEPTH_COMPAT_PATH="${GO2W_STEREO_SUMMARY_PATH:-${REPO_ROOT}/artifacts/stereo_depth_summary.json}"
YOLO_COMPAT_PATH="${GO2W_DEEPYOLO_SUMMARY_PATH:-${REPO_ROOT}/artifacts/vision_semantic_summary.json}"
CAPTURE_PID_FILE="${SERVICE_DIR}/capture_owner.pid"
REDUCER_PID_FILE="${SERVICE_DIR}/summary_reducer.pid"
SUPERVISOR_PID_FILE="${SERVICE_DIR}/supervisor.pid"
LOCK_FILE="${SERVICE_DIR}/manager.lock"
CAPTURE_LOG="${SERVICE_DIR}/capture_owner.log"
REDUCER_LOG="${SERVICE_DIR}/summary_reducer.log"
SUPERVISOR_LOG="${SERVICE_DIR}/supervisor.jsonl"
SUPERVISOR_SCRIPT="${SCRIPT_DIR}/go2w_d435_supervisor.py"
MANAGER_SCRIPT="${SCRIPT_DIR}/go2w_d435_perception_sidecar.sh"
PROFILE="${GO2W_D435_PROFILE:-${GO2W_DEEPYOLO_PROFILE:-resident}}"
INPUT_FPS="${GO2W_D435_INPUT_FPS:-15}"
DEPTH_EVERY_N="${GO2W_D435_DEPTH_EVERY_N:-2}"
STARTUP_WAIT_S="${GO2W_D435_STARTUP_WAIT_S:-8}"
SUPERVISOR_INTERVAL_S="${GO2W_D435_SUPERVISOR_INTERVAL_S:-2}"
SUPERVISOR_FAILURE_THRESHOLD="${GO2W_D435_SUPERVISOR_FAILURE_THRESHOLD:-3}"
SUPERVISOR_COOLDOWN_S="${GO2W_D435_SUPERVISOR_COOLDOWN_S:-30}"

case "${PROFILE}" in
  resident) CAPTURE_EVERY_N=5; INFERENCE_INTERVAL_MS=333; NICE_LEVEL=8 ;;
  balanced) CAPTURE_EVERY_N=3; INFERENCE_INTERVAL_MS=200; NICE_LEVEL=5 ;;
  diagnostic) CAPTURE_EVERY_N=1; INFERENCE_INTERVAL_MS=0; NICE_LEVEL=0 ;;
  *) echo "invalid GO2W_D435_PROFILE=${PROFILE}" >&2; exit 2 ;;
esac
CAPTURE_EVERY_N="${GO2W_DEEPYOLO_CAPTURE_EVERY_N:-${CAPTURE_EVERY_N}}"
INFERENCE_INTERVAL_MS="${GO2W_DEEPYOLO_INFERENCE_INTERVAL_MS:-${INFERENCE_INTERVAL_MS}}"
NICE_LEVEL="${GO2W_D435_NICE_LEVEL:-${NICE_LEVEL}}"

mkdir -p "${SERVICE_DIR}" "${STREAM_DIR}"

read_pid() {
  local path="$1"
  [[ -f "${path}" ]] || return 1
  tr -dc '0-9' < "${path}"
}

pid_matches() {
  local pid="$1" expected="$2"
  [[ -n "${pid}" && -r "/proc/${pid}/cmdline" ]] || return 1
  tr '\0' ' ' < "/proc/${pid}/cmdline" | grep -F -- "${expected}" >/dev/null
}

is_running() {
  local path="$1" expected="$2" pid
  pid="$(read_pid "${path}" 2>/dev/null || true)"
  [[ -n "${pid}" ]] && kill -0 "${pid}" 2>/dev/null && pid_matches "${pid}" "${expected}"
}

acquire_operation_lock() {
  command -v flock >/dev/null 2>&1 || {
    echo "missing required command: flock" >&2
    return 2
  }
  exec 9>"${LOCK_FILE}"
  flock -n 9 || {
    echo "D435 manager operation already in progress" >&2
    return 1
  }
}

refuse_competing_owner() {
  local proc cmd comm managed_pid dev holder
  managed_pid="$(read_pid "${CAPTURE_PID_FILE}" 2>/dev/null || true)"
  if command -v fuser >/dev/null 2>&1; then
    for dev in /dev/video*; do
      [[ -e "${dev}" ]] || continue
      for holder in $(fuser "${dev}" 2>/dev/null || true); do
        if [[ -n "${holder}" && "${holder}" != "${managed_pid}" && -r "/proc/${holder}/cmdline" ]]; then
          cmd="$(tr '\0' ' ' < "/proc/${holder}/cmdline")"
          echo "D435/video device already held: device=${dev} pid=${holder} cmd=${cmd}" >&2
          return 1
        fi
      done
    done
  fi
  for proc in /proc/[0-9]*; do
    [[ -r "${proc}/cmdline" ]] || continue
    cmd="$(tr '\0' ' ' < "${proc}/cmdline")"
    comm="$(cat "${proc}/comm" 2>/dev/null || true)"
    if [[ "${cmd}" == *"realsense_depth_summary.py"* ]]; then
      echo "competing D435 owner detected: pid=${proc##*/} cmd=${cmd}" >&2
      return 1
    fi
    if [[ "${comm}" == yolo_test_real* && "${proc##*/}" != "${managed_pid}" ]]; then
      echo "competing D435 owner detected: pid=${proc##*/} cmd=${cmd}" >&2
      return 1
    fi
  done
}

summary_health() {
  local capture_pid
  capture_pid="$(read_pid "${CAPTURE_PID_FILE}" 2>/dev/null || true)"
  [[ -n "${capture_pid}" ]] || return 1
  python3 - "${SUMMARY_PATH}" "${capture_pid}" <<'PY'
import json
import sys
import time

try:
    with open(sys.argv[1], encoding="utf-8") as handle:
        data = json.load(handle)
except (OSError, ValueError):
    raise SystemExit(1)

now_ms = int(time.time() * 1000)
timestamp_ms = data.get("timestamp_ms")
owner = data.get("owner") if isinstance(data.get("owner"), dict) else {}
capture = data.get("capture") if isinstance(data.get("capture"), dict) else {}
healthy = (
    data.get("status") == "fresh"
    and data.get("stale") is False
    and isinstance(timestamp_ms, int)
    and 0 <= now_ms - timestamp_ms <= 1000
    and owner.get("running") is True
    and owner.get("pid") == int(sys.argv[2])
    and capture.get("status") == "fresh"
)
raise SystemExit(0 if healthy else 1)
PY
}

health_sidecar() {
  is_running "${CAPTURE_PID_FILE}" "${HEADLESS_BIN}"
  is_running "${REDUCER_PID_FILE}" "d435_perception_summary.py"
  refuse_competing_owner
  summary_health
}

supervisor_running() {
  is_running "${SUPERVISOR_PID_FILE}" "${SUPERVISOR_SCRIPT}"
}

start_supervisor() {
  local pid
  if supervisor_running; then
    echo "d435_supervisor=already_running pid=$(read_pid "${SUPERVISOR_PID_FILE}")"
    return 0
  fi
  rm -f "${SUPERVISOR_PID_FILE}"
  nohup python3 "${SUPERVISOR_SCRIPT}" \
    --manager "${MANAGER_SCRIPT}" \
    --summary-path "${SUMMARY_PATH}" \
    --interval-s "${SUPERVISOR_INTERVAL_S}" \
    --failure-threshold "${SUPERVISOR_FAILURE_THRESHOLD}" \
    --cooldown-s "${SUPERVISOR_COOLDOWN_S}" \
    >> "${SUPERVISOR_LOG}" 2>&1 < /dev/null 9>&- &
  pid=$!
  echo "${pid}" > "${SUPERVISOR_PID_FILE}"
  sleep 0.2
  if ! supervisor_running; then
    echo "d435_supervisor=start_failed pid=${pid}" >&2
    rm -f "${SUPERVISOR_PID_FILE}"
    return 1
  fi
  echo "d435_supervisor=started pid=${pid}"
}

stop_supervisor() {
  stop_one "d435_supervisor" "${SUPERVISOR_PID_FILE}" "${SUPERVISOR_SCRIPT}"
}

stop_one() {
  local label="$1" path="$2" expected="$3" pid
  pid="$(read_pid "${path}" 2>/dev/null || true)"
  if [[ -z "${pid}" || ! -e "/proc/${pid}" ]]; then
    rm -f "${path}"
    echo "${label}=stopped"
    return 0
  fi
  if ! pid_matches "${pid}" "${expected}"; then
    echo "${label}=pid_mismatch pid=${pid}; refusing to signal" >&2
    return 1
  fi
  kill "${pid}"
  for _ in $(seq 1 50); do
    if ! kill -0 "${pid}" 2>/dev/null; then
      rm -f "${path}"
      echo "${label}=stopped pid=${pid}"
      return 0
    fi
    sleep 0.1
  done
  echo "${label}=still_running pid=${pid}" >&2
  return 1
}

stop_sidecar() {
  local rc=0
  if ! stop_one "reducer" "${REDUCER_PID_FILE}" "d435_perception_summary.py"; then
    rc=1
  fi
  if ! stop_one "capture_owner" "${CAPTURE_PID_FILE}" "${HEADLESS_BIN}"; then
    rc=1
  fi
  return "${rc}"
}

print_status() {
  local capture_pid reducer_pid supervisor_pid
  capture_pid="$(read_pid "${CAPTURE_PID_FILE}" 2>/dev/null || true)"
  reducer_pid="$(read_pid "${REDUCER_PID_FILE}" 2>/dev/null || true)"
  supervisor_pid="$(read_pid "${SUPERVISOR_PID_FILE}" 2>/dev/null || true)"
  is_running "${CAPTURE_PID_FILE}" "${HEADLESS_BIN}" \
    && echo "d435_capture_owner=running pid=${capture_pid}" \
    || echo "d435_capture_owner=stopped"
  is_running "${REDUCER_PID_FILE}" "d435_perception_summary.py" \
    && echo "d435_summary_reducer=running pid=${reducer_pid}" \
    || echo "d435_summary_reducer=stopped"
  supervisor_running \
    && echo "d435_supervisor=running pid=${supervisor_pid}" \
    || echo "d435_supervisor=stopped"
  echo "summary_path=${SUMMARY_PATH}"
}

start_sidecar() {
  [[ -x "${HEADLESS_BIN}" ]] || { echo "missing headless binary: ${HEADLESS_BIN}" >&2; exit 2; }
  if is_running "${CAPTURE_PID_FILE}" "${HEADLESS_BIN}" || is_running "${REDUCER_PID_FILE}" "d435_perception_summary.py"; then
    echo "D435 sidecar is already partially or fully running" >&2
    print_status
    exit 1
  fi
  refuse_competing_owner
  : > "${CAPTURE_LOG}"
  : > "${REDUCER_LOG}"
  nohup nice -n "${NICE_LEVEL}" env \
    GO2W_DEEPYOLO_OUTPUT_DIR="${STREAM_DIR}" \
    GO2W_DEEPYOLO_INPUT_FPS="${INPUT_FPS}" \
    GO2W_DEEPYOLO_IR_MODE=0 \
    GO2W_DEEPYOLO_RENDER_OVERLAY=0 \
    GO2W_DEEPYOLO_CAPTURE_EVERY_N="${CAPTURE_EVERY_N}" \
    GO2W_DEEPYOLO_INFERENCE_INTERVAL_MS="${INFERENCE_INTERVAL_MS}" \
    GO2W_DEEPYOLO_HEARTBEAT_MS=1000 \
    GO2W_D435_DEPTH_EVERY_N="${DEPTH_EVERY_N}" \
    GO2W_D435_DEPTH_PACKET_PATH="${DEPTH_PACKET}" \
    "${HEADLESS_BIN}" > "${CAPTURE_LOG}" 2>&1 < /dev/null 9>&- &
  echo "$!" > "${CAPTURE_PID_FILE}"
  nohup nice -n "${NICE_LEVEL}" python3 "${SCRIPT_DIR}/d435_perception_summary.py" \
    --depth-input "${DEPTH_PACKET}" \
    --yolo-input-dir "${STREAM_DIR}" \
    --output "${SUMMARY_PATH}" \
    --depth-output "${DEPTH_COMPAT_PATH}" \
    --yolo-output "${YOLO_COMPAT_PATH}" \
    --capture-owner-pid-file "${CAPTURE_PID_FILE}" \
    --capture-owner-process "${HEADLESS_BIN}" \
    --loop-interval-s 0.1 \
    --max-samples 0 \
    > "${REDUCER_LOG}" 2>&1 < /dev/null 9>&- &
  echo "$!" > "${REDUCER_PID_FILE}"
  sleep "${STARTUP_WAIT_S}"
  if ! health_sidecar; then
    echo "D435 sidecar failed startup" >&2
    stop_sidecar || true
    tail -n 40 "${CAPTURE_LOG}" >&2 || true
    tail -n 40 "${REDUCER_LOG}" >&2 || true
    exit 1
  fi
  print_status
}

case "${1:-status}" in
  build) exec bash "${SCRIPT_DIR}/build_deepyolo_headless.sh" ;;
  start) acquire_operation_lock; start_sidecar; start_supervisor ;;
  stop) acquire_operation_lock; stop_supervisor; stop_sidecar ;;
  restart) acquire_operation_lock; stop_supervisor; stop_sidecar; start_sidecar; start_supervisor ;;
  status) print_status ;;
  health) health_sidecar ;;
  supervisor-start) acquire_operation_lock; start_supervisor ;;
  supervisor-stop) acquire_operation_lock; stop_supervisor ;;
  restart-if-stale)
    health_sidecar >/dev/null 2>&1 && {
      echo "restart=not_needed"
      start_supervisor
      print_status
      exit 0
    }
    acquire_operation_lock
    health_sidecar >/dev/null 2>&1 && {
      echo "restart=not_needed"
      print_status
      exit 0
    }
    stop_sidecar
    start_sidecar
    start_supervisor
    ;;
  *) echo "usage: $0 {build|start|stop|restart|status|health|restart-if-stale|supervisor-start|supervisor-stop}" >&2; exit 2 ;;
esac
