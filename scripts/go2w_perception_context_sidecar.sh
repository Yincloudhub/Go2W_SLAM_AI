#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
SERVICE_DIR="${GO2W_PERCEPTION_CONTEXT_SERVICE_DIR:-${REPO_ROOT}/artifacts/perception_context_service}"
OUTPUT_PATH="${GO2W_PERCEPTION_CONTEXT_PATH:-${REPO_ROOT}/artifacts/perception_context_v1.json}"
PID_FILE="${SERVICE_DIR}/producer.pid"
LOG_FILE="${SERVICE_DIR}/producer.log"
LOCK_FILE="${SERVICE_DIR}/manager.lock"
PYTHON_BIN="${GO2W_PYTHON:-python3}"
INTERVAL_MS="${GO2W_PERCEPTION_CONTEXT_INTERVAL_MS:-250}"

mkdir -p "${SERVICE_DIR}"

read_pid() {
  [[ -f "${PID_FILE}" ]] || return 1
  tr -dc '0-9' < "${PID_FILE}"
}

is_running() {
  local pid
  pid="$(read_pid 2>/dev/null || true)"
  [[ -n "${pid}" && -r "/proc/${pid}/cmdline" ]] || return 1
  kill -0 "${pid}" 2>/dev/null || return 1
  tr '\0' ' ' < "/proc/${pid}/cmdline" | grep -F -- "perception_context_service.py" >/dev/null
}

health() {
  "${PYTHON_BIN}" "${SCRIPT_DIR}/perception_context_service.py" \
    --repo-root "${REPO_ROOT}" \
    --output "${OUTPUT_PATH}" \
    --check
}

start_service() {
  if is_running; then
    echo "perception_context=already_running pid=$(read_pid)"
    health
    return 0
  fi
  rm -f "${PID_FILE}"
  nohup "${PYTHON_BIN}" "${SCRIPT_DIR}/perception_context_service.py" \
    --repo-root "${REPO_ROOT}" \
    --output "${OUTPUT_PATH}" \
    --interval-ms "${INTERVAL_MS}" \
    >>"${LOG_FILE}" 2>&1 &
  local pid=$!
  echo "${pid}" > "${PID_FILE}"
  for _ in $(seq 1 30); do
    if is_running && health >/dev/null 2>&1; then
      echo "perception_context=started pid=${pid}"
      health
      return 0
    fi
    sleep 0.1
  done
  echo "perception_context=start_failed pid=${pid}" >&2
  stop_service || true
  return 1
}

stop_service() {
  local pid
  pid="$(read_pid 2>/dev/null || true)"
  if [[ -n "${pid}" ]] && is_running; then
    kill "${pid}" 2>/dev/null || true
    for _ in $(seq 1 30); do
      kill -0 "${pid}" 2>/dev/null || break
      sleep 0.1
    done
    kill -9 "${pid}" 2>/dev/null || true
  fi
  rm -f "${PID_FILE}"
  echo "perception_context=stopped"
}

status_service() {
  if is_running; then
    echo "perception_context=running pid=$(read_pid)"
  else
    echo "perception_context=stopped"
  fi
  echo "output_path=${OUTPUT_PATH}"
}

with_lock() {
  exec 9>"${LOCK_FILE}"
  flock -x 9
  "$@"
}

case "${1:-status}" in
  start)
    with_lock start_service
    ;;
  stop)
    with_lock stop_service
    ;;
  restart)
    with_lock stop_service
    with_lock start_service
    ;;
  restart-if-stale)
    if is_running && health >/dev/null 2>&1; then
      status_service
    else
      with_lock stop_service
      with_lock start_service
    fi
    ;;
  status)
    status_service
    ;;
  health)
    status_service
    health
    ;;
  *)
    echo "usage: $0 {start|stop|restart|restart-if-stale|status|health}" >&2
    exit 2
    ;;
esac
