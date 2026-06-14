#!/usr/bin/env bash
set -euo pipefail

# Keep the external PandarXT-16 clock aligned with the robot before starting
# xt16_driver. This script does not start SLAM, Gateway, or robot motion.

LIDAR_IP="${GO2W_XT16_LIDAR_IP:-192.168.123.20}"
PTP_IFACE="${GO2W_XT16_PTP_IFACE:-eth0}"
SERVICE_DIR="${GO2W_XT16_PTP_SERVICE_DIR:-/tmp/go2w_xt16_ptp}"
PID_FILE="${SERVICE_DIR}/ptp4l.pid"
LOG_FILE="${SERVICE_DIR}/ptp4l.log"
LOCK_TIMEOUT_S="${GO2W_XT16_PTP_LOCK_TIMEOUT_S:-90}"
HEALTH_SAMPLES="${GO2W_XT16_PTP_HEALTH_SAMPLES:-5}"
HEALTH_TIMEOUT_S="${GO2W_XT16_PTP_HEALTH_TIMEOUT_S:-20}"
TX_TIMESTAMP_TIMEOUT_MS="${GO2W_XT16_PTP_TX_TIMESTAMP_TIMEOUT_MS:-1000}"
CONFIG_URL="http://${LIDAR_IP}/pandar.cgi?action=get&object=lidar_config"
PTP_URL="http://${LIDAR_IP}/pandar.cgi?action=set&object=lidar&key=clock_source&value=1"
GPS_URL="http://${LIDAR_IP}/pandar.cgi?action=set&object=lidar&key=clock_source&value=0"

require_root() {
  if [[ "${EUID}" -ne 0 ]]; then
    echo "xt16_ptp=error reason=root_required command='sudo $0 $*'" >&2
    exit 1
  fi
}

read_pid() {
  [[ -f "${PID_FILE}" ]] || return 1
  tr -dc '0-9' < "${PID_FILE}"
}

pid_matches() {
  local pid="$1"
  [[ -n "${pid}" && -d "/proc/${pid}" ]] || return 1
  if [[ -r "/proc/${pid}/cmdline" ]]; then
    tr '\0' ' ' < "/proc/${pid}/cmdline" |
      grep -F -- "ptp4l -i ${PTP_IFACE}" >/dev/null
    return
  fi
  ps -p "${pid}" -o comm= 2>/dev/null | grep -Fx "ptp4l" >/dev/null
}

pid_exists() {
  local pid="$1"
  [[ -n "${pid}" ]] || return 1
  kill -0 "${pid}" 2>/dev/null || [[ -d "/proc/${pid}" ]]
}

is_running() {
  local pid
  pid="$(read_pid 2>/dev/null || true)"
  pid_exists "${pid}" && pid_matches "${pid}"
}

lidar_config() {
  curl --fail --silent --show-error --max-time 3 "${CONFIG_URL}"
}

ptp_healthy() {
  lidar_config | grep -E '"PTPStatus":"(Locked|Tracking)' >/dev/null
}

ptp_stably_healthy() {
  local sample
  for sample in $(seq 1 "${HEALTH_SAMPLES}"); do
    ptp_healthy || return 1
    if [[ "${sample}" -lt "${HEALTH_SAMPLES}" ]]; then
      sleep 1
    fi
  done
}

wait_for_stable_health() {
  local timeout_s="$1"
  local deadline=$((SECONDS + timeout_s))
  while (( SECONDS <= deadline )); do
    if ptp_stably_healthy; then
      return 0
    fi
    sleep 1
  done
  return 1
}

print_status() {
  local pid
  pid="$(read_pid 2>/dev/null || true)"
  if is_running; then
    echo "xt16_ptp=running pid=${pid} interface=${PTP_IFACE}"
  elif [[ -n "${pid}" && -d "/proc/${pid}" ]]; then
    echo "xt16_ptp=running_unverified pid=${pid} interface=${PTP_IFACE} reason=proc_restricted"
  else
    echo "xt16_ptp=stopped interface=${PTP_IFACE}"
  fi
  lidar_config || true
}

stop_ptp() {
  local pid
  pid="$(read_pid 2>/dev/null || true)"
  if [[ -n "${pid}" ]] && kill -0 "${pid}" 2>/dev/null; then
    if ! pid_matches "${pid}"; then
      echo "xt16_ptp=pid_mismatch pid=${pid}; refusing to signal" >&2
      return 1
    fi
    kill "${pid}"
    for _ in $(seq 1 30); do
      kill -0 "${pid}" 2>/dev/null || break
      sleep 0.1
    done
  fi
  rm -f "${PID_FILE}"
  curl --fail --silent --show-error --max-time 3 "${GPS_URL}" >/dev/null || true
  echo "xt16_ptp=stopped clock_source=gps"
}

stop_existing_process() {
  local pid
  pid="$(read_pid 2>/dev/null || true)"
  if [[ -z "${pid}" ]] || ! kill -0 "${pid}" 2>/dev/null; then
    rm -f "${PID_FILE}"
    return 0
  fi
  if ! pid_matches "${pid}"; then
    echo "xt16_ptp=pid_mismatch pid=${pid}; refusing to signal" >&2
    return 1
  fi
  kill "${pid}"
  for _ in $(seq 1 30); do
    kill -0 "${pid}" 2>/dev/null || break
    sleep 0.1
  done
  rm -f "${PID_FILE}"
}

check_ptp() {
  local pid
  pid="$(read_pid 2>/dev/null || true)"
  if ! is_running; then
    echo "xt16_ptp=unhealthy reason=process_not_running pid=${pid:-none}" >&2
    return 1
  fi
  if ! wait_for_stable_health "${HEALTH_TIMEOUT_S}"; then
    echo "xt16_ptp=unhealthy reason=lidar_not_tracking pid=${pid}" >&2
    lidar_config >&2 || true
    return 1
  fi
  echo "xt16_ptp=healthy pid=${pid} samples=${HEALTH_SAMPLES}"
  lidar_config
}

start_ptp() {
  command -v ptp4l >/dev/null || {
    echo "xt16_ptp=error reason=linuxptp_missing install='sudo apt-get install linuxptp'" >&2
    return 1
  }
  command -v curl >/dev/null || {
    echo "xt16_ptp=error reason=curl_missing" >&2
    return 1
  }
  if is_running && ptp_stably_healthy; then
    echo "xt16_ptp=already_healthy"
    check_ptp
    return 0
  fi
  stop_existing_process

  mkdir -p "${SERVICE_DIR}"
  : > "${LOG_FILE}"
  nohup ptp4l -i "${PTP_IFACE}" -S -4 -E -m -q \
    --tx_timestamp_timeout "${TX_TIMESTAMP_TIMEOUT_MS}" >"${LOG_FILE}" 2>&1 &
  echo "$!" > "${PID_FILE}"
  curl --fail --silent --show-error --max-time 3 "${PTP_URL}" >/dev/null

  local elapsed
  for elapsed in $(seq 1 "${LOCK_TIMEOUT_S}"); do
    if ptp_healthy; then
      if ptp_stably_healthy; then
        echo "xt16_ptp=healthy elapsed_s=${elapsed} samples=${HEALTH_SAMPLES}"
        print_status
        return 0
      fi
    fi
    sleep 1
  done

  echo "xt16_ptp=error reason=lock_timeout log=${LOG_FILE}" >&2
  stop_ptp
  return 1
}

case "${1:-status}" in
  start)
    require_root "$@"
    start_ptp
    ;;
  stop)
    require_root "$@"
    stop_ptp
    ;;
  restart)
    require_root "$@"
    stop_ptp
    start_ptp
    ;;
  status)
    print_status
    ;;
  check)
    check_ptp
    ;;
  *)
    echo "usage: $0 {start|stop|restart|status|check}" >&2
    exit 2
    ;;
esac
