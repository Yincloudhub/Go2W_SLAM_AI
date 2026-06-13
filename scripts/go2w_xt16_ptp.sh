#!/usr/bin/env bash
set -euo pipefail

# Keep the external PandarXT-16 clock aligned with the robot before starting
# xt16_driver. This script does not start SLAM, Gateway, or robot motion.

LIDAR_IP="${GO2W_XT16_LIDAR_IP:-192.168.123.20}"
PTP_IFACE="${GO2W_XT16_PTP_IFACE:-eth0}"
SERVICE_DIR="${GO2W_XT16_PTP_SERVICE_DIR:-/tmp/go2w_xt16_ptp}"
PID_FILE="${SERVICE_DIR}/ptp4l.pid"
LOG_FILE="${SERVICE_DIR}/ptp4l.log"
LOCK_TIMEOUT_S="${GO2W_XT16_PTP_LOCK_TIMEOUT_S:-45}"
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
  [[ -n "${pid}" && -r "/proc/${pid}/cmdline" ]] || return 1
  tr '\0' ' ' < "/proc/${pid}/cmdline" |
    grep -F -- "ptp4l -i ${PTP_IFACE}" >/dev/null
}

is_running() {
  local pid
  pid="$(read_pid 2>/dev/null || true)"
  [[ -n "${pid}" ]] && kill -0 "${pid}" 2>/dev/null && pid_matches "${pid}"
}

lidar_config() {
  curl --fail --silent --show-error --max-time 3 "${CONFIG_URL}"
}

ptp_locked() {
  lidar_config | grep -F '"PTPStatus":"Locked' >/dev/null
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

start_ptp() {
  command -v ptp4l >/dev/null || {
    echo "xt16_ptp=error reason=linuxptp_missing install='sudo apt-get install linuxptp'" >&2
    return 1
  }
  command -v curl >/dev/null || {
    echo "xt16_ptp=error reason=curl_missing" >&2
    return 1
  }
  if is_running && ptp_locked; then
    echo "xt16_ptp=already_locked"
    print_status
    return 0
  fi

  mkdir -p "${SERVICE_DIR}"
  : > "${LOG_FILE}"
  nohup ptp4l -i "${PTP_IFACE}" -S -4 -E -m -q >"${LOG_FILE}" 2>&1 &
  echo "$!" > "${PID_FILE}"
  curl --fail --silent --show-error --max-time 3 "${PTP_URL}" >/dev/null

  local elapsed
  for elapsed in $(seq 1 "${LOCK_TIMEOUT_S}"); do
    if ptp_locked; then
      echo "xt16_ptp=locked elapsed_s=${elapsed}"
      print_status
      return 0
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
  *)
    echo "usage: $0 {start|stop|restart|status}" >&2
    exit 2
    ;;
esac
