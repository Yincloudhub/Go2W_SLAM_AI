#!/usr/bin/env bash
set -euo pipefail

# Minimal GO2W SLAM runtime startup used by the edge autonomy entrypoint.
# This script intentionally starts only the LiDAR driver and Unitree SLAM backend.

UNITREE_SLAM_DIR="${UNITREE_SLAM_DIR:-/unitree/module/unitree_slam/bin}"
CYCLONEDDS_CONFIG="${CYCLONEDDS_CONFIG:-/unitree/module/unitree_slam/config/cyclonedds.xml}"
LOG_DIR="${GO2W_SLAM_LOG_DIR:-${HOME}/go2w_slam_agent/artifacts/slam_stack}"
STARTUP_WAIT_S="${GO2W_SLAM_STARTUP_WAIT_S:-8}"
STABILITY_WAIT_S="${GO2W_SLAM_STABILITY_WAIT_S:-4}"

mkdir -p "${LOG_DIR}"

ensure_unitree_slam_log_dirs() {
  local logs_dir="${UNITREE_SLAM_DIR}/logs"
  local server_dir="${logs_dir}/slam_server"
  local driver_dir="${logs_dir}/slam_driver"

  if [[ -d "${server_dir}" && -w "${server_dir}" ]]; then
    return 0
  fi

  if mkdir -p "${server_dir}" "${driver_dir}" >/dev/null 2>&1; then
    return 0
  fi

  if command -v sudo >/dev/null 2>&1 && sudo -n true >/dev/null 2>&1; then
    sudo mkdir -p "${server_dir}" "${driver_dir}"
    sudo chown -R "$(id -un):$(id -gn)" "${logs_dir}"
    return 0
  fi

  echo "warning: ${server_dir} is not writable; unitree_slam may exit during log init." >&2
  echo "fix with: sudo mkdir -p ${server_dir} ${driver_dir} && sudo chown -R $(id -un):$(id -gn) ${logs_dir}" >&2
  return 0
}

run_unitree_binary() {
  local name="$1"
  local binary="${UNITREE_SLAM_DIR}/${name}"
  local log_file="${LOG_DIR}/${name}.log"

  if [[ ! -x "${binary}" ]]; then
    echo "missing executable: ${binary}" >&2
    return 1
  fi

  if pidof "${name}" >/dev/null 2>&1; then
    echo "${name} already running"
    return 0
  fi

  echo "starting ${name}, log: ${log_file}"
  (
    cd "${UNITREE_SLAM_DIR}"
    nohup env -i \
      HOME="${HOME}" \
      USER="${USER:-unitree}" \
      LOGNAME="${LOGNAME:-unitree}" \
      PATH=/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin \
      LD_LIBRARY_PATH=/usr/local/lib \
      CYCLONEDDS_URI="file://${CYCLONEDDS_CONFIG}" \
      "./${name}" >"${log_file}" 2>&1 &
    echo $! >"${LOG_DIR}/${name}.pid"
  )
}

wait_for_process() {
  local name="$1"
  local deadline=$((SECONDS + STARTUP_WAIT_S))
  while (( SECONDS < deadline )); do
    if pidof "${name}" >/dev/null 2>&1; then
      echo "${name} is running"
      return 0
    fi
    sleep 1
  done
  echo "warning: ${name} did not appear within ${STARTUP_WAIT_S}s" >&2
  return 1
}

require_process_alive() {
  local name="$1"
  local log_file="${LOG_DIR}/${name}.log"
  if pidof "${name}" >/dev/null 2>&1; then
    return 0
  fi
  echo "error: ${name} is not running after startup checks" >&2
  if [[ -f "${log_file}" ]]; then
    echo "last ${name} log lines:" >&2
    tail -n 80 "${log_file}" >&2 || true
  fi
  return 1
}

fail_on_startup_log_error() {
  local name="$1"
  local log_file="${LOG_DIR}/${name}.log"
  [[ -f "${log_file}" ]] || return 0
  if grep -Eiq 'lidar ysn check failed|Permission denied|No such file or directory|segmentation fault|core dumped' "${log_file}"; then
    echo "error: ${name} startup log contains a fatal startup error" >&2
    tail -n 80 "${log_file}" >&2 || true
    return 1
  fi
}

check_topic_once() {
  local topic="$1"
  local timeout_s="$2"
  if ! command -v ros2 >/dev/null 2>&1; then
    return 0
  fi
  if timeout "${timeout_s}" ros2 topic echo "${topic}" --qos-reliability reliable --no-arr >/dev/null 2>&1; then
    echo "topic ready: ${topic}"
  else
    echo "warning: topic not confirmed yet: ${topic}" >&2
  fi
}

echo "GO2W SLAM stack startup"
echo "unitree_slam_dir: ${UNITREE_SLAM_DIR}"
echo "cyclonedds_config: ${CYCLONEDDS_CONFIG}"
echo "log_dir: ${LOG_DIR}"

ensure_unitree_slam_log_dirs

run_unitree_binary xt16_driver
wait_for_process xt16_driver
sleep "${STABILITY_WAIT_S}"
require_process_alive xt16_driver
fail_on_startup_log_error xt16_driver
check_topic_once /unitree/slam_lidar/points 6

run_unitree_binary unitree_slam
wait_for_process unitree_slam
sleep "${STABILITY_WAIT_S}"
require_process_alive unitree_slam
fail_on_startup_log_error unitree_slam
check_topic_once /slam_info 4
require_process_alive unitree_slam
fail_on_startup_log_error unitree_slam

echo "startup command finished"
