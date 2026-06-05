#!/usr/bin/env bash
set -euo pipefail

# Minimal GO2W SLAM runtime startup used by the edge autonomy entrypoint.
# This script intentionally starts only the LiDAR driver and Unitree SLAM backend.

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
UNITREE_SLAM_DIR="${UNITREE_SLAM_DIR:-/unitree/module/unitree_slam/bin}"
CYCLONEDDS_CONFIG="${CYCLONEDDS_CONFIG:-/unitree/module/unitree_slam/config/cyclonedds.xml}"
SLAM_PARAM_FILE="${SLAM_PARAM_FILE:-/unitree/module/unitree_slam/config/slam_interfaces_server_config/param.yaml}"
LOG_DIR="${GO2W_SLAM_LOG_DIR:-${REPO_ROOT}/artifacts/slam_stack}"
STARTUP_WAIT_S="${GO2W_SLAM_STARTUP_WAIT_S:-8}"
STABILITY_WAIT_S="${GO2W_SLAM_STABILITY_WAIT_S:-4}"

mkdir -p "${LOG_DIR}"

link_exists() {
  [[ -n "${1:-}" ]] && ip link show "$1" >/dev/null 2>&1
}

interface_kind() {
  case "${1:-}" in
    wl*|wlan*)
      echo "wireless"
      ;;
    eth*|en*)
      echo "wired"
      ;;
    *)
      echo "other"
      ;;
  esac
}

is_runtime_interface() {
  case "${1:-}" in
    ""|lo|docker*|veth*|br-*|virbr*|l4tbr*|tailscale*|tun*|tap*)
      return 1
      ;;
    *)
      return 0
      ;;
  esac
}

list_runtime_interfaces() {
  ip -o link show |
    awk -F': ' '{print $2}' |
    sed 's/@.*//' |
    while IFS= read -r candidate; do
      if is_runtime_interface "${candidate}"; then
        echo "${candidate}"
      fi
    done
}

default_route_interface() {
  ip route show default 2>/dev/null |
    awk 'NR == 1 { for (i = 1; i <= NF; i++) if ($i == "dev") { print $(i + 1); exit } }'
}

select_fallback_interface() {
  local configured_interface="$1"
  local configured_kind=""
  local candidate=""
  local route_interface=""

  configured_kind="$(interface_kind "${configured_interface}")"
  if [[ "${configured_kind}" != "other" ]]; then
    while IFS= read -r candidate; do
      if [[ "$(interface_kind "${candidate}")" == "${configured_kind}" ]] && link_exists "${candidate}"; then
        echo "${candidate}"
        return 0
      fi
    done < <(list_runtime_interfaces)
  fi

  route_interface="$(default_route_interface)"
  if is_runtime_interface "${route_interface}" && link_exists "${route_interface}"; then
    echo "${route_interface}"
    return 0
  fi

  while IFS= read -r candidate; do
    if link_exists "${candidate}"; then
      echo "${candidate}"
      return 0
    fi
  done < <(list_runtime_interfaces)
}

write_runtime_cyclonedds_config() {
  local configured_interface="$1"
  local selected_interface="$2"
  local runtime_config="$3"

  sed "s/name=\"${configured_interface}\"/name=\"${selected_interface}\"/" "${CYCLONEDDS_CONFIG}" >"${runtime_config}"
  CYCLONEDDS_CONFIG="${runtime_config}"
}

prepare_cyclonedds_config() {
  local configured_interface=""
  local selected_interface="${GO2W_DDS_INTERFACE:-}"
  local runtime_config="${LOG_DIR}/cyclonedds.runtime.xml"

  [[ -f "${CYCLONEDDS_CONFIG}" ]] || return 0
  configured_interface="$(sed -n 's/.*<NetworkInterface name="\([^"]*\)".*/\1/p' "${CYCLONEDDS_CONFIG}" | head -n 1)"
  if [[ -z "${configured_interface}" ]]; then
    return 0
  fi

  if [[ -n "${selected_interface}" ]]; then
    if ! link_exists "${selected_interface}"; then
      echo "error: requested GO2W_DDS_INTERFACE ${selected_interface} is not available" >&2
      return 1
    fi
    if [[ "${selected_interface}" == "${configured_interface}" ]]; then
      return 0
    fi
    write_runtime_cyclonedds_config "${configured_interface}" "${selected_interface}" "${runtime_config}"
    echo "warning: overriding CycloneDDS interface ${configured_interface} with ${selected_interface}" >&2
    return 0
  fi

  if link_exists "${configured_interface}"; then
    return 0
  fi

  selected_interface="$(select_fallback_interface "${configured_interface}")"
  if [[ -z "${selected_interface}" ]]; then
    echo "error: CycloneDDS interface ${configured_interface} is missing and no fallback interface is available" >&2
    return 1
  fi

  write_runtime_cyclonedds_config "${configured_interface}" "${selected_interface}" "${runtime_config}"
  echo "warning: CycloneDDS interface ${configured_interface} is missing; using runtime copy with ${selected_interface}" >&2
}

print_file_identity() {
  local label="$1"
  local path="$2"

  if [[ ! -e "${path}" ]]; then
    echo "warning: missing ${label}: ${path}" >&2
    return 0
  fi

  echo "${label}: ${path}"
  stat -c "${label}_stat: mtime=%y size=%s owner=%U:%G mode=%a" "${path}" 2>/dev/null || true
  if command -v sha256sum >/dev/null 2>&1; then
    sha256sum "${path}" | awk -v label="${label}" '{print label "_sha256: " $1}'
  fi
}

print_slam_runtime_identity() {
  echo "runtime_identity:"
  print_file_identity "unitree_slam_binary" "${UNITREE_SLAM_DIR}/unitree_slam"
  print_file_identity "xt16_driver_binary" "${UNITREE_SLAM_DIR}/xt16_driver"
  print_file_identity "slam_param_file" "${SLAM_PARAM_FILE}"

  if [[ -f "${SLAM_PARAM_FILE}" ]]; then
    awk '
      /^[^[:space:]].*:/ {
        profile = "";
        key = $1;
        sub(":", "", key);
        if (key == "B2" || key == "B2_W" || key == "Go2" || key == "Go2_W") {
          profile = key;
          print "profile=" profile;
        }
        next;
      }
      profile != "" && /^[[:space:]]+(lidar_type|lidar_ysn|lidar_ip):/ {
        line = $0;
        sub(/^[[:space:]]+/, "", line);
        print "  " line;
      }
    ' "${SLAM_PARAM_FILE}"
  fi
}

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
  local sample=""
  if ! command -v ros2 >/dev/null 2>&1; then
    return 0
  fi
  sample="$(timeout "${timeout_s}" ros2 topic echo "${topic}" --qos-reliability reliable --no-arr 2>/dev/null | sed -n '1{p;q;}')" || true
  if [[ -n "${sample}" ]]; then
    echo "topic ready: ${topic}"
  else
    echo "warning: topic not confirmed yet: ${topic}" >&2
  fi
}

echo "GO2W SLAM stack startup"
prepare_cyclonedds_config
echo "unitree_slam_dir: ${UNITREE_SLAM_DIR}"
echo "cyclonedds_config: ${CYCLONEDDS_CONFIG}"
echo "slam_param_file: ${SLAM_PARAM_FILE}"
echo "log_dir: ${LOG_DIR}"
print_slam_runtime_identity

if [[ "${GO2W_SLAM_CONFIG_ONLY:-0}" == "1" ]]; then
  echo "config-only preflight finished"
  exit 0
fi

ensure_unitree_slam_log_dirs

run_unitree_binary xt16_driver
wait_for_process xt16_driver
sleep "${STABILITY_WAIT_S}"
require_process_alive xt16_driver
fail_on_startup_log_error xt16_driver
check_topic_once /unitree/slam_lidar/points 6

if [[ "${GO2W_START_XT16_GEOMETRY:-0}" == "1" ]]; then
  if ! bash "${SCRIPT_DIR}/go2w_xt16_geometry_sidecar.sh" restart-if-stale; then
    echo "warning: XT16 geometry sidecar is unavailable; SLAM stays online and gateway remains fail-closed" >&2
  fi
fi

run_unitree_binary unitree_slam
wait_for_process unitree_slam
sleep "${STABILITY_WAIT_S}"
require_process_alive unitree_slam
fail_on_startup_log_error unitree_slam
check_topic_once /slam_info 4
require_process_alive unitree_slam
fail_on_startup_log_error unitree_slam

if [[ "${GO2W_START_STEREO_DEPTH:-1}" == "1" ]]; then
  if ! bash "${SCRIPT_DIR}/go2w_stereo_depth_sidecar.sh" restart-if-stale; then
    echo "warning: stereo depth sidecar is unavailable; SLAM stays online but real execution remains blocked" >&2
  fi
fi

echo "startup command finished"
