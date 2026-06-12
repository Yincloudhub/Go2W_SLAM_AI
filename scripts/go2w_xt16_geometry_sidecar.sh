#!/usr/bin/env bash
set -euo pipefail

# Lightweight XT16 PointCloud2 ROI summary producer used by the gateway safety
# path. It emits compact JSON only; raw point clouds stay inside the robot.

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
SERVICE_DIR="${GO2W_XT16_GEOMETRY_SERVICE_DIR:-${REPO_ROOT}/artifacts/xt16_geometry_service}"
SUMMARY_PATH="${GO2W_LIDAR_GEOMETRY_SUMMARY_PATH:-${REPO_ROOT}/artifacts/lidar_geometry_summary.json}"
PID_FILE="${SERVICE_DIR}/producer.pid"
LOG_FILE="${SERVICE_DIR}/producer.log"
PYTHON_BIN="${GO2W_PYTHON:-python3}"
NICE_LEVEL="${GO2W_XT16_GEOMETRY_NICE_LEVEL:-5}"
TOPIC="${GO2W_XT16_POINTCLOUD_TOPIC:-/unitree/slam_lidar/points}"
RATE_LIMIT_HZ="${GO2W_XT16_GEOMETRY_RATE_LIMIT_HZ:-5}"
SAFETY_STALE_MS="${GO2W_LIDAR_GEOMETRY_STALE_MS:-1000}"
CALIBRATION_REQUESTED="${GO2W_XT16_GEOMETRY_CALIBRATED:-0}"
CALIBRATION_RECORD="${GO2W_XT16_CALIBRATION_RECORD:-${REPO_ROOT}/configs/perception/xt16_geometry_calibration.json}"
export GO2W_XT16_GEOMETRY_RANGE_M="${GO2W_XT16_GEOMETRY_RANGE_M:-6.0}"
export GO2W_XT16_GEOMETRY_PERCENTILE="${GO2W_XT16_GEOMETRY_PERCENTILE:-10.0}"
export GO2W_XT16_GEOMETRY_MIN_POINTS_PER_ROI="${GO2W_XT16_GEOMETRY_MIN_POINTS_PER_ROI:-8}"
export GO2W_XT16_GEOMETRY_FRONT_HALF_WIDTH_M="${GO2W_XT16_GEOMETRY_FRONT_HALF_WIDTH_M:-0.45}"
export GO2W_XT16_GEOMETRY_SIDE_FORWARD_M="${GO2W_XT16_GEOMETRY_SIDE_FORWARD_M:-0.75}"
export GO2W_XT16_GEOMETRY_REAR_HALF_WIDTH_M="${GO2W_XT16_GEOMETRY_REAR_HALF_WIDTH_M:-0.45}"
export GO2W_XT16_GEOMETRY_FOOTPRINT_FRONT_M="${GO2W_XT16_GEOMETRY_FOOTPRINT_FRONT_M:-0.25}"
export GO2W_XT16_GEOMETRY_FOOTPRINT_REAR_M="${GO2W_XT16_GEOMETRY_FOOTPRINT_REAR_M:-0.50}"
export GO2W_XT16_GEOMETRY_FOOTPRINT_HALF_WIDTH_M="${GO2W_XT16_GEOMETRY_FOOTPRINT_HALF_WIDTH_M:-0.30}"
export GO2W_XT16_GEOMETRY_MIN_Z_M="${GO2W_XT16_GEOMETRY_MIN_Z_M:--0.25}"
export GO2W_XT16_GEOMETRY_BODY_MIN_Z_M="${GO2W_XT16_GEOMETRY_BODY_MIN_Z_M:--0.10}"
export GO2W_XT16_GEOMETRY_MAX_Z_M="${GO2W_XT16_GEOMETRY_MAX_Z_M:-1.20}"
export GO2W_XT16_GEOMETRY_CLUSTER_GAP_M="${GO2W_XT16_GEOMETRY_CLUSTER_GAP_M:-0.15}"
export GO2W_XT16_GEOMETRY_SUPPORT_BIN_M="${GO2W_XT16_GEOMETRY_SUPPORT_BIN_M:-0.05}"
export GO2W_XT16_GEOMETRY_MIN_SPATIAL_BINS="${GO2W_XT16_GEOMETRY_MIN_SPATIAL_BINS:-2}"
export GO2W_XT16_GEOMETRY_PENDING_MIN_POINTS="${GO2W_XT16_GEOMETRY_PENDING_MIN_POINTS:-3}"
export GO2W_XT16_GEOMETRY_MIN_CLOUD_POINTS="${GO2W_XT16_GEOMETRY_MIN_CLOUD_POINTS:-1000}"
export GO2W_XT16_GEOMETRY_NO_RETURN_CONFIDENCE="${GO2W_XT16_GEOMETRY_NO_RETURN_CONFIDENCE:-0.5}"
export GO2W_XT16_GEOMETRY_FORWARD_AXIS="${GO2W_XT16_GEOMETRY_FORWARD_AXIS:-y}"
export GO2W_XT16_GEOMETRY_LATERAL_AXIS="${GO2W_XT16_GEOMETRY_LATERAL_AXIS:-x}"
export GO2W_XT16_GEOMETRY_VERTICAL_AXIS="${GO2W_XT16_GEOMETRY_VERTICAL_AXIS:-z}"
export GO2W_XT16_GEOMETRY_FORWARD_SIGN="${GO2W_XT16_GEOMETRY_FORWARD_SIGN:--1.0}"
export GO2W_XT16_GEOMETRY_LATERAL_SIGN="${GO2W_XT16_GEOMETRY_LATERAL_SIGN:-1.0}"
export GO2W_XT16_GEOMETRY_VERTICAL_SIGN="${GO2W_XT16_GEOMETRY_VERTICAL_SIGN:-1.0}"

mkdir -p "${SERVICE_DIR}"

read_pid() {
  [[ -f "${PID_FILE}" ]] || return 1
  tr -dc '0-9' < "${PID_FILE}"
}

pid_matches() {
  local pid="$1"
  [[ -n "${pid}" && -r "/proc/${pid}/cmdline" ]] || return 1
  tr '\0' ' ' < "/proc/${pid}/cmdline" | grep -F -- "xt16_lidar_geometry_summary.py" >/dev/null
}

is_running() {
  local pid
  pid="$(read_pid 2>/dev/null || true)"
  [[ -n "${pid}" ]] && kill -0 "${pid}" 2>/dev/null && pid_matches "${pid}"
}

summary_health() {
  SUMMARY_PATH="${SUMMARY_PATH}" SAFETY_STALE_MS="${SAFETY_STALE_MS}" "${PYTHON_BIN}" - <<'PY'
import json
import math
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
receipt_age_ms = max(0, int(time.time() * 1000) - timestamp_ms) if timestamp_ms else -1
sensor_latency = data.get("latency_ms")
sensor_latency_ms = (
    max(0.0, float(sensor_latency))
    if isinstance(sensor_latency, (int, float))
    and not isinstance(sensor_latency, bool)
    and math.isfinite(float(sensor_latency))
    else 0.0
)
age_ms = (
    int(math.ceil(receipt_age_ms + sensor_latency_ms))
    if receipt_age_ms >= 0
    else -1
)
roi = data.get("roi_confidence") if isinstance(data.get("roi_confidence"), dict) else {}
low_hazards = data.get("low_hazard_directions") if isinstance(data.get("low_hazard_directions"), list) else []
fresh = timestamp_ms > 0 and age_ms <= max_age_ms and not bool(data.get("stale", False))
print(
    "summary_health="
    + ("ok" if fresh else "stale")
    + f" age_ms={age_ms} stale_ms={max_age_ms}"
    + f" receipt_age_ms={receipt_age_ms} sensor_latency_ms={sensor_latency_ms:.1f}"
    + f" stale_reasons={data.get('stale_reasons')}"
    + f" front_m={data.get('front_clearance_m')} left_m={data.get('left_clearance_m')} right_m={data.get('right_clearance_m')}"
    + f" low_hazards={low_hazards}"
    + f" roi_front={roi.get('front')} roi_left={roi.get('left')} roi_right={roi.get('right')}"
)
raise SystemExit(0 if fresh else 1)
PY
}

print_status() {
  local pid
  pid="$(read_pid 2>/dev/null || true)"
  if is_running; then
    echo "xt16_geometry=running pid=${pid}"
  else
    echo "xt16_geometry=stopped"
  fi
  echo "summary_path=${SUMMARY_PATH}"
  echo "topic=${TOPIC}"
  echo "calibration_requested=${CALIBRATION_REQUESTED}"
  echo "calibration_record=${CALIBRATION_RECORD}"
  summary_health || true
}

stop_sidecar() {
  local pid
  pid="$(read_pid 2>/dev/null || true)"
  if [[ -z "${pid}" ]]; then
    echo "xt16_geometry=stopped"
    return 0
  fi
  if ! kill -0 "${pid}" 2>/dev/null; then
    rm -f "${PID_FILE}"
    echo "xt16_geometry=stopped stale_pid=${pid}"
    return 0
  fi
  if ! pid_matches "${pid}"; then
    echo "xt16_geometry=pid_mismatch pid=${pid}; refusing to signal" >&2
    return 1
  fi
  kill "${pid}"
  for _ in $(seq 1 30); do
    if ! kill -0 "${pid}" 2>/dev/null; then
      rm -f "${PID_FILE}"
      echo "xt16_geometry=stopped pid=${pid}"
      return 0
    fi
    sleep 0.1
  done
  echo "xt16_geometry=still_running pid=${pid}; inspect manually" >&2
  return 1
}

start_sidecar() {
  if is_running; then
    echo "xt16_geometry=already_running"
    print_status
    return 0
  fi
  : > "${LOG_FILE}"
  local calibrated_args=()
  if [[ "${CALIBRATION_REQUESTED}" == "1" || "${CALIBRATION_REQUESTED}" == "true" || "${CALIBRATION_REQUESTED}" == "yes" ]]; then
    if [[ -n "${GO2W_XT16_GEOMETRY_EXTRA_ARGS:-}" ]]; then
      echo "xt16_geometry=calibration_rejected reason=extra_args_not_allowed" >&2
      return 1
    fi
    local calibration_id
    if ! calibration_id="$("${PYTHON_BIN}" "${REPO_ROOT}/scripts/xt16_calibration_guard.py" --record "${CALIBRATION_RECORD}")"; then
      echo "xt16_geometry=calibration_rejected reason=${calibration_id}" >&2
      return 1
    fi
    calibrated_args=(--calibrated --calibration-id "${calibration_id}")
  fi
  # shellcheck disable=SC2086
  nohup nice -n "${NICE_LEVEL}" "${PYTHON_BIN}" "${REPO_ROOT}/scripts/xt16_lidar_geometry_summary.py" \
    --topic "${TOPIC}" \
    --output "${SUMMARY_PATH}" \
    --rate-limit-hz "${RATE_LIMIT_HZ}" \
    --range-m "${GO2W_XT16_GEOMETRY_RANGE_M}" \
    --percentile "${GO2W_XT16_GEOMETRY_PERCENTILE}" \
    --min-points-per-roi "${GO2W_XT16_GEOMETRY_MIN_POINTS_PER_ROI}" \
    --front-half-width-m "${GO2W_XT16_GEOMETRY_FRONT_HALF_WIDTH_M}" \
    --side-forward-m "${GO2W_XT16_GEOMETRY_SIDE_FORWARD_M}" \
    --rear-half-width-m "${GO2W_XT16_GEOMETRY_REAR_HALF_WIDTH_M}" \
    --footprint-front-m "${GO2W_XT16_GEOMETRY_FOOTPRINT_FRONT_M}" \
    --footprint-rear-m "${GO2W_XT16_GEOMETRY_FOOTPRINT_REAR_M}" \
    --footprint-half-width-m "${GO2W_XT16_GEOMETRY_FOOTPRINT_HALF_WIDTH_M}" \
    --min-z-m "${GO2W_XT16_GEOMETRY_MIN_Z_M}" \
    --body-min-z-m "${GO2W_XT16_GEOMETRY_BODY_MIN_Z_M}" \
    --max-z-m "${GO2W_XT16_GEOMETRY_MAX_Z_M}" \
    --clearance-cluster-gap-m "${GO2W_XT16_GEOMETRY_CLUSTER_GAP_M}" \
    --support-bin-m "${GO2W_XT16_GEOMETRY_SUPPORT_BIN_M}" \
    --min-spatial-bins "${GO2W_XT16_GEOMETRY_MIN_SPATIAL_BINS}" \
    --pending-min-points "${GO2W_XT16_GEOMETRY_PENDING_MIN_POINTS}" \
    --min-cloud-points-for-no-return "${GO2W_XT16_GEOMETRY_MIN_CLOUD_POINTS}" \
    --no-return-confidence "${GO2W_XT16_GEOMETRY_NO_RETURN_CONFIDENCE}" \
    --forward-axis "${GO2W_XT16_GEOMETRY_FORWARD_AXIS}" \
    --lateral-axis "${GO2W_XT16_GEOMETRY_LATERAL_AXIS}" \
    --vertical-axis "${GO2W_XT16_GEOMETRY_VERTICAL_AXIS}" \
    --forward-sign "${GO2W_XT16_GEOMETRY_FORWARD_SIGN}" \
    --lateral-sign "${GO2W_XT16_GEOMETRY_LATERAL_SIGN}" \
    --vertical-sign "${GO2W_XT16_GEOMETRY_VERTICAL_SIGN}" \
    "${calibrated_args[@]}" \
    ${GO2W_XT16_GEOMETRY_EXTRA_ARGS:-} \
    > "${LOG_FILE}" 2>&1 < /dev/null &
  echo "$!" > "${PID_FILE}"
  sleep 2
  if ! is_running; then
    echo "xt16_geometry=start_failed" >&2
    tail -n 80 "${LOG_FILE}" >&2 || true
    stop_sidecar || true
    return 1
  fi
  if [[ "${#calibrated_args[@]}" -gt 0 ]] && ! summary_health; then
    echo "xt16_geometry=start_failed_unhealthy" >&2
    tail -n 80 "${LOG_FILE}" >&2 || true
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
