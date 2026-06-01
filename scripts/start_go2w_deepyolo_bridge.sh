#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"

INPUT_DIR="${GO2W_DEEPYOLO_INPUT_DIR:-/home/unitree/librealsense/examples/DeepYolo_test/output}"
OUTPUT="${GO2W_DEEPYOLO_SUMMARY_PATH:-${REPO_ROOT}/artifacts/vision_semantic_summary.json}"
LOOP_INTERVAL="${GO2W_DEEPYOLO_LOOP_INTERVAL_S:-0.5}"
MAX_SAMPLES="${GO2W_DEEPYOLO_MAX_SAMPLES:-0}"
STALE_MS="${GO2W_DEEPYOLO_STALE_MS:-3000}"
SOURCE_STALE_MS="${GO2W_DEEPYOLO_SOURCE_STALE_MS:-30000}"
CLOCK_SKEW_LIMIT_MS="${GO2W_DEEPYOLO_CLOCK_SKEW_LIMIT_MS:-5000}"
MAX_OBJECTS="${GO2W_DEEPYOLO_MAX_OBJECTS:-8}"
PRINT_ARGS=()
if [[ "${GO2W_DEEPYOLO_PRINT:-0}" == "1" ]]; then
  PRINT_ARGS+=(--print)
fi

cd "${REPO_ROOT}"
exec python3 scripts/deepyolo_semantic_bridge.py \
  --input-dir "${INPUT_DIR}" \
  --output "${OUTPUT}" \
  --loop-interval-s "${LOOP_INTERVAL}" \
  --max-samples "${MAX_SAMPLES}" \
  --stale-ms "${STALE_MS}" \
  --source-stale-ms "${SOURCE_STALE_MS}" \
  --clock-skew-limit-ms "${CLOCK_SKEW_LIMIT_MS}" \
  --max-objects "${MAX_OBJECTS}" \
  "${PRINT_ARGS[@]}"
