#!/usr/bin/env bash
set -euo pipefail

if [[ $# -ne 3 ]]; then
    echo "Usage: run_mainline.sh <raw_bin_path> <vfit_mat_path> <jpg_output_dir>" >&2
    exit 1
fi

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
RELEASE_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
PROJECT_DIR="$(cd "${RELEASE_DIR}/../.." && pwd)"
BUILD_DIR="${PROJECT_DIR}/build"
CONFIG_PATH="${RELEASE_DIR}/config/fft_imaging_orin.cfg"

RAW_PATH="$1"
VFIT_PATH="$2"
OUTPUT_DIR="$3"

"${BUILD_DIR}/fft_imaging_cli" \
    --config "${CONFIG_PATH}" \
    "${RAW_PATH}" \
    "${VFIT_PATH}" \
    "${OUTPUT_DIR}"
