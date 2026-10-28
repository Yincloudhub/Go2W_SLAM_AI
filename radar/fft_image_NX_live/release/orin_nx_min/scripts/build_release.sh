#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
RELEASE_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
PROJECT_DIR="$(cd "${RELEASE_DIR}/../.." && pwd)"
BUILD_DIR="${PROJECT_DIR}/build"

cmake -S "${PROJECT_DIR}" -B "${BUILD_DIR}" -DCMAKE_BUILD_TYPE=Release
cmake --build "${BUILD_DIR}" -j"$(nproc)"

echo "Built: ${BUILD_DIR}/fft_imaging_cli"
