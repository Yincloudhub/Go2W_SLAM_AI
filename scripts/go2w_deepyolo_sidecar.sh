#!/usr/bin/env bash
set -euo pipefail

# Compatibility entrypoint. The unified manager is the only D435 owner.
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
export GO2W_D435_PROFILE="${GO2W_D435_PROFILE:-${GO2W_DEEPYOLO_PROFILE:-resident}}"
exec bash "${SCRIPT_DIR}/go2w_d435_perception_sidecar.sh" "${1:-status}"
