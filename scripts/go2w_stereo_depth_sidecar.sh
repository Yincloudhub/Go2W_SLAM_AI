#!/usr/bin/env bash
set -euo pipefail

# Compatibility entrypoint. Depth is produced by the unified D435 capture owner.
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
exec bash "${SCRIPT_DIR}/go2w_d435_perception_sidecar.sh" "${1:-status}"
