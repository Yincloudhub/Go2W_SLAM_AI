#!/usr/bin/env bash
set -euo pipefail

# Read-only snapshot for the unified D435 capture owner and summary reducer.

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
SERVICE_DIR="${GO2W_D435_SERVICE_DIR:-${REPO_ROOT}/artifacts/d435_perception_service}"

echo "timestamp=$(date --iso-8601=seconds)"
bash "${SCRIPT_DIR}/go2w_d435_perception_sidecar.sh" status
bash "${SCRIPT_DIR}/go2w_perception_context_sidecar.sh" status
bash "${SCRIPT_DIR}/go2w_perception_context_sidecar.sh" health || true

for pid_file in "${SERVICE_DIR}/capture_owner.pid" "${SERVICE_DIR}/summary_reducer.pid"; do
  if [[ -f "${pid_file}" ]]; then
    pid="$(tr -dc '0-9' < "${pid_file}")"
    if [[ -n "${pid}" ]] && kill -0 "${pid}" 2>/dev/null; then
      ps -p "${pid}" -o pid=,pcpu=,pmem=,etime=,stat=,args=
    fi
  fi
done

if command -v tegrastats >/dev/null 2>&1; then
  timeout 3s tegrastats --interval 1000 | head -n 1 || true
fi
