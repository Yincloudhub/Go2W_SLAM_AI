#!/usr/bin/env bash
set -euo pipefail

# Short, non-motion entry point for supervised GO2W acceptance.
# This wrapper can relocate SLAM coordinates, but it never executes navigation.

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PYTHON_BIN="${GO2W_PYTHON:-python3}"
ACCEPTANCE="${SCRIPT_DIR}/go2w_supervised_acceptance.py"

usage() {
  cat <<'EOF'
usage:
  bash scripts/go2w_accept.sh status
  bash scripts/go2w_accept.sh relocate [ANCHOR] confirm
  bash scripts/go2w_accept.sh verify [ANCHOR]
  bash scripts/go2w_accept.sh check TARGET [ANCHOR]

status    Read-only runtime and safety state.
relocate  Requires the literal third argument "confirm"; does not move chassis.
verify    Read-only repeated localization verification.
check     Read-only navigation preflight; prints a command but never runs it.
EOF
}

action="${1:-status}"
case "${action}" in
  status)
    exec "${PYTHON_BIN}" "${ACCEPTANCE}" --stage status
    ;;
  relocate)
    anchor="${2:-mapping_origin}"
    confirmation="${3:-}"
    if [[ "${confirmation}" != "confirm" ]]; then
      echo "relocate requires: relocate [ANCHOR] confirm" >&2
      usage >&2
      exit 2
    fi
    exec "${PYTHON_BIN}" "${ACCEPTANCE}" \
      --stage relocate \
      --anchor "${anchor}" \
      --confirm-relocation "${anchor}"
    ;;
  verify)
    anchor="${2:-mapping_origin}"
    exec "${PYTHON_BIN}" "${ACCEPTANCE}" \
      --stage verify-localization \
      --anchor "${anchor}"
    ;;
  check)
    target="${2:-}"
    anchor="${3:-mapping_origin}"
    if [[ -z "${target}" ]]; then
      echo "check requires TARGET" >&2
      usage >&2
      exit 2
    fi
    exec "${PYTHON_BIN}" "${ACCEPTANCE}" \
      --stage prepare-navigation \
      --target "${target}" \
      --anchor "${anchor}"
    ;;
  -h|--help|help)
    usage
    ;;
  *)
    echo "unknown action: ${action}" >&2
    usage >&2
    exit 2
    ;;
esac
