#!/usr/bin/env bash
set -euo pipefail

# Short, non-motion entry point for supervised GO2W acceptance.
# This wrapper can relocate SLAM coordinates, but it never executes navigation.

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PYTHON_BIN="${GO2W_PYTHON:-python3}"
ACCEPTANCE="${SCRIPT_DIR}/go2w_supervised_acceptance.py"
CAPTURE="${SCRIPT_DIR}/capture_go2w_field_acceptance.py"
XT16_CAPTURE="${SCRIPT_DIR}/capture_xt16_calibration_scene.py"

usage() {
  cat <<'EOF'
usage:
  bash scripts/go2w_accept.sh status
  bash scripts/go2w_accept.sh relocate ANCHOR confirm
  bash scripts/go2w_accept.sh verify ANCHOR
  bash scripts/go2w_accept.sh check TARGET
  bash scripts/go2w_accept.sh snapshot TEST_ID [ANCHOR]
  bash scripts/go2w_accept.sh xt16-baseline OPERATOR FRONT_M LEFT_M RIGHT_M REAR_M
  bash scripts/go2w_accept.sh xt16-scene SCENE OPERATOR MEASURED_M

status    Read-only runtime and safety state.
relocate  Requires the literal third argument "confirm"; does not move chassis.
verify    Read-only repeated localization verification.
check     Read-only navigation preflight; prints a command but never runs it.
snapshot  Read-only evidence capture; optional ANCHOR adds consecutive localization samples.
xt16-baseline
          Capture 25 stationary XT16 samples with four measured clearances.
xt16-scene
          Capture 25 stationary XT16 samples for front/left/right/rear.
          For baseline, call the Python tool directly with all four measurements.
EOF
}

action="${1:-status}"
case "${action}" in
  status)
    exec "${PYTHON_BIN}" "${ACCEPTANCE}" --stage status
    ;;
  relocate)
    anchor="${2:-}"
    confirmation="${3:-}"
    if [[ -z "${anchor}" || "${confirmation}" != "confirm" ]]; then
      echo "relocate requires: relocate ANCHOR confirm" >&2
      usage >&2
      exit 2
    fi
    exec "${PYTHON_BIN}" "${ACCEPTANCE}" \
      --stage relocate \
      --anchor "${anchor}" \
      --confirm-relocation "${anchor}"
    ;;
  verify)
    anchor="${2:-}"
    if [[ -z "${anchor}" ]]; then
      echo "verify requires: verify ANCHOR" >&2
      usage >&2
      exit 2
    fi
    exec "${PYTHON_BIN}" "${ACCEPTANCE}" \
      --stage verify-localization \
      --anchor "${anchor}"
    ;;
  check)
    target="${2:-}"
    if [[ -z "${target}" ]]; then
      echo "check requires TARGET" >&2
      usage >&2
      exit 2
    fi
    exec "${PYTHON_BIN}" "${ACCEPTANCE}" \
      --stage prepare-navigation \
      --target "${target}"
    ;;
  snapshot)
    test_id="${2:-}"
    anchor="${3:-}"
    if [[ -z "${test_id}" ]]; then
      echo "snapshot requires TEST_ID and accepts an optional ANCHOR" >&2
      usage >&2
      exit 2
    fi
    args=("${CAPTURE}" --test-id "${test_id}")
    if [[ -n "${anchor}" ]]; then
      args+=(--verify-anchor "${anchor}")
    fi
    exec "${PYTHON_BIN}" "${args[@]}"
    ;;
  xt16-scene)
    scene="${2:-}"
    operator="${3:-}"
    measured="${4:-}"
    case "${scene}" in
      front|left|right|rear) ;;
      *)
        echo "xt16-scene requires front, left, right, or rear" >&2
        usage >&2
        exit 2
        ;;
    esac
    if [[ -z "${operator}" || -z "${measured}" ]]; then
      echo "xt16-scene requires: xt16-scene SCENE OPERATOR MEASURED_M" >&2
      usage >&2
      exit 2
    fi
    exec "${PYTHON_BIN}" "${XT16_CAPTURE}" \
      --scene "${scene}" \
      --operator "${operator}" \
      "--measured-${scene}-m" "${measured}"
    ;;
  xt16-baseline)
    operator="${2:-}"
    front="${3:-}"
    left="${4:-}"
    right="${5:-}"
    rear="${6:-}"
    if [[ -z "${operator}" || -z "${front}" || -z "${left}" || -z "${right}" || -z "${rear}" ]]; then
      echo "xt16-baseline requires: xt16-baseline OPERATOR FRONT_M LEFT_M RIGHT_M REAR_M" >&2
      usage >&2
      exit 2
    fi
    exec "${PYTHON_BIN}" "${XT16_CAPTURE}" \
      --scene baseline \
      --operator "${operator}" \
      --measured-front-m "${front}" \
      --measured-left-m "${left}" \
      --measured-right-m "${right}" \
      --measured-rear-m "${rear}"
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
