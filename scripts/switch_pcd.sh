#!/usr/bin/env bash
set -euo pipefail

# ─── switch_pcd.sh ─────────────────────────────────────────────────────────
# Switch between PCD maps by relocating through a transition anchor.
#
# Usage:
#   bash scripts/switch_pcd.sh <from_map> <to_map>
#   bash scripts/switch_pcd.sh <from_map> <to_map> --anchor <anchor_id>
#   bash scripts/switch_pcd.sh <from_map> <to_map> --dry-run
#
# The script:
#   1. Looks up the transition anchor connecting the two maps
#   2. Verifies the anchor exists and is verified in the target map
#   3. Relocates the robot using the target map's PCD + transition anchor
#   4. Verifies localization quality
# ────────────────────────────────────────────────────────────────────────────

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
REGISTRY="${REPO_ROOT}/configs/maps/go2w_multi_map_registry_v2.json"
GATEWAY_CLIENT="${REPO_ROOT}/robot/slam_gateway_refactor/build/slam_llm_command_client"
PYTHON_BIN="${GO2W_PYTHON:-python3}"

DRY_RUN=0
FROM_MAP=""
TO_MAP=""
FORCE_ANCHOR=""

usage() {
    cat <<'EOF'
Usage: bash scripts/switch_pcd.sh <from_map> <to_map> [options]

Options:
  --anchor <id>  Force a specific transition anchor (skip auto-detection).
  --dry-run      Print commands without executing.
  --help         Show this help.

Examples:
  bash scripts/switch_pcd.sh map_701_left map_701_right
  bash scripts/switch_pcd.sh map_701_right map_702 --dry-run

EOF
}

while [[ $# -gt 0 ]]; do
    case "$1" in
        --anchor)     FORCE_ANCHOR="$2"; shift 2 ;;
        --dry-run)    DRY_RUN=1; shift ;;
        --help|-h)    usage; exit 0 ;;
        -*)           echo "Unknown option: $1" >&2; usage >&2; exit 2 ;;
        *)
            if [[ -z "$FROM_MAP" ]]; then FROM_MAP="$1"
            elif [[ -z "$TO_MAP" ]]; then TO_MAP="$1"
            else echo "Too many arguments: $1" >&2; usage >&2; exit 2
            fi
            shift
            ;;
    esac
done

if [[ -z "$FROM_MAP" || -z "$TO_MAP" ]]; then
    echo "error: both from_map and to_map are required" >&2
    usage >&2
    exit 2
fi

if [[ "$FROM_MAP" == "$TO_MAP" ]]; then
    echo "error: from_map and to_map are the same ($FROM_MAP)" >&2
    exit 2
fi

info()    { echo -e "\n  >>> $*"; }
warn()    { echo -e "  ⚠  $*" >&2; }
die()     { echo -e "  ✗  $*" >&2; exit 1; }
run_cmd() {
    if [[ "$DRY_RUN" -eq 1 ]]; then
        echo "  [DRY-RUN] $*"
    else
        "$@"
    fi
}

# ── Resolve transition anchor ──
info "Looking up transition: $FROM_MAP → $TO_MAP"

RESOLVE=$("$PYTHON_BIN" -c "
import json, sys

with open('$REGISTRY') as f:
    reg = json.load(f)

maps = {m['map_id']: m for m in reg['maps']}
from_map = maps.get('$FROM_MAP')
to_map = maps.get('$TO_MAP')

if not from_map:
    print(f'ERROR: from_map \"$FROM_MAP\" not in registry', file=sys.stderr)
    sys.exit(1)
if not to_map:
    print(f'ERROR: to_map \"$TO_MAP\" not in registry', file=sys.stderr)
    sys.exit(1)

# Find transition anchor in from_map that connects to to_map
anchor_id = None
reverse_anchor = ''
for ta in from_map.get('transition_anchors', []):
    if ta.get('connects_to') == '$TO_MAP':
        anchor_id = ta['anchor_id']
        reverse_anchor = ta.get('reverse_anchor', '')
        break

if not anchor_id:
    # Try reverse lookup: find anchor in to_map's relocalization_anchors
    # that might be the transition from from_map
    for ra in to_map.get('relocalization_anchors', []):
        aid = ra['anchor_id']
        if 'transition' in aid.lower() and '$FROM_MAP' in aid.lower().replace('_to_', '_from_'):
            # Try heuristic match
            pass  # fall through to error

if not anchor_id and '$FORCE_ANCHOR':
    anchor_id = '$FORCE_ANCHOR'
    # When forcing, try to find reverse_anchor from the forced anchor's entry
    for ta in from_map.get('transition_anchors', []):
        if ta['anchor_id'] == anchor_id:
            reverse_anchor = ta.get('reverse_anchor', '')
            break

if not anchor_id:
    print('ERROR: no transition anchor found from $FROM_MAP to $TO_MAP', file=sys.stderr)
    print('Available transitions from $FROM_MAP:', file=sys.stderr)
    for ta in from_map.get('transition_anchors', []):
        print(f\"  {ta['anchor_id']} → {ta.get('connects_to', '?')}\", file=sys.stderr)
    sys.exit(1)

# Find the anchor in the TARGET map's relocalization_anchors
# reverse_anchor was set above: either from the matched transition_anchors
# entry or from --anchor force lookup. It points to the target map's
# mapping_origin_anchor_id (the transition spot IS the next map's origin).

# Fallback: if reverse_anchor wasn't set, try to derive from anchor_id
if not reverse_anchor:
    # Fallback: derive from anchor_id naming convention (legacy)
    parts = anchor_id.split('_to_')
    if len(parts) == 2:
        reverse_anchor = parts[1] + '_to_' + parts[0]
    else:
        reverse_anchor = anchor_id

target_anchor = None
for ra in to_map.get('relocalization_anchors', []):
    if ra['anchor_id'] == reverse_anchor:
        target_anchor = ra
        break

# If reverse not found, try exact match
if not target_anchor:
    for ra in to_map.get('relocalization_anchors', []):
        if ra['anchor_id'] == anchor_id:
            target_anchor = ra
            break

if not target_anchor:
    print(f'ERROR: transition anchor \"{reverse_anchor}\" not found in target map \"$TO_MAP\" relocalization_anchors', file=sys.stderr)
    print(f'Available relocalization anchors in $TO_MAP:', file=sys.stderr)
    for ra in to_map.get('relocalization_anchors', []):
        print(f\"  {ra['anchor_id']} ({ra.get('status', 'candidate')})\", file=sys.stderr)
    sys.exit(1)

# Output: anchor_id|map_path|pose_x|pose_y|status
pose = target_anchor['pose']
print(f\"{target_anchor['anchor_id']}|{to_map['pcd_path']}|{pose['x']}|{pose['y']}|{target_anchor.get('status','candidate')}\")
")

if [[ $? -ne 0 ]]; then
    echo "$RESOLVE" >&2
    exit 1
fi

ANCHOR_ID=$(echo "$RESOLVE" | cut -d'|' -f1)
TARGET_PCD=$(echo "$RESOLVE" | cut -d'|' -f2)
ANCHOR_X=$(echo "$RESOLVE" | cut -d'|' -f3)
ANCHOR_Y=$(echo "$RESOLVE" | cut -d'|' -f4)
ANCHOR_STATUS=$(echo "$RESOLVE" | cut -d'|' -f5)

echo "  Transition anchor: $ANCHOR_ID"
echo "  Target PCD:        $TARGET_PCD"
echo "  Anchor pose:       ($ANCHOR_X, $ANCHOR_Y)"
echo "  Anchor status:     $ANCHOR_STATUS"

# ── Validate anchor is verified ──
if [[ "$ANCHOR_STATUS" != "verified" && ! "$ANCHOR_STATUS" =~ ^verified_ ]]; then
    die "Anchor '$ANCHOR_ID' status is '$ANCHOR_STATUS' — must be 'verified'. Run calibration first."
fi

# ── Verify robot is at transition location ──
echo ""
echo "  ╔══════════════════════════════════════════════════════════════════╗"
echo "  ║  SWITCH READY                                                    ║"
echo "  ╚══════════════════════════════════════════════════════════════════╝"
echo ""
echo "  The robot will relocate to the $TO_MAP coordinate frame."
echo "  Make sure the robot is physically at the TRANSITION location"
echo "  where both PCDs overlap."
echo ""
echo "  If the robot is NOT at the transition spot:"
echo "    1. First navigate to the transition spot using the current PCD"
echo "    2. Then run this switch script again"
echo ""

if [[ "$DRY_RUN" -ne 1 ]]; then
    read -p "  >>> Ready to switch PCD? (y/N) " CONFIRM
    if [[ ! "$CONFIRM" =~ ^[Yy]$ ]]; then
        echo "  Aborted."
        exit 0
    fi
fi

# ── Execute relocation ──
info "Relocating to target PCD via anchor: $ANCHOR_ID"

# The Gateway relocate command needs:
#   map_id = target map's map_id
#   map_path = target map's pcd_path
#   anchor_id = transition anchor in target map
#   initial_pose = transition anchor's pose (with name == anchor_id)

# Read full anchor pose from registry for the relocate command
RELC_CMD=$("$PYTHON_BIN" -c "
import json
with open('$REGISTRY') as f:
    reg = json.load(f)

to_map = next((m for m in reg['maps'] if m['map_id'] == '$TO_MAP'), None)
if to_map is None:
    print('ERROR: map not found', file=sys.stderr)
    sys.exit(1)
anchor = next((a for a in to_map['relocalization_anchors'] if a['anchor_id'] == '$ANCHOR_ID'), None)
if anchor is None:
    print(f'ERROR: anchor \"$ANCHOR_ID\" not in $TO_MAP relocalization_anchors', file=sys.stderr)
    sys.exit(1)
pose = anchor['pose']

cmd = {
    'action': 'relocate',
    'operator_ack': True,
    'map_id': '$TO_MAP',
    'map_path': to_map['pcd_path'],
    'anchor_id': anchor['anchor_id'],
    'initial_pose': {
        'name': anchor['anchor_id'],
        'x': pose['x'],
        'y': pose['y'],
        'z': pose.get('z', 0.0),
        'q_x': pose.get('q_x', 0.0),
        'q_y': pose.get('q_y', 0.0),
        'q_z': pose.get('q_z', 0.0),
        'q_w': pose.get('q_w', 1.0),
        'speed': 0.0,
        'mode': 0
    }
}
print(json.dumps(cmd, ensure_ascii=False))
")

if [[ "$DRY_RUN" -eq 1 ]]; then
    echo "  [DRY-RUN] Gateway command:"
    echo "$RELC_CMD" | "$PYTHON_BIN" -m json.tool
else
    info "Sending relocate command..."
    RESP=$(echo "$RELC_CMD" | "$GATEWAY_CLIENT" eth0 2>&1 || true)
    echo "$RESP" | "$PYTHON_BIN" -c "
import sys, json
text = sys.stdin.read()
# Find last JSON object
decoder = json.JSONDecoder()
objs = []
for i, ch in enumerate(text):
    if ch == '{':
        try:
            val, _ = decoder.raw_decode(text[i:])
            if isinstance(val, dict):
                objs.append(val)
        except:
            pass
if objs:
    # Prefer dict with 'accepted' key for display
    target = None
    for o in objs:
        if 'accepted' in o:
            target = o
            break
    if target is None:
        target = objs[-1]
    print(json.dumps(target, indent=2, ensure_ascii=False))
else:
    print('Raw response:', text[:500])
"

    ACCEPTED=$(echo "$RESP" | "$PYTHON_BIN" -c "
import sys, json
text = sys.stdin.read()
decoder = json.JSONDecoder()
for i, ch in enumerate(text):
    if ch == '{':
        try:
            val, _ = decoder.raw_decode(text[i:])
            if isinstance(val, dict) and 'accepted' in val:
                print(val['accepted'])
                break
        except: pass
" 2>/dev/null || echo "unknown")

    if [[ "$ACCEPTED" != "True" ]]; then
        warn "Relocation may have failed. Check Gateway response above."
        echo ""
        echo "  Troubleshooting:"
        echo "    1. Is the robot physically at the transition spot?"
        echo "    2. Is the anchor status 'verified'?"
        echo "    3. Run: bash scripts/go2w_accept.sh status  — to check SLAM state"
    fi
fi

# ── Verify localization ──
if [[ "$DRY_RUN" -ne 1 && "$ACCEPTED" == "True" ]]; then
    info "Verifying localization on new PCD..."
    echo "  Running 3 verification cycles..."
    run_cmd bash "$SCRIPT_DIR/go2w_accept.sh" verify "$ANCHOR_ID"
fi

echo ""
echo "  ╔══════════════════════════════════════════════════════════════════╗"
echo "  ║  SWITCH COMPLETE                                                 ║"
echo "  ╚══════════════════════════════════════════════════════════════════╝"
echo ""
echo "  Now on map: $TO_MAP"
echo "  PCD: $TARGET_PCD"
echo ""
echo "  You can now navigate within $TO_MAP using:"
echo "    python3 scripts/run_robot_closed_loop.py --command <node> --map-id $TO_MAP --map-path $TARGET_PCD ..."
echo ""
