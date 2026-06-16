#!/usr/bin/env bash
set -euo pipefail

# ═══════════════════════════════════════════════════════════════════════════════
# build_multi_pcd.sh — Multi-PCD mapping toolkit
# ═══════════════════════════════════════════════════════════════════════════════
#
# Atomic operations for building, switching, and verifying multi-PCD maps.
# Compose them in any order. Each command is one Gateway interaction.
#
# ─── NAMING CONVENTIONS ───────────────────────────────────────────────────
#   Map IDs:         map_<area>          (e.g. map_701, map_701_left)
#   Mapping origin:  mapping_origin_<map> (e.g. mapping_origin_701_left)
#   Transition:      transition_<from>_to_<to> (recorded in <from>'s frame)
#   Topology nodes:  <person>_station, <place>_corridor, etc.
#   PCD files:       /home/unitree/maps/staging/<map_id>.pcd
#
# ─── HOW TO "EXIT" CURRENT PCD ────────────────────────────────────────────
#   There is no explicit "exit PCD" command. start_mapping implicitly resets
#   the SLAM internal map — the old PCD stays on disk, memory is cleared,
#   and the robot's current position becomes (0,0,0) in a fresh map.
#   All PCDs live in ONE registry (go2w_multi_map_registry_v2.json).
#
# ─── GATEWAY RESPONSES ────────────────────────────────────────────────────
#   start_mapping:   {"accepted":true, "action":"start_mapping", ...}
#                    REJECT: operator_ack_required_for_start_mapping
#   end_mapping:     {"accepted":true, "action":"end_mapping", "map_path":"..."}
#                    REJECT: operator_ack_required_for_end_mapping
#   relocate:        {"accepted":true, "action":"relocate", "localization_verified":true, ...}
#                    REJECT: relocalization_active_anchor_not_found (not verified)
#                            relocalization_initial_pose_name_mismatch (name!=anchor_id)
#                            PCD file not found, ICP match failure
#   switch_pcd.sh:   same as relocate — sends relocate with target map's PCD+anchor
#
# ─── COMPLETE 3-PCD BUILD FLOW ────────────────────────────────────────────
#
#   ## PREP
#   bash scripts/start_go2w_slam_stack.sh
#
#   ## MAP 1: map_701 (701中心区)
#   build_multi_pcd.sh start
#       → Gateway: {"accepted":true}  — old map cleared, new map at (0,0,0)
#   # ... push robot to scan area ...
#   build_multi_pcd.sh end map_701
#       → Gateway: {"accepted":true, "map_path":"...staging/map_701.pcd"}
#       → PCD saved. SLAM still running but not localized until relocate.
#   build_multi_pcd.sh relocate
#       → Gateway: {"accepted":true, "localization_verified":true}
#       → RViz2 shows PCD. Robot is localized in map_701's frame.
#
#   ## TRANSITION: map_701 → map_701_left
#   # ... push robot to map_701_left's future origin spot ...
#   build_multi_pcd.sh mark-transition map_701_left
#       → Records current SLAM pose as transition_to_map_701_left
#       → Prints JSON template for registry
#       → *** DO NOT MOVE ROBOT ***
#
#   ## MAP 2: map_701_left (701左侧)
#   build_multi_pcd.sh start
#       → Gateway: {"accepted":true}  — IMPLICITLY EXITS map_701's PCD
#       → New map at (0,0,0) (robot's current position = map_701_left's origin)
#   # ... push robot to scan area ...
#   build_multi_pcd.sh end map_701_left
#       → Gateway: {"accepted":true}
#   build_multi_pcd.sh verify-switch map_701
#       → Tests: can we relocate back to map_701 from this spot?
#       → Success = transition coordinates are correct, PCDs aligned
#
#   ## MAP 3: map_terrace_wc (露台+厕所走廊)
#   # Return to map_701 via switch, push to terrace origin, repeat...
#
# ─── SUBCOMMANDS ──────────────────────────────────────────────────────────
#   start                         Start mapping (implicitly exits previous PCD)
#   end <map_id> [--pcd <path>]   End mapping, save PCD to staging/
#   relocate [--anchor <id>]      Relocate (default: mapping_origin of chosen map)
#   mark-transition <to_map_id>   Snapshot current pose as transition anchor
#   verify-switch <other_map_id>  Test bidirectional relocation
#   list-maps                     Show all maps in registry
# ═══════════════════════════════════════════════════════════════════════════════

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
REGISTRY="${REPO_ROOT}/configs/maps/go2w_multi_map_registry_v2.json"
GATEWAY_CLIENT="${REPO_ROOT}/robot/slam_gateway_refactor/build/slam_llm_command_client"
STAGING_DIR="/home/unitree/maps/staging"
PYTHON_BIN="${GO2W_PYTHON:-python3}"

ACTION="${1:-}"
shift || true

usage() {
    cat <<'EOF'
Usage: bash scripts/build_multi_pcd.sh <subcommand> [args]

Subcommands:
  start                         Start SLAM mapping (implicitly exits previous PCD).
  end <map_id> [--pcd <path>]   End mapping, save PCD to staging/.
  relocate [--anchor <id>]      Relocate using mapping_origin (or specified anchor).
  mark-transition <to_map_id>   Record transition anchor at current pose.
  verify-switch <other_map_id>  Test relocating to another map from current spot.
  list-maps                     Show registry summary.

Naming conventions:
  Map IDs:        map_<area>           (e.g. map_701, map_701_left)
  Mapping origin: mapping_origin_<map> (e.g. mapping_origin_701_left)
  Transition:     transition_<from>_to_<to>
  Topology nodes: <person>_station, <place>_corridor, etc.
  PCD files:      /home/unitree/maps/staging/<map_id>.pcd

All maps share ONE registry: configs/maps/go2w_multi_map_registry_v2.json
PCDs are saved to staging/ — migrate to final paths after verification.
EOF
}

# ── Helpers ──
gateway_cmd() {
    local json="$1"
    "$PYTHON_BIN" -c "
import json, subprocess, sys
client = '$GATEWAY_CLIENT'
payload = sys.argv[1] + '\n'
p = subprocess.Popen([client, 'eth0'], stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
stdout, stderr = p.communicate(payload, timeout=30)
if p.returncode != 0:
    print(f'Gateway error: {stderr.strip()[-300:]}', file=sys.stderr)
    sys.exit(1)
decoder = json.JSONDecoder()
objs = []
for i, ch in enumerate(stdout):
    if ch == '{':
        try:
            val, _ = decoder.raw_decode(stdout[i:])
            if isinstance(val, dict): objs.append(val)
        except: pass
if not objs:
    print(stdout.strip()[-500:])
    sys.exit(1)
# Prefer the dict with 'accepted' key (the command response),
# then the one with 'world_state', then the last one.
target = None
for o in objs:
    if 'accepted' in o:
        target = o
        break
if target is None:
    for o in objs:
        if 'world_state' in o:
            target = o
            break
if target is None:
    target = objs[-1]
print(json.dumps(target, indent=2, ensure_ascii=False))
" "$json"
}

registry_lookup() {
    "$PYTHON_BIN" -c "
import json
with open('$REGISTRY') as f:
    reg = json.load(f)
for m in reg['maps']:
    if m['map_id'] == '$1':
        print(m.get('$2', ''))
        break
"
}

# ═══════════════════════════════════════════════════════════════════════════
#  start — implicitly exits previous PCD, starts fresh map
# ═══════════════════════════════════════════════════════════════════════════
if [[ "$ACTION" == "start" ]]; then
    echo "╔══════════════════════════════════════════════════════════════════╗"
    echo "║  START MAPPING (exits previous PCD, fresh map at robot position)║"
    echo "╚══════════════════════════════════════════════════════════════════╝"
    echo ""
    echo "  Current position becomes (0,0,0) in new map frame."
    echo "  Old PCD stays on disk — NOT deleted."
    echo ""
    read -p "  Ready? (Enter to start, Ctrl+C to abort) " _

    RESP=$(gateway_cmd '{"action":"start_mapping","operator_ack":true,"slam_type":"indoor"}')
    echo "$RESP"

    ACCEPTED=$(echo "$RESP" | "$PYTHON_BIN" -c "import sys,json; print(json.load(sys.stdin).get('accepted',False))" 2>/dev/null || echo "False")
    if [[ "$ACCEPTED" != "True" ]]; then
        echo "ERROR: start_mapping rejected. Is SLAM running? Run: ps aux | grep unitree_slam" >&2
        exit 1
    fi
    echo ""
    echo "  ✅ Mapping ACTIVE. Push robot SLOWLY to scan."
    echo "  When done: build_multi_pcd.sh end <map_id>"
    exit 0
fi

# ═══════════════════════════════════════════════════════════════════════════
#  end <map_id> [--pcd <path>] — save PCD to staging/
# ═══════════════════════════════════════════════════════════════════════════
if [[ "$ACTION" == "end" ]]; then
    MAP_ID="${1:-}"
    if [[ -z "$MAP_ID" ]]; then
        echo "ERROR: map_id required. Usage: build_multi_pcd.sh end <map_id> [--pcd <path>]" >&2
        exit 2
    fi
    shift

    PCD_PATH=""
    if [[ "${1:-}" == "--pcd" ]]; then
        PCD_PATH="$2"
        shift 2 || true
    fi
    if [[ -z "$PCD_PATH" ]]; then
        PCD_PATH=$(registry_lookup "$MAP_ID" "pcd_path" 2>/dev/null || echo "")
    fi
    if [[ -z "$PCD_PATH" ]]; then
        PCD_PATH="$STAGING_DIR/${MAP_ID}.pcd"
    fi

    MAP_EXISTS=$(registry_lookup "$MAP_ID" "map_id" 2>/dev/null || echo "")

    echo "╔══════════════════════════════════════════════════════════════════╗"
    echo "║  END MAPPING & SAVE PCD                                         ║"
    echo "╚══════════════════════════════════════════════════════════════════╝"
    echo ""
    echo "  Map ID:  $MAP_ID"
    echo "  PCD:     $PCD_PATH  (staging)"
    if [[ -z "$MAP_EXISTS" ]]; then
        echo "  ⚠  '$MAP_ID' not in registry — add it after build."
    fi
    echo ""
    read -p "  Save? (Enter, Ctrl+C abort) " _

    mkdir -p "$STAGING_DIR"

    RESP=$(gateway_cmd "{\"action\":\"end_mapping\",\"operator_ack\":true,\"map_path\":\"$PCD_PATH\"}")
    echo "$RESP"

    ACCEPTED=$(echo "$RESP" | "$PYTHON_BIN" -c "import sys,json; print(json.load(sys.stdin).get('accepted',False))" 2>/dev/null || echo "False")
    if [[ "$ACCEPTED" != "True" ]]; then
        echo "ERROR: end_mapping rejected" >&2
        exit 1
    fi

    if [[ -f "$PCD_PATH" ]]; then
        SIZE=$(stat -c%s "$PCD_PATH" 2>/dev/null || echo "?")
        echo ""
        echo "  ✅ PCD saved: $PCD_PATH ($SIZE bytes)"
    else
        echo "  ⚠  PCD file not found — check Gateway logs" >&2
    fi

    echo ""
    echo "  Next:"
    echo "    build_multi_pcd.sh relocate              # verify PCD"
    echo "    build_multi_pcd.sh mark-transition <map>  # record transition"
    exit 0
fi

# ═══════════════════════════════════════════════════════════════════════════
#  relocate [--anchor <id>]
# ═══════════════════════════════════════════════════════════════════════════
if [[ "$ACTION" == "relocate" ]]; then
    ANCHOR=""
    if [[ "${1:-}" == "--anchor" ]]; then
        ANCHOR="$2"
        shift 2 || true
    fi
    if [[ -z "$ANCHOR" ]]; then
        echo "Which map's mapping_origin?"
        echo ""
        "$PYTHON_BIN" -c "
import json
with open('$REGISTRY') as f:
    reg = json.load(f)
for m in reg['maps']:
    print(f\"  {m['map_id']}  →  {m.get('mapping_origin_anchor_id','?')}\")
"
        echo ""
        read -p "  map_id: " MAP_FOR_RELOC
        ANCHOR=$(registry_lookup "$MAP_FOR_RELOC" "mapping_origin_anchor_id" 2>/dev/null || echo "")
        if [[ -z "$ANCHOR" ]]; then
            echo "ERROR: no mapping_origin for '$MAP_FOR_RELOC'" >&2
            exit 1
        fi
        PCD_FOR_RELOC=$(registry_lookup "$MAP_FOR_RELOC" "pcd_path" 2>/dev/null || echo "")
    else
        MAP_FOR_RELOC=$("$PYTHON_BIN" -c "
import json
with open('$REGISTRY') as f:
    reg = json.load(f)
for m in reg['maps']:
    for a in m.get('relocalization_anchors',[]):
        if a['anchor_id'] == '$ANCHOR':
            print(m['map_id'])
            break
")
        PCD_FOR_RELOC=$(registry_lookup "$MAP_FOR_RELOC" "pcd_path" 2>/dev/null || echo "")
    fi

    echo "Relocating: $ANCHOR (map: $MAP_FOR_RELOC, PCD: $PCD_FOR_RELOC)"
    echo ""
    echo "  Expected Gateway response:"
    echo "    {\"accepted\":true, \"localization_verified\":true, ...}"
    echo ""

    # Send relocate directly via Gateway (bypasses go2w_accept.sh which uses old registry)
    RELOC_JSON=$("$PYTHON_BIN" -c "
import json
with open('$REGISTRY') as f:
    reg = json.load(f)
for m in reg['maps']:
    if m['map_id'] == '$MAP_FOR_RELOC':
        for a in m.get('relocalization_anchors', []):
            if a['anchor_id'] == '$ANCHOR':
                p = a['pose']
                cmd = {
                    'action': 'relocate',
                    'operator_ack': True,
                    'map_id': 'go2w_real_site',  # Gateway hardcoded map_id
                    'map_path': m['pcd_path'],
                    'anchor_id': a['anchor_id'],
                    'initial_pose': {
                        'name': a['anchor_id'],
                        'x': p.get('x', 0.0),
                        'y': p.get('y', 0.0),
                        'z': p.get('z', 0.0),
                        'q_x': p.get('q_x', 0.0),
                        'q_y': p.get('q_y', 0.0),
                        'q_z': p.get('q_z', 0.0),
                        'q_w': p.get('q_w', 1.0),
                        'speed': 0.0,
                        'mode': 0
                    }
                }
                print(json.dumps(cmd, ensure_ascii=False))
                break
        break
")
    gateway_cmd "$RELOC_JSON"
    exit $?
fi

# ═══════════════════════════════════════════════════════════════════════════
#  mark-transition <to_map_id>
# ═══════════════════════════════════════════════════════════════════════════
if [[ "$ACTION" == "mark-transition" ]]; then
    TO_MAP="${1:-}"
    if [[ -z "$TO_MAP" ]]; then
        echo "ERROR: to_map_id required." >&2
        echo "Usage: build_multi_pcd.sh mark-transition <to_map_id>" >&2
        "$PYTHON_BIN" -c "
import json
with open('$REGISTRY') as f:
    reg = json.load(f)
for m in reg['maps']:
    print(f\"  {m['map_id']}  — {m.get('name','')}\")
"
        exit 2
    fi

    TO_NAME=$(registry_lookup "$TO_MAP" "name" 2>/dev/null || echo "$TO_MAP")
    TO_ORIGIN=$(registry_lookup "$TO_MAP" "mapping_origin_anchor_id" 2>/dev/null || echo "mapping_origin_${TO_MAP}")

    TRANSITION_ID="transition_to_${TO_MAP}"

    echo "╔══════════════════════════════════════════════════════════════════╗"
    echo "║  RECORD TRANSITION ANCHOR → $TO_MAP                            ║"
    echo "╚══════════════════════════════════════════════════════════════════╝"
    echo ""
    echo "  Records current SLAM pose as transition anchor in current PCD's frame."
    echo "  FROM: currently loaded PCD   TO: $TO_MAP ($TO_NAME)"
    echo ""
    echo "  PREREQUISITES:"
    echo "    1. PCD loaded + localized (run relocate first)"
    echo "    2. Robot at $TO_MAP's future mapping_origin"
    echo "    3. This spot covered by current PCD's scan"
    echo ""
    read -p "  Snapshot? (Enter, Ctrl+C abort) " _

    echo ""
    echo "  Snapshotting: $TRANSITION_ID"
    bash "$SCRIPT_DIR/go2w_accept.sh" snapshot "${TRANSITION_ID}" 2>/dev/null || {
        echo "  ⚠  Snapshot failed. SLAM localized? PCD loaded?" >&2
        exit 1
    }

    echo ""
    echo "  ╔══════════════════════════════════════════════════════════════════╗"
    echo "  ║  ★ DO NOT MOVE THE ROBOT ★                                     ║"
    echo "  ║  It is at $TO_MAP's mapping_origin.                            ║"
    echo "  ╚══════════════════════════════════════════════════════════════════╝"
    echo ""
    echo "  Fill into registry (current map → transition_anchors):"
    echo ""
    echo "    {"
    echo "      \"anchor_id\":      \"$TRANSITION_ID\","
    echo "      \"connects_to\":    \"$TO_MAP\","
    echo "      \"reverse_anchor\": \"$TO_ORIGIN\","
    echo "      \"pose\": { ... from snapshot output above ... }"
    echo "    }"
    echo ""
    echo "  To build $TO_MAP from this spot (without moving robot):"
    echo "    build_multi_pcd.sh start              # exits current PCD, fresh map"
    echo "    # push robot to scan ${TO_MAP}'s area..."
    echo "    build_multi_pcd.sh end $TO_MAP"
    exit 0
fi

# ═══════════════════════════════════════════════════════════════════════════
#  verify-switch <other_map_id>
# ═══════════════════════════════════════════════════════════════════════════
if [[ "$ACTION" == "verify-switch" ]]; then
    OTHER="${1:-}"
    if [[ -z "$OTHER" ]]; then
        echo "ERROR: other_map_id required." >&2
        echo "Usage: build_multi_pcd.sh verify-switch <other_map_id>" >&2
        "$PYTHON_BIN" -c "
import json
with open('$REGISTRY') as f:
    reg = json.load(f)
for m in reg['maps']:
    print(f\"  {m['map_id']}\")
"
        exit 2
    fi

    REVERSE=$("$PYTHON_BIN" -c "
import json
with open('$REGISTRY') as f:
    reg = json.load(f)
for m in reg['maps']:
    if m['map_id'] == '$OTHER':
        print(m.get('mapping_origin_anchor_id', ''))
        break
else:
    print('MAP_NOT_FOUND')
")

    if [[ "$REVERSE" == "MAP_NOT_FOUND" || -z "$REVERSE" ]]; then
        echo "ERROR: map '$OTHER' not found or has no mapping_origin" >&2
        exit 1
    fi

    OTHER_PCD=$(registry_lookup "$OTHER" "pcd_path" 2>/dev/null || echo "")

    echo "╔══════════════════════════════════════════════════════════════════╗"
    echo "║  VERIFY BIDIRECTIONAL SWITCH → $OTHER                          ║"
    echo "╚══════════════════════════════════════════════════════════════════╝"
    echo ""
    echo "  Tests: can we relocate to $OTHER from current position?"
    echo "  Anchor: $REVERSE  |  PCD: $OTHER_PCD"
    echo ""
    echo "  If this SUCCEEDS, transition coordinates are correct."
    echo "  If this FAILS: check anchor pose, PCD coverage, robot position."
    echo ""

    # Send relocate via Gateway directly (V2 registry)
    echo "  Relocating to $OTHER via $REVERSE..."
    RELOC_JSON=$("$PYTHON_BIN" -c "
import json
with open('$REGISTRY') as f:
    reg = json.load(f)
for m in reg['maps']:
    if m['map_id'] == '$OTHER':
        for a in m.get('relocalization_anchors', []):
            if a['anchor_id'] == '$REVERSE':
                p = a['pose']
                cmd = {
                    'action': 'relocate',
                    'operator_ack': True,
                    'map_id': 'go2w_real_site',  # Gateway hardcoded map_id
                    'map_path': m['pcd_path'],
                    'anchor_id': a['anchor_id'],
                    'initial_pose': {
                        'name': a['anchor_id'],
                        'x': p.get('x', 0.0),
                        'y': p.get('y', 0.0),
                        'z': p.get('z', 0.0),
                        'q_x': p.get('q_x', 0.0),
                        'q_y': p.get('q_y', 0.0),
                        'q_z': p.get('q_z', 0.0),
                        'q_w': p.get('q_w', 1.0),
                        'speed': 0.0,
                        'mode': 0
                    }
                }
                print(json.dumps(cmd, ensure_ascii=False))
                break
        break
")
    gateway_cmd "$RELOC_JSON" || {
        echo ""
        echo "  ❌ FAILED to relocate to $OTHER"
        exit 1
    }

    echo ""
    echo "  ✅ Switched to $OTHER successfully."
    echo "  Run: build_multi_pcd.sh relocate  to return to current map."
    exit 0
fi

# ═══════════════════════════════════════════════════════════════════════════
#  list-maps
# ═══════════════════════════════════════════════════════════════════════════
if [[ "$ACTION" == "list-maps" ]]; then
    "$PYTHON_BIN" -c "
import json
with open('$REGISTRY') as f:
    reg = json.load(f)
print(f'Registry: {reg.get(\"default_map_id\",\"?\")} (v{reg.get(\"version\",\"?\")})')
print(f'Staging:  $STAGING_DIR')
print()
for m in reg['maps']:
    print(f\"  {m['map_id']}\")
    print(f\"    name:     {m.get('name','')}\")
    print(f\"    PCD:      {m['pcd_path']}\")
    print(f\"    origin:   {m.get('mapping_origin_anchor_id','?')}\")
    print(f\"    nodes:    {len(m.get('topology_nodes',[]))}\")
    print(f\"    transitions: {len(m.get('transition_anchors',[]))}\")
    for ta in m.get('transition_anchors',[]):
        print(f\"      → {ta.get('connects_to','?')}  (reverse: {ta.get('reverse_anchor','?')})\")
    print()
"
    exit 0
fi

# ═══════════════════════════════════════════════════════════════════════════
echo "ERROR: unknown action '$ACTION'" >&2
usage >&2
exit 2
