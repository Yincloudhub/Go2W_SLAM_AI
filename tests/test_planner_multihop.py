"""
Integration test: simulate the full planner pipeline with coarse_map.
Tests that the LLM prompt contains all necessary information for multi-hop routing.
"""

import sys, os, json

os.chdir("/home/unitree/Go2W_SLAM_AI")
sys.path.insert(0, "src")

from edge_autonomy.local_llm_planner import (
    build_lightweight_planner_context,
    build_lightweight_planner_prompt,
    deterministic_intent_from_context,
)
from edge_autonomy.path_validator import check_path_between_nodes

# Simulate a planner_context similar to what run_robot_closed_loop.py builds
mock_context = {
    "user_command": "zhao_bo_office_front",
    "world_state_summary": {
        "robot": {"localized": True, "nearest_node": {"node_id": "yin_siyuan_station"}},
        "slam": {"health_status": "ok"},
        "map": {"map_id": "go2w_real_site"},
        "topology": {
            "available_nodes": [
                {"node_id": "initial_point", "name": "初始点", "distance_from_robot_m": 1.8,
                 "tags": ["live_verified"], "aliases": []},
                {"node_id": "yin_siyuan_station", "name": "尹思园工位", "distance_from_robot_m": 0.18,
                 "tags": ["live_verified"], "aliases": []},
                {"node_id": "nie_guoli_office_front", "name": "聂国力办公室", "distance_from_robot_m": 3.1,
                 "tags": ["live_verified"], "aliases": []},
                {"node_id": "zhao_bo_office_front", "name": "赵博办公室门口", "distance_from_robot_m": 6.0,
                 "tags": ["live_verified", "photo_required"], "aliases": []},
                {"node_id": "chen_jiayu_station", "name": "陈家宇工位", "distance_from_robot_m": 2.7,
                 "tags": ["live_verified"], "aliases": []},
            ]
        },
    },
    "map_id": "go2w_real_site",
}

# Test 1: Build planner context and check coarse_map
print("=" * 60)
print("TEST 1: Planner context includes coarse_map")
ctx = build_lightweight_planner_context(mock_context)
print(f"  coarse_map length: {len(ctx.get('coarse_map', ''))}")
print(f"  has walls: {chr(0x2588) in ctx.get('coarse_map', '')}")
print(f"  hint present: {bool(ctx.get('navigation_hint'))}")
print(f"  nodes in map: {len(ctx.get('coarse_map_nodes', {}))}")
print(f"  requested_target: {ctx.get('requested_target_guess')}")
print(f"  matched_targets: {len(ctx.get('matched_targets', []))}")
print("  PASS" if all([
    ctx.get('coarse_map'),
    ctx.get('navigation_hint'),
    ctx.get('requested_target_guess') == 'zhao_bo_office_front',
]) else "  FAIL")

# Test 2: Path validation
print("=" * 60)
print("TEST 2: Path validation between topology nodes")
paths = [
    ("yin_siyuan_station", "zhao_bo_office_front"),
    ("yin_siyuan_station", "nie_guoli_office_front"),
    ("nie_guoli_office_front", "zhao_bo_office_front"),
]
all_correct = True
for a, b in paths:
    r = check_path_between_nodes(a, b)
    an = a.split("_")[0]
    bn = b.split("_")[0]
    status = "BLOCKED" if not r["passable"] else "PASS"
    expected_blocked = (a == "yin_siyuan_station" and b == "zhao_bo_office_front") or \
                       (a == "yin_siyuan_station" and b == "nie_guoli_office_front")
    correct = (not r["passable"]) == expected_blocked
    if not correct:
        all_correct = False
    mark = "✓" if correct else "✗"
    print(f"  {mark} {an:10s} -> {bn:10s}: {status} (expected {'BLOCKED' if expected_blocked else 'PASS'})")
print(f"  {'PASS' if all_correct else 'FAIL'}")

# Test 3: Lightweight prompt generation
print("=" * 60)
print("TEST 3: Lightweight prompt contains multi-hop instruction")
prompt = build_lightweight_planner_prompt(mock_context)
has_multi_hop = "MULTI-HOP" in prompt or "multi-hop" in prompt.lower()
has_coarse_map_in_prompt = chr(0x2588) in prompt
print(f"  prompt length: {len(prompt)}")
print(f"  has MULTI-HOP rule: {has_multi_hop}")
print(f"  has coarse_map (██) in prompt: {has_coarse_map_in_prompt}")
print(f"  {'PASS' if has_multi_hop and has_coarse_map_in_prompt else 'FAIL'}")

# Test 4: Deterministic intent correctly identifies target
print("=" * 60)
print("TEST 4: Deterministic intent matching")
intent = deterministic_intent_from_context(mock_context)
print(f"  intent mode: {intent.get('mode')}")
print(f"  intent target: {intent.get('target_node')}")
print(f"  intent confidence: {intent.get('confidence')}")
print(f"  {'PASS' if intent and intent.get('target_node') == 'zhao_bo_office_front' else 'FAIL'}")

# Test 5: Show what the LLM sees (first 1200 chars of prompt)
print("=" * 60)
print("TEST 5: Prompt preview (first 1200 chars)")
print(prompt[:1200])
print("...")

print("=" * 60)
all_pass = all([
    ctx.get('coarse_map') and ctx.get('navigation_hint'),
    all_correct,
    has_multi_hop and has_coarse_map_in_prompt,
    intent and intent.get('target_node') == 'zhao_bo_office_front',
])
print(f"OVERALL: {'ALL TESTS PASSED' if all_pass else 'SOME TESTS FAILED'}")
