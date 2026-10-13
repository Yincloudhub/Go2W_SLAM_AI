"""Local end-to-end test of the cloud LLM chain (no robot needed).

Simulates what Hermes does:
  1. GET /planner/context          (pull world model)
  2. reason about a route          (cloud LLM job, simulated here)
  3. POST /planner/validate_hops   (push hop proposal, robot validates)
  4. POST invalid hops             (negative case must be rejected)
"""
import json
import urllib.request

BASE = "http://127.0.0.1:18080"
TOKEN = "localtest"


def call(method, path, body=None):
    data = json.dumps(body).encode() if body is not None else None
    req = urllib.request.Request(
        BASE + path,
        data=data,
        headers={"Authorization": f"Bearer {TOKEN}",
                 "Content-Type": "application/json"},
        method=method,
    )
    with urllib.request.urlopen(req, timeout=15) as resp:
        return json.loads(resp.read().decode())


# 1. Pull context (simulating user saying 去赵博办公室门口)
ctx = call("GET", "/planner/context?cmd=%E5%8E%BB%E8%B5%B5%E5%8D%9A&map_id=go2w_real_site&registry=configs/maps/go2w_real_site_map_registry.json")
conn = ctx.get("waypoint_connectivity", {})
nodes = conn.get("nodes", [])
adj = conn.get("adjacency", {})
print("[1] GET /planner/context")
print(f"    nodes={len(nodes)} edge_source={conn.get('edge_source')} gateway_status={ctx.get('status', {}).get('reason')}")
print(f"    adjacency: {adj}")

# 2. Simulate cloud LLM spatial reasoning: pick a valid 2-hop route
#    Per adjacency: wp_e643bc -> wp_60497f -> wp_b738c7 (all registry edges).
hops = ["wp_e643bc", "wp_60497f", "wp_b738c7"]
print("[2] Hermes reasoning -> hops:", " -> ".join(hops))

# 3. POST validate_hops
resp = call("POST", "/planner/validate_hops",
            {"hops": hops, "map_id": "go2w_real_site",
             "registry": "configs/maps/go2w_real_site_map_registry.json",
             "reason": "经 wp_b738c7 绕行至赵博办公室门口"})
plan = resp.get("plan", {})
print("[3] POST /planner/validate_hops")
print(f"    accepted={resp.get('accepted')} readonly={resp.get('readonly')} exec={resp.get('execution_enabled')}")
print(f"    plan mode={plan.get('mode')} confidence={plan.get('confidence')}")
print(f"    steps={[s['tool'] + ':' + s.get('arguments', {}).get('target_node', '') for s in plan.get('steps', [])]}")

# 4. Negative case: invalid hop (not in registry)
bad = call("POST", "/planner/validate_hops",
           {"hops": ["wp_e643bc", "does_not_exist"], "map_id": "go2w_real_site",
            "registry": "configs/maps/go2w_real_site_map_registry.json"})
bad_plan = bad.get("plan", {})
print("[4] Negative case (unknown hop)")
print(f"    accepted={bad.get('accepted')} mode={bad_plan.get('mode')} reason={bad_plan.get('reason')}")

# 5. Negative case: disconnected pair
disc = call("POST", "/planner/validate_hops",
            {"hops": ["wp_e643bc", "wp_832687"], "map_id": "go2w_real_site",
             "registry": "configs/maps/go2w_real_site_map_registry.json"})
disc_plan = disc.get("plan", {})
print("[5] Negative case (disconnected pair)")
print(f"    accepted={disc.get('accepted')} mode={disc_plan.get('mode')} reason={disc_plan.get('reason')}")
