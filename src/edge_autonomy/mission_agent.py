from __future__ import annotations

import json
import os
import re
import subprocess
import time
import urllib.error
import urllib.request
from collections import deque
from pathlib import Path
from typing import Any, Dict, Iterable, List, Optional, Tuple


DEFAULT_LOCAL_ASK_SCRIPT = "/home/unitree/llm_runtime/scripts/ask_qwen.sh"
DEFAULT_LOCAL_MODEL = "/home/unitree/models/Qwen_Qwen3-4B-Instruct-2507-Q4_K_M.gguf"
DEFAULT_CLOUD_BASE_URL = "https://api.deepseek.com"
DEFAULT_CLOUD_MODEL = "deepseek-chat"

MISSION_SCHEMA = "go2w_mission_plan_v1"
SUPPORTED_PROVIDERS = {"cloud", "local"}


AGENT_SYSTEM_PROMPT = """You are the GO2W robot dog mission agent.
Return a concise helpful reply. Also reason in terms of two step types:
motion steps move between registered topology nodes, and behavior steps do
non-chassis actions such as chat, speak, photo, report, or wait.
Never output raw velocity, cmd_vel, Unitree API ids, or direct motor commands.
Motion execution is disabled unless a robot-side safety gate explicitly enables it.
If the user is only chatting, answer naturally and do not invent a mission."""


def _now_ms() -> int:
    return int(time.time() * 1000)


def _env_first(*names: str) -> str:
    for name in names:
        value = os.environ.get(name)
        if value:
            return value
    return ""


def _read_text_file(path: Path) -> str:
    try:
        if path.exists() and path.is_file():
            return path.read_text(encoding="utf-8").strip()
    except OSError:
        return ""
    return ""


def _cloud_api_key_from_file() -> str:
    candidates = []
    env_path = os.environ.get("GO2W_CLOUD_LLM_API_KEY_FILE")
    if env_path:
        candidates.append(Path(env_path))
    candidates.append(Path.cwd() / "artifacts" / "cloud_llm_api.key")
    try:
        candidates.append(Path.home() / ".config" / "go2w" / "cloud_llm_api.key")
    except RuntimeError:
        pass
    for path in candidates:
        value = _read_text_file(path)
        if value:
            return value
    return ""


def _chat_completions_url(base_url: str) -> str:
    base = (base_url or DEFAULT_CLOUD_BASE_URL).rstrip("/")
    if base.endswith("/chat/completions"):
        return base
    if base.endswith("/v1"):
        return base + "/chat/completions"
    return base + "/v1/chat/completions"


def call_openai_compatible_chat(
    message: str,
    *,
    system_prompt: str = AGENT_SYSTEM_PROMPT,
    base_url: str = "",
    api_key: str = "",
    model: str = "",
    timeout_s: int = 45,
    max_tokens: int = 512,
    temperature: float = 0.2,
) -> Dict[str, Any]:
    key = api_key or _env_first("GO2W_CLOUD_LLM_API_KEY", "DEEPSEEK_API_KEY", "OPENAI_API_KEY") or _cloud_api_key_from_file()
    url_base = base_url or _env_first("GO2W_CLOUD_LLM_BASE_URL", "DEEPSEEK_BASE_URL", "OPENAI_BASE_URL") or DEFAULT_CLOUD_BASE_URL
    model_name = model or os.environ.get("GO2W_CLOUD_LLM_MODEL") or DEFAULT_CLOUD_MODEL
    if not key:
        return {
            "ok": False,
            "provider": "cloud",
            "model": model_name,
            "error": "cloud LLM API key is not configured",
            "configured": False,
        }

    payload = {
        "model": model_name,
        "messages": [
            {"role": "system", "content": system_prompt},
            {"role": "user", "content": message},
        ],
        "temperature": float(temperature),
        "max_tokens": int(max_tokens),
    }
    data = json.dumps(payload, ensure_ascii=False).encode("utf-8")
    request = urllib.request.Request(
        _chat_completions_url(url_base),
        data=data,
        headers={
            "Authorization": "Bearer " + key,
            "Content-Type": "application/json; charset=utf-8",
        },
        method="POST",
    )
    start = time.time()
    try:
        with urllib.request.urlopen(request, timeout=timeout_s) as response:
            raw = response.read().decode("utf-8", errors="replace")
    except urllib.error.HTTPError as exc:
        body = exc.read().decode("utf-8", errors="replace")
        return {
            "ok": False,
            "provider": "cloud",
            "model": model_name,
            "error": "cloud LLM HTTP %s: %s" % (exc.code, body[:500]),
            "configured": True,
            "elapsed_s": round(time.time() - start, 3),
        }
    except (urllib.error.URLError, TimeoutError, OSError) as exc:
        return {
            "ok": False,
            "provider": "cloud",
            "model": model_name,
            "error": "cloud LLM request failed: %s" % exc,
            "configured": True,
            "elapsed_s": round(time.time() - start, 3),
        }

    try:
        obj = json.loads(raw)
        choices = obj.get("choices") if isinstance(obj, dict) else None
        first = choices[0] if isinstance(choices, list) and choices else {}
        msg = first.get("message") if isinstance(first, dict) else {}
        reply = msg.get("content") if isinstance(msg, dict) else ""
    except (json.JSONDecodeError, KeyError, TypeError):
        obj = {"raw": raw}
        reply = raw

    return {
        "ok": bool(reply),
        "provider": "cloud",
        "model": model_name,
        "reply": str(reply or ""),
        "raw_response": obj,
        "configured": True,
        "elapsed_s": round(time.time() - start, 3),
    }


def call_local_llm_chat(
    message: str,
    *,
    system_prompt: str = AGENT_SYSTEM_PROMPT,
    ask_script: str = DEFAULT_LOCAL_ASK_SCRIPT,
    model_path: str = DEFAULT_LOCAL_MODEL,
    timeout_s: int = 120,
    max_tokens: int = 512,
) -> Dict[str, Any]:
    script = ask_script or DEFAULT_LOCAL_ASK_SCRIPT
    if not Path(script).exists():
        return {
            "ok": False,
            "provider": "local",
            "model": model_path,
            "error": "local LLM ask script not found: %s" % script,
            "configured": False,
        }
    env = os.environ.copy()
    env["MODEL_PATH"] = model_path or env.get("MODEL_PATH", DEFAULT_LOCAL_MODEL)
    start = time.time()
    try:
        completed = subprocess.run(
            [script, "--max-tokens", str(int(max_tokens)), "--system", system_prompt, message],
            text=True,
            encoding="utf-8",
            errors="replace",
            capture_output=True,
            timeout=timeout_s,
            env=env,
        )
    except (OSError, subprocess.TimeoutExpired) as exc:
        return {
            "ok": False,
            "provider": "local",
            "model": env["MODEL_PATH"],
            "error": "local LLM request failed: %s" % exc,
            "configured": True,
            "elapsed_s": round(time.time() - start, 3),
        }

    reply = completed.stdout.strip()
    try:
        from .local_llm_planner import parse_ask_qwen_answer

        reply = parse_ask_qwen_answer(completed.stdout).strip()
    except Exception:
        pass
    return {
        "ok": completed.returncode == 0 and bool(reply),
        "provider": "local",
        "model": env["MODEL_PATH"],
        "reply": reply,
        "returncode": completed.returncode,
        "stderr": completed.stderr[-2000:],
        "configured": True,
        "elapsed_s": round(time.time() - start, 3),
    }


def _load_registry_nodes(registry_path: str, map_id: str) -> Tuple[List[Dict[str, Any]], Dict[str, Any]]:
    if not registry_path:
        return [], {}
    path = Path(registry_path)
    if not path.exists():
        return [], {}
    data = json.loads(path.read_text(encoding="utf-8"))
    for item in data.get("maps", []):
        if item.get("map_id") == map_id:
            nodes = item.get("topology_nodes", [])
            return [n for n in nodes if isinstance(n, dict)], item
    return [], {}


def _iter_node_terms(node: Dict[str, Any]) -> Iterable[str]:
    for key in ("node_id", "name"):
        value = node.get(key)
        if isinstance(value, str) and value:
            yield value
    aliases = node.get("aliases")
    if isinstance(aliases, list):
        for value in aliases:
            if isinstance(value, str) and value:
                yield value


def _match_nodes(message: str, nodes: List[Dict[str, Any]]) -> List[Dict[str, Any]]:
    text = message.lower()
    matches: List[Dict[str, Any]] = []
    seen = set()
    for node in nodes:
        node_id = str(node.get("node_id") or "")
        if not node_id or node_id in seen:
            continue
        best_index: Optional[int] = None
        best_term = ""
        for term in _iter_node_terms(node):
            idx = text.find(term.lower())
            if idx < 0:
                continue
            if best_index is None or idx < best_index or (idx == best_index and len(term) > len(best_term)):
                best_index = idx
                best_term = term
        if best_index is not None:
            seen.add(node_id)
            matches.append(
                {
                    "node_id": node_id,
                    "name": node.get("name") or node_id,
                    "matched_term": best_term,
                    "first_index": best_index,
                }
            )
    matches.sort(key=lambda item: (int(item.get("first_index", 10**9)), -len(str(item.get("matched_term") or ""))))
    return matches


def _contains_any(message: str, terms: Iterable[str]) -> bool:
    lower = message.lower()
    for term in terms:
        if not term:
            continue
        if term.isascii() and re.match(r"^[a-z0-9_ -]+$", term):
            if re.search(r"\b" + re.escape(term.lower()) + r"\b", lower):
                return True
        elif term in lower:
            return True
    return False


def _requests_motion(message: str) -> bool:
    return _contains_any(
        message,
        (
            "go",
            "navigate",
            "move",
            "walk",
            "route",
            "from",
            "to",
            "\u53bb",
            "\u8d70",
            "\u8def\u7ebf",
            "\u4ece",
        ),
    )


def _requests_photo(message: str) -> bool:
    return _contains_any(message, ("photo", "picture", "capture", "keyframe", "\u62cd\u7167", "\u7167\u7247", "\u56fe\u50cf"))


def _requests_speech(message: str) -> bool:
    return _contains_any(message, ("speak", "say", "tell", "\u8bf4", "\u64ad\u62a5", "\u8bed\u97f3"))


def _requests_report(message: str) -> bool:
    return _contains_any(message, ("report", "summary", "\u6c47\u62a5", "\u603b\u7ed3"))


def _bfs_route(adjacency: Dict[str, Any], start: str, goal: str) -> List[str]:
    if not start or not goal:
        return []
    if start == goal:
        return [start]
    queue = deque([[start]])
    visited = {start}
    while queue:
        path = queue.popleft()
        current = path[-1]
        neighbors = adjacency.get(current, [])
        if not isinstance(neighbors, list):
            continue
        for neighbor in neighbors:
            next_node = str(neighbor)
            if next_node in visited:
                continue
            next_path = path + [next_node]
            if next_node == goal:
                return next_path
            visited.add(next_node)
            queue.append(next_path)
    return []


def _registry_adjacency(nodes: List[Dict[str, Any]], selected_map: Dict[str, Any]) -> Dict[str, List[str]]:
    node_ids = [str(node.get("node_id") or "") for node in nodes if node.get("node_id")]
    known = set(node_ids)
    adjacency: Dict[str, List[str]] = {node_id: [] for node_id in node_ids}
    edges = selected_map.get("topology_edges") if isinstance(selected_map, dict) else []
    if not isinstance(edges, list):
        return adjacency
    for edge in edges:
        if not isinstance(edge, dict):
            continue
        source = str(edge.get("from") or edge.get("from_node") or edge.get("source") or edge.get("a") or "")
        target = str(edge.get("to") or edge.get("to_node") or edge.get("target") or edge.get("b") or "")
        if source not in known or target not in known:
            continue
        if target not in adjacency[source]:
            adjacency[source].append(target)
        if edge.get("bidirectional", True) and source not in adjacency[target]:
            adjacency[target].append(source)
    return adjacency


def _normal_communication_policy() -> Dict[str, Any]:
    return {
        "mode": "normal",
        "send": ["task_state", "mission_decision", "navigation_feedback", "world_state_summary"],
        "drop": [],
        "reason": "normal link",
    }


def build_mission_preview(
    message: str,
    *,
    registry_path: str = "",
    map_id: str = "go2w_real_site",
    current_node: str = "",
    llm_reply: str = "",
) -> Dict[str, Any]:
    nodes, selected_map = _load_registry_nodes(registry_path, map_id)
    matches = _match_nodes(message, nodes)
    steps: List[Dict[str, Any]] = []
    behavior_count = 0
    motion_count = 0

    motion_requested = _requests_motion(message)
    if motion_requested:
        from_node = current_node
        to_node = ""
        if len(matches) >= 2:
            from_node = str(matches[0]["node_id"])
            to_node = str(matches[-1]["node_id"])
        elif len(matches) == 1:
            to_node = str(matches[0]["node_id"])

        motion_count += 1
        step: Dict[str, Any] = {
            "step_id": "motion_%d" % motion_count,
            "type": "motion",
            "from_node": from_node or "current_pose",
            "to_node": to_node,
            "executor": "nav_core",
            "execution_enabled": False,
            "status": "needs_target" if not to_node else "pending_validation",
            "reason": "motion intent parsed from user message",
        }
        if registry_path and to_node:
            registry_route = _bfs_route(_registry_adjacency(nodes, selected_map), from_node, to_node)
            if registry_route:
                step["route"] = registry_route
                step["status"] = "validated"
                step["validation"] = {
                    "accepted": True,
                    "source": "registry_topology_edges",
                }
            try:
                from .cloud_llm_planner import build_waypoint_connectivity

                connectivity = build_waypoint_connectivity(registry_path, map_id=map_id)
                known = set(str(v) for v in connectivity.get("nodes", []))
                adjacency = connectivity.get("adjacency", {})
                if step.get("status") == "validated":
                    pass
                elif from_node and from_node in known and to_node in known and isinstance(adjacency, dict):
                    route = _bfs_route(adjacency, from_node, to_node)
                    if route:
                        step["route"] = route
                        step["status"] = "validated"
                        step["validation"] = {
                            "accepted": True,
                            "source": "waypoint_connectivity",
                            "edge_source": connectivity.get("edge_source"),
                        }
                    else:
                        step["status"] = "blocked"
                        step["validation"] = {
                            "accepted": False,
                            "reason": "no route in waypoint connectivity",
                        }
                elif not from_node:
                    step["validation"] = {
                        "accepted": False,
                        "reason": "current node is unknown; route validation deferred",
                    }
                else:
                    step["validation"] = {
                        "accepted": False,
                        "reason": "from or target node is not in connectivity graph",
                    }
            except Exception as exc:
                step["validation"] = {"accepted": False, "reason": "route validation error: %s" % exc}
        steps.append(step)

    if _requests_photo(message):
        behavior_count += 1
        target_node = str(matches[-1]["node_id"]) if matches else ""
        steps.append(
            {
                "step_id": "behavior_%d" % behavior_count,
                "type": "behavior",
                "behavior": "capture_photo",
                "target_node": target_node,
                "execution_enabled": False,
                "status": "planned",
            }
        )
    if _requests_speech(message):
        behavior_count += 1
        steps.append(
            {
                "step_id": "behavior_%d" % behavior_count,
                "type": "behavior",
                "behavior": "speak",
                "text": llm_reply or message,
                "execution_enabled": False,
                "status": "planned",
            }
        )
    if _requests_report(message):
        behavior_count += 1
        steps.append(
            {
                "step_id": "behavior_%d" % behavior_count,
                "type": "behavior",
                "behavior": "report",
                "text": llm_reply or "mission preview ready",
                "execution_enabled": False,
                "status": "planned",
            }
        )

    intent_kind = "chat"
    if motion_count and behavior_count:
        intent_kind = "mixed"
    elif motion_count:
        intent_kind = "motion"
    elif behavior_count:
        intent_kind = "behavior"

    if not steps:
        steps.append(
            {
                "step_id": "chat_1",
                "type": "chat",
                "behavior": "chat",
                "execution_enabled": False,
                "status": "answered",
            }
        )

    return {
        "schema": MISSION_SCHEMA,
        "schema_version": 1,
        "plan_id": "mission_%d" % _now_ms(),
        "intent_kind": intent_kind,
        "map_id": map_id,
        "matched_nodes": matches,
        "steps": steps,
        "communication_policy": _normal_communication_policy(),
        "execution_enabled": False,
        "motion_authority": "robot_side_nav_core_and_safety_gate",
        "llm_direct_motion": False,
    }


def handle_agent_message(
    message: str,
    *,
    provider: str = "cloud",
    registry_path: str = "",
    map_id: str = "go2w_real_site",
    current_node: str = "",
    cloud_base_url: str = "",
    cloud_api_key: str = "",
    cloud_model: str = "",
    local_ask_script: str = DEFAULT_LOCAL_ASK_SCRIPT,
    local_model: str = DEFAULT_LOCAL_MODEL,
    timeout_s: int = 45,
    max_tokens: int = 512,
) -> Dict[str, Any]:
    selected_provider = provider or "cloud"
    if selected_provider not in SUPPORTED_PROVIDERS:
        return {
            "ok": False,
            "error": "provider must be one of: cloud, local",
            "provider": selected_provider,
            "execution_enabled": False,
        }
    if selected_provider == "cloud":
        llm = call_openai_compatible_chat(
            message,
            base_url=cloud_base_url,
            api_key=cloud_api_key,
            model=cloud_model,
            timeout_s=timeout_s,
            max_tokens=max_tokens,
        )
    else:
        llm = call_local_llm_chat(
            message,
            ask_script=local_ask_script,
            model_path=local_model,
            timeout_s=timeout_s,
            max_tokens=max_tokens,
        )

    reply = str(llm.get("reply") or "")
    if not reply and not llm.get("ok"):
        reply = str(llm.get("error") or "")
    mission_plan = build_mission_preview(
        message,
        registry_path=registry_path,
        map_id=map_id,
        current_node=current_node,
        llm_reply=reply,
    )
    return {
        "ok": bool(llm.get("ok")),
        "provider": selected_provider,
        "reply": reply,
        "llm": llm,
        "mission_plan": mission_plan,
        "readonly": True,
        "execution_enabled": False,
    }
