from __future__ import annotations

import json
import re
import shlex
import subprocess
import time
from copy import deepcopy
from dataclasses import dataclass
from typing import Any, Protocol


PLAN_REQUIRED_KEYS = {"plan_id", "mode", "confidence", "reason", "steps", "communication_policy", "requires_human_ack"}
PLAN_MODES = {"mapped_navigation", "mapless_scout", "safe_hold", "human_confirm"}
PLAN_TOOLS = {
    "set_communication_policy",
    "create_navigation_subgoal",
    "wait_until",
    "capture_keyframe",
    "start_mapless_scout",
    "request_human_confirm",
    "hold_position",
}
COMMUNICATION_MODES = {"normal", "semantic_only", "keyframe_low_rate", "hold_remote"}
SEND_ITEMS = {"task_state", "risk_events", "keyframe", "semantic_topology", "navigation_feedback", "world_state_summary"}
DROP_ITEMS = {"raw_video", "dense_pointcloud", "full_log", "high_rate_images"}


DEFAULT_SYSTEM_PROMPT = (
    "You are the local planner for a Unitree GO2W robot. "
    "Output exactly one JSON object matching local_llm_plan.schema.json. "
    "Your entire response must start with '{' and end with '}'. "
    "Do not repeat the input. Do not output markdown, comments, natural-language explanation, raw Unitree API ids, or cmd_vel. "
    "Use only registered tools. Safety rules override user requests. "
    "The top-level keys must be exactly: plan_id, mode, confidence, reason, steps, communication_policy, requires_human_ack. "
    "Each step must have exactly: step_id, tool, arguments. "
    "communication_policy must be an object with mode, send, drop, and reason. "
    "For mapped_navigation, include create_navigation_subgoal and wait_until. "
    "create_navigation_subgoal arguments must contain map_id and target_node, not target_node_id. "
    "If target_node is not present in semantic topology, use human_confirm and never create_navigation_subgoal. "
    "If distance_to_requested_target_m is within arrival_distance_m, use hold_position and never create_navigation_subgoal. "
    "If battery_percent is below low_battery_percent and the task is not charging or emergency, ask for human confirmation or return to charging_point before any other navigation. "
    "If bandwidth_kbps is below weak_bandwidth_kbps, the first step must be set_communication_policy with semantic_only mode and drop raw_video, dense_pointcloud, and high_rate_images. "
    "Allowed communication send values are task_state, risk_events, keyframe, semantic_topology, navigation_feedback, world_state_summary. "
    "Allowed communication drop values are raw_video, dense_pointcloud, full_log, high_rate_images. "
    "Never put navigation_feedback or world_state_summary in drop."
)


PROMPT_TEMPLATE = """You must solve the robot planning case below.

Return only one JSON object. Do not repeat these instructions or the input.
Required top-level keys: plan_id, mode, confidence, reason, steps, communication_policy, requires_human_ack.
Allowed modes: mapped_navigation, mapless_scout, safe_hold, human_confirm.
Each step object must contain exactly: step_id, tool, arguments.
communication_policy must contain: mode, send, drop, reason.
Keep JSON compact: confidence must be a number from 0 to 1; reason strings should be short; do not add extra send/drop items.

Hard rules:
- Do not copy any input JSON object.
- Do not invent topology nodes.
- Do not navigate when localization is lost or SLAM is not usable.
- Do not navigate into critical human/crowd risk.
- Do not output raw Unitree API ids such as 1102, 1201, or 1202.

Decision priority, apply in this order before choosing steps:
1. Unknown target / missing topology node: if the requested target is absent from semantic topology node_id values, mode must be human_confirm, include request_human_confirm, and do not create_navigation_subgoal.
2. Already at target: if distance_to_requested_target_m <= arrival_distance_m, mode must be safe_hold, include hold_position, and do not create_navigation_subgoal.
3. Low battery: if battery_percent < low_battery_percent and the task is not charging_point or emergency, mode must be human_confirm with request_human_confirm, or navigate only to charging_point. If the user explicitly asks to return to charging_point, use mapped_navigation to charging_point with wait_until.
4. Weak bandwidth is detected from world_state.link_quality, not from user words. If bandwidth_kbps < weak_bandwidth_kbps, even when the user does not mention network status, keep mapped_navigation when localization/SLAM/map are valid, but first step must be set_communication_policy. Use communication_policy.mode semantic_only, send exactly task_state, navigation_feedback, world_state_summary, and drop exactly raw_video, dense_pointcloud, high_rate_images.
5. Normal mapped navigation: only after the checks above pass, use create_navigation_subgoal followed by wait_until. Never omit wait_until.

- If mapped_navigation is safe, target_node must be one of the semantic topology node_id values.
- For create_navigation_subgoal, arguments must include "map_id" and "target_node"; never use "target_node_id".
- For mapped_navigation, include a separate wait_until step after create_navigation_subgoal.
- Minimum valid mapped_navigation steps are:
  normal link: create_navigation_subgoal -> wait_until.
  weak bandwidth: set_communication_policy -> create_navigation_subgoal -> wait_until.
- A mapped_navigation answer without wait_until is invalid, even when the target_node is correct.
- If localization is valid, SLAM is ok, and a registered map/topology is available, weak bandwidth does not change the navigation mode. Use mapped_navigation, not mapless_scout.
- In weak bandwidth, when link_quality.bandwidth_kbps < safety_limits.weak_bandwidth_kbps, the first step must be set_communication_policy even if the user command is a normal navigation command. Also set top-level communication_policy.mode to semantic_only, keep navigation_feedback and world_state_summary in send, and drop raw_video, dense_pointcloud, and high_rate_images.
- If a target is unknown or target_node is not one of semantic topology node_id values, use human_confirm and request_human_confirm; do not create_navigation_subgoal.
- If a blocked door or blocked edge is reported, use human_confirm with hold_position before request_human_confirm.
- If critical human/crowd risk appears in a corridor or path, use human_confirm with hold_position and request_human_confirm; do not use mapless_scout.
- If battery_percent is below low_battery_percent and the user does not ask for charging_point or an emergency task, apply this before target navigation: use human_confirm and ask whether to return to charging_point, or navigate only to charging_point.
- If distance_to_requested_target_m is within arrival_distance_m, use safe_hold with hold_position and do not create a navigation subgoal.
- If the target node has photo_required or the command asks to拍照/capture/photo/keyframe, include capture_keyframe after wait_until.
- If a door_may_close target has a door_closed_blocking_goal event or blocked edge, do not navigate; hold and ask for human confirmation.

Planner context:
{planner_context}

Final JSON:"""


LIGHTWEIGHT_SYSTEM_PROMPT = (
    "You are a fast local planner for a Unitree GO2W robot. "
    "Output exactly one compact JSON object. No markdown. No explanation outside JSON. "
    "Use only known target_node ids from candidates. Never output raw Unitree API ids or cmd_vel. "
    "If unsafe or unclear, use human_confirm or safe_hold."
)


LIGHTWEIGHT_PROMPT_TEMPLATE = """Return one compact JSON object with exactly these top-level keys:
plan_id, mode, confidence, reason, steps, communication_policy, requires_human_ack.

Allowed modes: mapped_navigation, safe_hold, human_confirm, mapless_scout.
Allowed tools: set_communication_policy, create_navigation_subgoal, wait_until, capture_keyframe, request_human_confirm, hold_position.
Each step must have exactly: step_id, tool, arguments.
Use short strings. Top-level reason must be under 12 words.
Do not put reason, distance, pose, speed, or photo_required inside step arguments.
create_navigation_subgoal arguments must be exactly {{"map_id": "...", "target_node": "..."}}.
wait_until arguments should be exactly {{"condition":"arrived"}}.
capture_keyframe arguments should be exactly {{"target_node":"..."}}.
communication_policy must include exactly mode, send, drop, reason.
Normal communication_policy is {{"mode":"normal","send":["task_state","navigation_feedback","world_state_summary"],"drop":[],"reason":"normal link"}}.

Priority rules:
1. If slam_ok is false or localized is false: safe_hold with hold_position.
2. If target_node is unknown or not in candidates: human_confirm with request_human_confirm. Do not navigate.
3. If distance_to_requested_target_m <= arrival_distance_m: safe_hold with hold_position. Do not navigate.
4. If low_battery is true and target is not charging_point: human_confirm with request_human_confirm. Do not navigate.
5. If weak_bandwidth is true: first step set_communication_policy; communication_policy.mode semantic_only; drop raw_video, dense_pointcloud, high_rate_images.
6. For valid navigation: create_navigation_subgoal then wait_until. If photo_required is true, include capture_keyframe after wait_until.

Case:
{planner_context}

JSON:"""


INTENT_SYSTEM_PROMPT = (
    "You map a user command to one robot intent JSON. "
    "Output only JSON. Use only candidate node ids. If unclear or unsafe, do not navigate."
)


INTENT_PROMPT_TEMPLATE = """Return exactly one compact JSON object:
{{"mode":"mapped_navigation","target_node":"node_id_or_empty","confidence":0.0,"reason":"short","requires_human_ack":false}}

Rules:
- mode must be exactly one of: mapped_navigation, safe_hold, human_confirm.
- If slam_ok=false or localized=false: mode safe_hold, target_node empty, requires_human_ack true.
- If command target is not in candidates: mode human_confirm, target_node empty, requires_human_ack true.
- If low_battery=true and target_node is not charging_point: mode human_confirm.
- Otherwise choose one candidate target_node and mode mapped_navigation.

Case:
{planner_context}

JSON:"""


class LlmBackend(Protocol):
    def generate(self, prompt: str, *, system_prompt: str, max_tokens: int, timeout_s: int) -> str:
        ...


@dataclass(frozen=True)
class PlannerRunResult:
    plan: dict[str, Any]
    raw_answer: str
    elapsed_s: float


class LocalCommandBackend:
    def __init__(self, command_template: str) -> None:
        self.command_template = command_template

    def generate(self, prompt: str, *, system_prompt: str, max_tokens: int, timeout_s: int) -> str:
        command = self.command_template.format(
            prompt=shlex.quote(prompt),
            system=shlex.quote(system_prompt),
            max_tokens=int(max_tokens),
        )
        completed = subprocess.run(command, shell=True, text=True, capture_output=True, timeout=timeout_s)
        if completed.returncode != 0:
            raise RuntimeError(completed.stderr.strip() or f"local LLM command failed with exit {completed.returncode}")
        return parse_ask_qwen_answer(completed.stdout)


class SshAskQwenBackend:
    def __init__(self, *, host: str, username: str, password: str, ask_script: str, model_path: str) -> None:
        self.host = host
        self.username = username
        self.password = password
        self.ask_script = ask_script
        self.model_path = model_path

    def generate(self, prompt: str, *, system_prompt: str, max_tokens: int, timeout_s: int) -> str:
        try:
            import paramiko
        except ImportError as exc:
            raise RuntimeError("paramiko is required for --backend ssh") from exc

        client = paramiko.SSHClient()
        client.set_missing_host_key_policy(paramiko.AutoAddPolicy())
        client.connect(
            hostname=self.host,
            username=self.username,
            password=self.password,
            timeout=15,
            banner_timeout=15,
            auth_timeout=15,
        )
        try:
            command = (
                f"MODEL_PATH={shlex.quote(self.model_path)} {shlex.quote(self.ask_script)} "
                f"--max-tokens {int(max_tokens)} "
                f"--system {shlex.quote(system_prompt)} "
                f"{shlex.quote(prompt)}"
            )
            stdin, stdout, stderr = client.exec_command(command, timeout=timeout_s)
            out = stdout.read().decode(errors="replace")
            err = stderr.read().decode(errors="replace")
            exit_code = stdout.channel.recv_exit_status()
            if exit_code != 0:
                raise RuntimeError(err.strip() or f"remote LLM command failed with exit {exit_code}")
            return parse_ask_qwen_answer(out)
        finally:
            client.close()


def build_planner_prompt(planner_context: dict[str, Any]) -> str:
    return PROMPT_TEMPLATE.format(planner_context=json.dumps(planner_context, ensure_ascii=False, indent=2))


def build_lightweight_planner_context(planner_context: dict[str, Any]) -> dict[str, Any]:
    summary = planner_context.get("world_state_summary", {})
    robot = summary.get("robot", {}) if isinstance(summary, dict) else {}
    slam = summary.get("slam", {}) if isinstance(summary, dict) else {}
    topology = summary.get("topology", {}) if isinstance(summary, dict) else {}
    nodes = topology.get("available_nodes", []) if isinstance(topology, dict) else []
    user_command = str(planner_context.get("user_command", ""))

    candidates = []
    requested_node = None
    command_lower = user_command.lower()
    if isinstance(nodes, list):
        for node in nodes:
            if not isinstance(node, dict):
                continue
            node_id = str(node.get("node_id", ""))
            aliases = [str(v) for v in node.get("aliases", []) if v]
            name = str(node.get("name", node_id))
            tags = [str(v) for v in node.get("tags", []) if v]
            candidates.append(
                {
                    "node_id": node_id,
                    "name": name,
                    "aliases": aliases[:5],
                    "tags": tags,
                    "distance_m": node.get("distance_from_robot_m"),
                    "photo_required": "photo_required" in tags,
                    "door_may_close": "door_may_close" in tags,
                }
            )
            match_terms = [node_id, name, *aliases]
            if requested_node is None and any(term and term.lower() in command_lower for term in match_terms):
                requested_node = node_id

    link_quality = _dig_value(planner_context, "link_quality") or {}
    bandwidth = _dig_value(planner_context, "bandwidth_kbps")
    weak_bandwidth = _dig_value(planner_context, "weak_bandwidth_kbps")
    battery = _dig_value(planner_context, "battery_percent")
    low_battery_limit = _dig_value(planner_context, "low_battery_percent")
    distance_to_target = _dig_value(planner_context, "distance_to_requested_target_m")
    arrival_distance = _dig_value(planner_context, "arrival_distance_m")
    if requested_node and distance_to_target is None:
        for candidate in candidates:
            if candidate["node_id"] == requested_node:
                distance_to_target = candidate.get("distance_m")
                break

    return {
        "user_command": user_command,
        "map_id": (summary.get("map", {}) if isinstance(summary, dict) else {}).get("map_id"),
        "slam_ok": slam.get("health_status") == "ok",
        "localized": bool(robot.get("localized")),
        "nearest_node": (robot.get("nearest_node") or {}).get("node_id") if isinstance(robot.get("nearest_node"), dict) else None,
        "requested_target_guess": requested_node,
        "distance_to_requested_target_m": distance_to_target,
        "arrival_distance_m": arrival_distance if arrival_distance is not None else 0.3,
        "battery_percent": battery,
        "low_battery_percent": low_battery_limit,
        "low_battery": isinstance(battery, (int, float)) and isinstance(low_battery_limit, (int, float)) and float(battery) < float(low_battery_limit),
        "bandwidth_kbps": bandwidth if bandwidth is not None else (link_quality.get("bandwidth_kbps") if isinstance(link_quality, dict) else None),
        "weak_bandwidth_kbps": weak_bandwidth,
        "weak_bandwidth": isinstance(bandwidth, (int, float)) and isinstance(weak_bandwidth, (int, float)) and float(bandwidth) < float(weak_bandwidth),
        "candidates": candidates,
    }


def build_lightweight_planner_prompt(planner_context: dict[str, Any]) -> str:
    light_context = build_lightweight_planner_context(planner_context)
    return LIGHTWEIGHT_PROMPT_TEMPLATE.format(planner_context=json.dumps(light_context, ensure_ascii=False, separators=(",", ":")))


def build_intent_planner_prompt(planner_context: dict[str, Any]) -> str:
    light_context = build_lightweight_planner_context(planner_context)
    return INTENT_PROMPT_TEMPLATE.format(planner_context=json.dumps(light_context, ensure_ascii=False, separators=(",", ":")))


def intent_to_local_plan(intent: dict[str, Any], planner_context: dict[str, Any]) -> dict[str, Any]:
    mode = str(intent.get("mode", "human_confirm"))
    if "|" in mode:
        if "mapped_navigation" in mode:
            mode = "mapped_navigation"
        elif "safe_hold" in mode:
            mode = "safe_hold"
        else:
            mode = "human_confirm"
    target_node = str(intent.get("target_node") or "")
    requires_ack = bool(intent.get("requires_human_ack", mode == "human_confirm"))
    try:
        confidence = float(intent.get("confidence", 0.5))
    except (TypeError, ValueError):
        confidence = 0.5
    confidence = max(0.0, min(1.0, confidence))
    reason = str(intent.get("reason") or "intent planner result")[:160]
    map_id = _dig_value(planner_context, "map_id") or "unknown"
    communication_policy = _normal_communication_policy()

    if mode == "mapped_navigation" and target_node:
        steps = [
            {"step_id": "nav_1", "tool": "create_navigation_subgoal", "arguments": {"map_id": map_id, "target_node": target_node}},
            {"step_id": "wait_1", "tool": "wait_until", "arguments": {"condition": "arrived"}},
        ]
    elif mode == "safe_hold":
        steps = [{"step_id": "hold_1", "tool": "hold_position", "arguments": {"reason": reason}}]
        target_node = ""
    else:
        mode = "human_confirm"
        steps = [{"step_id": "ask_1", "tool": "request_human_confirm", "arguments": {"reason": reason, "target_node": target_node}}]
        requires_ack = True

    return {
        "plan_id": f"intent_plan_{int(time.time() * 1000)}",
        "mode": mode,
        "confidence": confidence,
        "reason": reason,
        "steps": steps,
        "communication_policy": communication_policy,
        "requires_human_ack": requires_ack,
    }


def deterministic_intent_from_context(planner_context: dict[str, Any]) -> dict[str, Any] | None:
    light_context = build_lightweight_planner_context(planner_context)
    if not light_context.get("slam_ok") or not light_context.get("localized"):
        return {
            "mode": "safe_hold",
            "target_node": "",
            "confidence": 1.0,
            "reason": "slam or localization not ready",
            "requires_human_ack": True,
        }

    target_node = str(light_context.get("requested_target_guess") or "")
    known_nodes = {str(candidate.get("node_id")) for candidate in light_context.get("candidates", []) if isinstance(candidate, dict)}
    if not target_node or target_node not in known_nodes:
        return None

    arrival_distance = light_context.get("arrival_distance_m")
    distance_to_target = light_context.get("distance_to_requested_target_m")
    if isinstance(arrival_distance, (int, float)) and isinstance(distance_to_target, (int, float)) and float(distance_to_target) <= float(arrival_distance):
        return {
            "mode": "safe_hold",
            "target_node": target_node,
            "confidence": 1.0,
            "reason": "already near target",
            "requires_human_ack": False,
        }

    if light_context.get("low_battery") and target_node != "charging_point":
        return {
            "mode": "human_confirm",
            "target_node": target_node,
            "confidence": 1.0,
            "reason": "low battery requires confirmation",
            "requires_human_ack": True,
        }

    return {
        "mode": "mapped_navigation",
        "target_node": target_node,
        "confidence": 1.0,
        "reason": "deterministic target match",
        "requires_human_ack": False,
    }


def parse_ask_qwen_answer(output: str) -> str:
    match = re.search(r"(?s)\bAnswer:\n(.*?)(?:\n\nStats:\n|$)", output)
    if match:
        return match.group(1).strip()
    return output.strip()


def extract_json_object(text: str) -> dict[str, Any]:
    cleaned = text.strip()
    if cleaned.startswith("```"):
        cleaned = re.sub(r"^```(?:json)?\s*", "", cleaned)
        cleaned = re.sub(r"\s*```$", "", cleaned)

    decoder = json.JSONDecoder()
    candidates: list[dict[str, Any]] = []
    for index, char in enumerate(cleaned):
        if char != "{":
            continue
        try:
            value, _ = decoder.raw_decode(cleaned[index:])
        except json.JSONDecodeError:
            continue
        if isinstance(value, dict):
            candidates.append(value)
    for candidate in candidates:
        if PLAN_REQUIRED_KEYS.issubset(candidate.keys()):
            return candidate
    if candidates:
        return candidates[-1]
    raise ValueError("no JSON object found in model output")


def require(condition: bool, message: str) -> None:
    if not condition:
        raise ValueError(message)


def validate_local_llm_plan(plan: dict[str, Any]) -> None:
    require(set(plan.keys()) == PLAN_REQUIRED_KEYS, f"plan keys mismatch: {sorted(plan.keys())}")
    require(isinstance(plan["plan_id"], str) and plan["plan_id"], "plan_id must be a non-empty string")
    require(plan["mode"] in PLAN_MODES, f"invalid mode: {plan['mode']}")
    require(isinstance(plan["confidence"], (int, float)) and 0 <= plan["confidence"] <= 1, "confidence must be 0..1")
    require(isinstance(plan["reason"], str) and plan["reason"], "reason must be a non-empty string")
    require(isinstance(plan["requires_human_ack"], bool), "requires_human_ack must be boolean")
    require(isinstance(plan["steps"], list) and 1 <= len(plan["steps"]) <= 6, "steps length must be 1..6")
    for step in plan["steps"]:
        require(isinstance(step, dict), "step must be object")
        require(set(step.keys()) == {"step_id", "tool", "arguments"}, f"step keys mismatch: {step}")
        require(isinstance(step["step_id"], str) and step["step_id"], "step_id must be a non-empty string")
        require(step["tool"] in PLAN_TOOLS, f"invalid tool: {step['tool']}")
        require(isinstance(step["arguments"], dict), "step arguments must be object")

    comm = plan["communication_policy"]
    require(isinstance(comm, dict), "communication_policy must be object")
    require({"mode", "send", "drop"}.issubset(comm.keys()), "communication_policy missing required keys")
    require(comm["mode"] in COMMUNICATION_MODES, f"invalid communication mode: {comm['mode']}")
    require(isinstance(comm["send"], list), "communication send must be list")
    require(isinstance(comm["drop"], list), "communication drop must be list")
    for item in comm["send"]:
        require(item in SEND_ITEMS, f"invalid send item: {item}")
    for item in comm["drop"]:
        require(item in DROP_ITEMS, f"invalid drop item: {item}")


def validate_execution_contract(plan: dict[str, Any]) -> None:
    if plan.get("mode") != "mapped_navigation":
        return
    nav_steps = [step for step in plan.get("steps", []) if step.get("tool") == "create_navigation_subgoal"]
    wait_steps = [step for step in plan.get("steps", []) if step.get("tool") == "wait_until"]
    require(nav_steps, "mapped_navigation must include create_navigation_subgoal")
    require(wait_steps, "mapped_navigation must include wait_until")
    nav_args = nav_steps[0].get("arguments", {})
    require("map_id" in nav_args, "create_navigation_subgoal arguments must include map_id")
    require("target_node" in nav_args, "create_navigation_subgoal arguments must include target_node")
    require("target_node_id" not in nav_args, "use target_node instead of target_node_id")


def _collect_tools(plan: dict[str, Any]) -> list[str]:
    return [str(step.get("tool")) for step in plan.get("steps", []) if isinstance(step, dict)]


def _nav_target_nodes(plan: dict[str, Any]) -> list[str]:
    targets: list[str] = []
    for step in plan.get("steps", []):
        if not isinstance(step, dict) or step.get("tool") != "create_navigation_subgoal":
            continue
        args = step.get("arguments", {})
        if isinstance(args, dict) and isinstance(args.get("target_node"), str):
            targets.append(args["target_node"])
    return targets


def _dig_value(value: Any, key: str) -> Any:
    if isinstance(value, dict):
        if key in value:
            return value[key]
        for item in value.values():
            found = _dig_value(item, key)
            if found is not None:
                return found
    elif isinstance(value, list):
        for item in value:
            found = _dig_value(item, key)
            if found is not None:
                return found
    return None


def _semantic_nodes(planner_context: dict[str, Any]) -> set[str]:
    topology = planner_context.get("semantic_topology") or planner_context.get("topology") or planner_context
    nodes = []
    if isinstance(topology, dict):
        nodes = topology.get("nodes") or topology.get("available_nodes") or []
    if not nodes:
        found_nodes = _dig_value(planner_context, "available_nodes")
        nodes = found_nodes if isinstance(found_nodes, list) else []
    out: set[str] = set()
    if isinstance(nodes, list):
        for node in nodes:
            if isinstance(node, dict) and isinstance(node.get("node_id"), str):
                out.add(node["node_id"])
    return out


def _semantic_node_by_id(planner_context: dict[str, Any], node_id: str) -> dict[str, Any] | None:
    found_nodes = _dig_value(planner_context, "available_nodes")
    nodes = found_nodes if isinstance(found_nodes, list) else []
    if not nodes:
        topology = planner_context.get("semantic_topology") or planner_context.get("topology") or {}
        nodes = topology.get("nodes", []) if isinstance(topology, dict) else []
    for node in nodes:
        if isinstance(node, dict) and node.get("node_id") == node_id:
            return node
    return None


def _node_requires_photo(planner_context: dict[str, Any], node_id: str) -> bool:
    node = _semantic_node_by_id(planner_context, node_id)
    if not node:
        return False
    tags = node.get("tags", [])
    return isinstance(tags, list) and "photo_required" in tags


def _weak_communication_policy() -> dict[str, Any]:
    return {
        "mode": "semantic_only",
        "send": ["task_state", "navigation_feedback", "world_state_summary"],
        "drop": ["raw_video", "dense_pointcloud", "high_rate_images"],
        "reason": "weak bandwidth detected from local link state",
    }


def _normal_communication_policy() -> dict[str, Any]:
    return {
        "mode": "normal",
        "send": ["task_state", "navigation_feedback", "world_state_summary"],
        "drop": [],
        "reason": "normal link",
    }


def apply_context_policy_overrides(plan: dict[str, Any], planner_context: dict[str, Any]) -> dict[str, Any]:
    """Apply deterministic local safety/communication rules to a schema-valid plan."""
    fixed = deepcopy(plan)
    steps = fixed.get("steps", [])
    if not isinstance(steps, list):
        return fixed

    confidence = fixed.get("confidence")
    if not isinstance(confidence, (int, float)):
        try:
            fixed["confidence"] = float(str(confidence).strip().rstrip("%")) / (100.0 if "%" in str(confidence) else 1.0)
        except (TypeError, ValueError):
            fixed["confidence"] = 0.5
    if isinstance(fixed.get("confidence"), (int, float)):
        fixed["confidence"] = max(0.0, min(1.0, float(fixed["confidence"])))

    comm = fixed.get("communication_policy")
    if not isinstance(comm, dict):
        fixed["communication_policy"] = _normal_communication_policy()
    else:
        normalized_comm = _normal_communication_policy()
        normalized_comm.update({k: v for k, v in comm.items() if k in {"mode", "send", "drop", "reason"}})
        if not isinstance(normalized_comm.get("send"), list):
            normalized_comm["send"] = ["task_state", "navigation_feedback", "world_state_summary"]
        if not isinstance(normalized_comm.get("drop"), list):
            normalized_comm["drop"] = []
        if not normalized_comm.get("reason"):
            normalized_comm["reason"] = "normal link"
        fixed["communication_policy"] = normalized_comm

    bandwidth = _dig_value(planner_context, "bandwidth_kbps")
    weak_bandwidth = _dig_value(planner_context, "weak_bandwidth_kbps")
    if isinstance(bandwidth, (int, float)) and isinstance(weak_bandwidth, (int, float)) and float(bandwidth) < float(weak_bandwidth):
        weak_comm = _weak_communication_policy()
        fixed["communication_policy"] = weak_comm
        if not steps or steps[0].get("tool") != "set_communication_policy":
            steps.insert(0, {"step_id": "comm_1", "tool": "set_communication_policy", "arguments": weak_comm})
        else:
            steps[0]["arguments"] = weak_comm

    if fixed.get("mode") == "mapped_navigation":
        nav_index = next((i for i, step in enumerate(steps) if isinstance(step, dict) and step.get("tool") == "create_navigation_subgoal"), None)
        if nav_index is not None:
            nav_args = steps[nav_index].setdefault("arguments", {})
            if isinstance(nav_args, dict):
                if not nav_args.get("map_id"):
                    map_id = _dig_value(planner_context, "map_id")
                    if map_id:
                        nav_args["map_id"] = map_id
        has_wait = any(isinstance(step, dict) and step.get("tool") == "wait_until" for step in steps)
        if nav_index is not None and not has_wait:
            steps.insert(
                nav_index + 1,
                {
                    "step_id": "wait_1",
                    "tool": "wait_until",
                    "arguments": {"source": "/slam_info", "condition": "ctrl_info_arrived_or_finished"},
                },
            )
        nav_targets = _nav_target_nodes(fixed)
        if nav_targets and _node_requires_photo(planner_context, nav_targets[0]):
            has_capture = any(isinstance(step, dict) and step.get("tool") == "capture_keyframe" for step in steps)
            if not has_capture:
                wait_index = next((i for i, step in enumerate(steps) if isinstance(step, dict) and step.get("tool") == "wait_until"), nav_index)
                insert_at = (wait_index + 1) if wait_index is not None else len(steps)
                steps.insert(
                    insert_at,
                    {
                        "step_id": "capture_1",
                        "tool": "capture_keyframe",
                        "arguments": {"target_node": nav_targets[0], "reason": "photo_required target"},
                    },
                )

    fixed["steps"] = steps[:6]
    return fixed


def validate_context_policy(plan: dict[str, Any], planner_context: dict[str, Any]) -> None:
    tools = _collect_tools(plan)
    nav_targets = _nav_target_nodes(plan)
    known_nodes = _semantic_nodes(planner_context)
    if known_nodes:
        for target in nav_targets:
            require(target in known_nodes, f"target_node {target!r} is not in semantic topology")

    arrival_distance = _dig_value(planner_context, "arrival_distance_m")
    distance_to_target = _dig_value(planner_context, "distance_to_requested_target_m")
    if isinstance(arrival_distance, (int, float)) and isinstance(distance_to_target, (int, float)):
        if float(distance_to_target) <= float(arrival_distance):
            require("create_navigation_subgoal" not in tools, "already near target: do not create navigation subgoal")
            require("hold_position" in tools, "already near target: must hold_position")

    battery_percent = _dig_value(planner_context, "battery_percent")
    low_battery = _dig_value(planner_context, "low_battery_percent")
    user_command = str(planner_context.get("user_command", ""))
    emergency = any(word in user_command.lower() for word in ("emergency", "urgent")) or any(word in user_command for word in ("紧急", "急救", "危险"))
    charging_task = "charging_point" in nav_targets or "充电" in user_command or "回充" in user_command
    if isinstance(battery_percent, (int, float)) and isinstance(low_battery, (int, float)):
        if float(battery_percent) < float(low_battery) and not emergency and not charging_task:
            require(plan.get("mode") == "human_confirm", "low battery non-emergency task must use human_confirm")
            require("request_human_confirm" in tools, "low battery non-emergency task must request human confirmation")
            require("create_navigation_subgoal" not in tools, "low battery non-emergency task must not navigate directly")

    bandwidth = _dig_value(planner_context, "bandwidth_kbps")
    weak_bandwidth = _dig_value(planner_context, "weak_bandwidth_kbps")
    if isinstance(bandwidth, (int, float)) and isinstance(weak_bandwidth, (int, float)) and float(bandwidth) < float(weak_bandwidth):
        require(tools and tools[0] == "set_communication_policy", "weak bandwidth: first step must set communication policy")
        comm = plan.get("communication_policy", {})
        require(isinstance(comm, dict) and comm.get("mode") == "semantic_only", "weak bandwidth: communication mode must be semantic_only")
        drops = set(comm.get("drop", [])) if isinstance(comm.get("drop"), list) else set()
        for item in ("raw_video", "dense_pointcloud", "high_rate_images"):
            require(item in drops, f"weak bandwidth: missing drop item {item}")

    for target in nav_targets:
        if _node_requires_photo(planner_context, target):
            require("capture_keyframe" in tools, f"photo_required target {target!r} must capture_keyframe after arrival")


def run_local_llm_planner(
    planner_context: dict[str, Any],
    backend: LlmBackend,
    *,
    system_prompt: str = DEFAULT_SYSTEM_PROMPT,
    max_tokens: int = 768,
    timeout_s: int = 240,
    prompt_mode: str = "full",
) -> PlannerRunResult:
    if prompt_mode == "hybrid":
        intent = deterministic_intent_from_context(planner_context)
        if intent is not None:
            plan = intent_to_local_plan(intent, planner_context)
            plan = apply_context_policy_overrides(plan, planner_context)
            validate_local_llm_plan(plan)
            validate_execution_contract(plan)
            validate_context_policy(plan, planner_context)
            return PlannerRunResult(plan=plan, raw_answer=json.dumps(intent, ensure_ascii=False), elapsed_s=0.0)
        prompt = build_intent_planner_prompt(planner_context)
        if system_prompt == DEFAULT_SYSTEM_PROMPT:
            system_prompt = INTENT_SYSTEM_PROMPT
    elif prompt_mode == "intent":
        prompt = build_intent_planner_prompt(planner_context)
        if system_prompt == DEFAULT_SYSTEM_PROMPT:
            system_prompt = INTENT_SYSTEM_PROMPT
    elif prompt_mode == "light":
        prompt = build_lightweight_planner_prompt(planner_context)
        if system_prompt == DEFAULT_SYSTEM_PROMPT:
            system_prompt = LIGHTWEIGHT_SYSTEM_PROMPT
    else:
        prompt = build_planner_prompt(planner_context)
    start = time.time()
    raw_answer = backend.generate(prompt, system_prompt=system_prompt, max_tokens=max_tokens, timeout_s=timeout_s)
    elapsed_s = time.time() - start
    if prompt_mode in {"intent", "hybrid"}:
        plan = intent_to_local_plan(extract_json_object(raw_answer), planner_context)
    else:
        plan = extract_json_object(raw_answer)
    plan = apply_context_policy_overrides(plan, planner_context)
    validate_local_llm_plan(plan)
    validate_execution_contract(plan)
    validate_context_policy(plan, planner_context)
    return PlannerRunResult(plan=plan, raw_answer=raw_answer, elapsed_s=elapsed_s)
