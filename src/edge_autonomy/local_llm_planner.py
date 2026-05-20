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


def apply_context_policy_overrides(plan: dict[str, Any], planner_context: dict[str, Any]) -> dict[str, Any]:
    """Apply deterministic local safety/communication rules to a schema-valid plan."""
    fixed = deepcopy(plan)
    steps = fixed.get("steps", [])
    if not isinstance(steps, list):
        return fixed

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
) -> PlannerRunResult:
    prompt = build_planner_prompt(planner_context)
    start = time.time()
    raw_answer = backend.generate(prompt, system_prompt=system_prompt, max_tokens=max_tokens, timeout_s=timeout_s)
    elapsed_s = time.time() - start
    plan = extract_json_object(raw_answer)
    validate_local_llm_plan(plan)
    plan = apply_context_policy_overrides(plan, planner_context)
    validate_local_llm_plan(plan)
    validate_execution_contract(plan)
    validate_context_policy(plan, planner_context)
    return PlannerRunResult(plan=plan, raw_answer=raw_answer, elapsed_s=elapsed_s)
