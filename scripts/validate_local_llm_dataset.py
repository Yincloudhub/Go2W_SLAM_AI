from __future__ import annotations

import json
import sys
from pathlib import Path
from typing import Any


ROOT = Path(__file__).resolve().parents[1]
SCHEMA_PATH = ROOT / "schemas" / "local_llm_plan.schema.json"
DEFAULT_SFT_JSON = ROOT / "data" / "local_llm_sft" / "go2w_competition_planner_sft.json"
DEFAULT_EVAL_JSONL = ROOT / "data" / "local_llm_eval" / "go2w_competition_eval.jsonl"


def load_json_records(path: Path) -> list[dict[str, Any]]:
    if path.suffix == ".jsonl":
        return [json.loads(line) for line in path.read_text(encoding="utf-8").splitlines() if line.strip()]
    data = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(data, list):
        raise ValueError(f"{path} must contain a JSON array")
    return data


def require(condition: bool, message: str) -> None:
    if not condition:
        raise ValueError(message)


def validate_plan(plan: dict[str, Any]) -> None:
    modes = {"mapped_navigation", "mapless_scout", "safe_hold", "human_confirm"}
    tools = {
        "set_communication_policy",
        "create_navigation_subgoal",
        "wait_until",
        "capture_keyframe",
        "relative_motion_preview",
        "start_mapless_scout",
        "request_human_confirm",
        "hold_position",
    }
    comm_modes = {"normal", "semantic_only", "keyframe_low_rate", "hold_remote"}
    send_items = {"task_state", "risk_events", "keyframe", "semantic_topology", "navigation_feedback", "world_state_summary"}
    drop_items = {"raw_video", "dense_pointcloud", "full_log", "high_rate_images"}

    required = {"plan_id", "mode", "confidence", "reason", "steps", "communication_policy", "requires_human_ack"}
    require(set(plan.keys()) == required, f"plan keys mismatch: {sorted(plan.keys())}")
    require(isinstance(plan["plan_id"], str) and plan["plan_id"], "plan_id must be a non-empty string")
    require(plan["mode"] in modes, f"invalid mode: {plan['mode']}")
    require(isinstance(plan["confidence"], (int, float)) and 0 <= plan["confidence"] <= 1, "confidence must be 0..1")
    require(isinstance(plan["reason"], str) and plan["reason"], "reason must be a non-empty string")
    require(isinstance(plan["requires_human_ack"], bool), "requires_human_ack must be boolean")
    require(isinstance(plan["steps"], list) and 1 <= len(plan["steps"]) <= 6, "steps length must be 1..6")
    for step in plan["steps"]:
        require(set(step.keys()) == {"step_id", "tool", "arguments"}, f"step keys mismatch: {step}")
        require(isinstance(step["step_id"], str) and step["step_id"], "step_id must be a non-empty string")
        require(step["tool"] in tools, f"invalid tool: {step['tool']}")
        require(isinstance(step["arguments"], dict), "step arguments must be object")

    comm = plan["communication_policy"]
    require(isinstance(comm, dict), "communication_policy must be object")
    require({"mode", "send", "drop"}.issubset(comm.keys()), "communication_policy missing required keys")
    require(plan["communication_policy"]["mode"] in comm_modes, f"invalid communication mode: {comm['mode']}")
    require(isinstance(comm["send"], list), "communication send must be list")
    require(isinstance(comm["drop"], list), "communication drop must be list")
    for item in comm["send"]:
        require(item in send_items, f"invalid send item: {item}")
    for item in comm["drop"]:
        require(item in drop_items, f"invalid drop item: {item}")


def extract_assistant_plan(record: dict[str, Any]) -> dict[str, Any]:
    conversations = record.get("conversations")
    require(isinstance(conversations, list) and len(conversations) >= 2, "record conversations missing")
    assistant = conversations[-1]
    require(assistant.get("from") == "assistant", "last conversation must be assistant")
    value = assistant.get("value")
    require(isinstance(value, str), "assistant value must be string")
    parsed = json.loads(value)
    require(isinstance(parsed, dict), "assistant value must parse to object")
    return parsed


def validate_sft(path: Path) -> int:
    records = load_json_records(path)
    for index, record in enumerate(records, 1):
        try:
            plan = extract_assistant_plan(record)
            validate_plan(plan)
        except Exception as exc:
            raise ValueError(f"{path} record {index} invalid: {exc}") from exc
    return len(records)


def validate_eval(path: Path) -> int:
    records = load_json_records(path)
    for index, record in enumerate(records, 1):
        try:
            require("case_id" in record and "category" in record, "eval record missing case_id/category")
            expected_plan = record.get("expected_plan")
            require(isinstance(expected_plan, dict), "expected_plan must be object")
            validate_plan(expected_plan)
            require(isinstance(record.get("expected_policy"), dict), "expected_policy must be object")
        except Exception as exc:
            raise ValueError(f"{path} record {index} invalid: {exc}") from exc
    return len(records)


def main() -> None:
    sft_path = Path(sys.argv[1]) if len(sys.argv) > 1 else DEFAULT_SFT_JSON
    eval_path = Path(sys.argv[2]) if len(sys.argv) > 2 else DEFAULT_EVAL_JSONL
    sft_count = validate_sft(sft_path)
    eval_count = validate_eval(eval_path)
    print(f"SFT OK: {sft_count} records -> {sft_path}")
    print(f"EVAL OK: {eval_count} records -> {eval_path}")


if __name__ == "__main__":
    main()
