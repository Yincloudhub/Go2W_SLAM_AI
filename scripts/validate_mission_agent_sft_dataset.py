from __future__ import annotations

import json
import sys
from pathlib import Path
from typing import Any, Dict, Iterable, List


ROOT = Path(__file__).resolve().parents[1]
DEFAULT_SFT_JSON = ROOT / "data" / "local_llm_sft" / "go2w_mission_agent_sft.json"
DEFAULT_EVAL_JSONL = ROOT / "data" / "local_llm_eval" / "go2w_mission_agent_eval.jsonl"
MISSION_SCHEMA = "go2w_mission_plan_v1"


def load_json_records(path: Path) -> List[Dict[str, Any]]:
    if path.suffix == ".jsonl":
        return [json.loads(line) for line in path.read_text(encoding="utf-8").splitlines() if line.strip()]
    data = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(data, list):
        raise ValueError("%s must contain a JSON array" % path)
    return data


def require(condition: bool, message: str) -> None:
    if not condition:
        raise ValueError(message)


def check_no_mojibake(text: str, label: str) -> None:
    blocked = ["\ufffd", "\u951f", "\u95bf", "\u53e8"]
    for token in blocked:
        require(token not in text, "%s contains suspicious mojibake token %r" % (label, token))


def validate_step(step: Dict[str, Any]) -> None:
    require(step.get("execution_enabled") is False, "step execution_enabled must be false")
    require(step.get("type") in {"chat", "motion", "behavior", "map_transition"}, "invalid step type: %r" % step.get("type"))
    if step.get("type") == "motion":
        require(step.get("executor") == "nav_core", "motion executor must be nav_core")
        validation = step.get("validation")
        require(isinstance(validation, dict), "motion step missing validation")
        require("status" in step, "motion step missing status")
        accepted = validation.get("accepted")
        status = str(step.get("status") or "")
        require(not accepted or status == "validated", "accepted motion step must be validated")
        if status != "validated":
            require(accepted is False, "non-validated motion step must not be accepted")
        route = step.get("route") if isinstance(step.get("route"), list) else []
        for node_item in route:
            require(isinstance(node_item, dict), "motion route item must be object")
            require(
                node_item.get("map_id") == step.get("from_map") and step.get("from_map") == step.get("to_map"),
                "motion route map must match the step map",
            )
            if status == "validated":
                require(node_item.get("pose_valid") is not False, "validated route must not contain invalid-pose nodes")
        # Route endpoint consistency: validated route endpoints must match step from/to
        if route and status == "validated":
            from_node = str(step.get("from_node") or "")
            to_node = str(step.get("to_node") or "")
            require(
                route[0].get("node_id") == from_node,
                "validated route first node %r != from_node %r" % (route[0].get("node_id"), from_node),
            )
            require(
                route[-1].get("node_id") == to_node,
                "validated route last node %r != to_node %r" % (route[-1].get("node_id"), to_node),
            )
    if step.get("type") == "behavior":
        require(step.get("behavior") in {"capture_photo", "speak", "report", "wait"}, "invalid behavior")
    raw = json.dumps(step, ensure_ascii=False)
    for blocked in ("cmd_vel", "Unitree API", "shell_command", "slam_operate"):
        require(blocked not in raw, "blocked low-level token in step: %s" % blocked)


def validate_plan(plan: Dict[str, Any]) -> None:
    required = {
        "schema",
        "schema_version",
        "plan_id",
        "intent_kind",
        "map_id",
        "matched_nodes",
        "steps",
        "communication_policy",
        "execution_enabled",
        "motion_authority",
        "llm_direct_motion",
    }
    require(set(plan.keys()) == required, "plan keys mismatch: %s" % sorted(plan.keys()))
    require(plan["schema"] == MISSION_SCHEMA, "schema mismatch")
    require(plan["schema_version"] == 1, "schema_version must be 1")
    require(plan["intent_kind"] in {"chat", "motion", "behavior", "mixed"}, "invalid intent_kind")
    require(plan["execution_enabled"] is False, "plan execution_enabled must be false")
    require(plan["llm_direct_motion"] is False, "llm_direct_motion must be false")
    require(plan["motion_authority"] == "robot_side_nav_core_and_safety_gate", "unexpected motion authority")
    require(isinstance(plan["matched_nodes"], list), "matched_nodes must be list")
    require(isinstance(plan["steps"], list) and plan["steps"], "steps must be non-empty list")
    comm = plan["communication_policy"]
    require(isinstance(comm, dict), "communication_policy must be object")
    require(comm.get("mode") in {"normal", "semantic_only", "keyframe_low_rate", "hold_remote"}, "invalid communication mode")
    require(isinstance(comm.get("send"), list), "communication send must be list")
    require(isinstance(comm.get("drop"), list), "communication drop must be list")
    for step in plan["steps"]:
        require(isinstance(step, dict), "step must be object")
        validate_step(step)


def validate_sft_record(record: Dict[str, Any], index: int) -> None:
    require("case_id" in record, "record %d missing case_id" % index)
    require(record.get("system"), "%s missing system" % record.get("case_id"))
    require("tools" in record, "%s missing tools" % record.get("case_id"))
    check_no_mojibake(json.dumps(record, ensure_ascii=False), record["case_id"])
    conversations = record.get("conversations")
    require(isinstance(conversations, list) and len(conversations) == 2, "%s conversations length must be 2" % record["case_id"])
    require(conversations[0].get("from") == "user", "%s first message must be user" % record["case_id"])
    require(conversations[1].get("from") == "assistant", "%s second message must be assistant" % record["case_id"])
    output = json.loads(conversations[1]["value"])
    require(set(output.keys()) == {"reply", "mission_plan"}, "%s assistant output keys mismatch" % record["case_id"])
    require(isinstance(output["reply"], str) and output["reply"], "%s reply must be non-empty" % record["case_id"])
    validate_plan(output["mission_plan"])


def validate_eval_record(record: Dict[str, Any], index: int) -> None:
    require("case_id" in record and "category" in record, "eval record %d missing id/category" % index)
    check_no_mojibake(json.dumps(record, ensure_ascii=False), record["case_id"])
    require(isinstance(record.get("expected_mission_plan"), dict), "%s expected_mission_plan missing" % record["case_id"])
    validate_plan(record["expected_mission_plan"])
    policy = record.get("expected_policy")
    require(isinstance(policy, dict), "%s expected_policy missing" % record["case_id"])
    require(policy.get("execution_enabled") is False, "%s expected execution flag must be false" % record["case_id"])
    require(policy.get("llm_direct_motion") is False, "%s expected llm_direct_motion must be false" % record["case_id"])


def count_categories(records: Iterable[Dict[str, Any]]) -> Dict[str, int]:
    counts: Dict[str, int] = {}
    for item in records:
        key = str(item.get("category") or "")
        counts[key] = counts.get(key, 0) + 1
    return dict(sorted(counts.items()))


def main() -> int:
    sft_path = Path(sys.argv[1]) if len(sys.argv) > 1 else DEFAULT_SFT_JSON
    eval_path = Path(sys.argv[2]) if len(sys.argv) > 2 else DEFAULT_EVAL_JSONL
    sft_records = load_json_records(sft_path)
    eval_records = load_json_records(eval_path)
    seen = set()
    for index, item in enumerate(sft_records, 1):
        validate_sft_record(item, index)
        case_id = item["case_id"]
        require(case_id not in seen, "duplicate case_id: %s" % case_id)
        seen.add(case_id)
    for index, item in enumerate(eval_records, 1):
        validate_eval_record(item, index)
    print("MissionAgent SFT OK: %d records -> %s" % (len(sft_records), sft_path))
    print("MissionAgent EVAL OK: %d records -> %s" % (len(eval_records), eval_path))
    print("Categories: %s" % json.dumps(count_categories(sft_records), ensure_ascii=False, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
