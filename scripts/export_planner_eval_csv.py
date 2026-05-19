from __future__ import annotations

import argparse
import csv
import json
import re
from pathlib import Path
from typing import Any


ROOT = Path(__file__).resolve().parents[1]


def load_jsonl(path: Path) -> list[dict[str, Any]]:
    return [json.loads(line) for line in path.read_text(encoding="utf-8").splitlines() if line.strip()]


def compact_json(value: Any) -> str:
    return json.dumps(value, ensure_ascii=False, separators=(",", ":")) if value is not None else ""


def tools(plan: dict[str, Any] | None) -> str:
    if not isinstance(plan, dict):
        return ""
    return "|".join(str(step.get("tool", "")) for step in plan.get("steps", []) if isinstance(step, dict))


def target_nodes(plan: dict[str, Any] | None) -> str:
    if not isinstance(plan, dict):
        return ""
    out: list[str] = []
    for step in plan.get("steps", []):
        if not isinstance(step, dict):
            continue
        args = step.get("arguments", {})
        if isinstance(args, dict):
            for key in ("target_node", "target_node_id"):
                if isinstance(args.get(key), str):
                    out.append(args[key])
    return "|".join(out)


def comm_mode(plan: dict[str, Any] | None) -> str:
    if not isinstance(plan, dict):
        return ""
    comm = plan.get("communication_policy", {})
    return str(comm.get("mode", "")) if isinstance(comm, dict) else str(comm)


def comm_send_drop(plan: dict[str, Any] | None, key: str) -> str:
    if not isinstance(plan, dict):
        return ""
    comm = plan.get("communication_policy", {})
    if not isinstance(comm, dict):
        return ""
    value = comm.get(key, [])
    return "|".join(str(item) for item in value) if isinstance(value, list) else str(value)


def extract_user_command(input_text: str) -> str:
    match = re.search(r"User command:\s*(.*?)(?:\n\n|$)", input_text, flags=re.S)
    return match.group(1).strip() if match else ""


def extract_world_state(input_text: str) -> str:
    match = re.search(r"World state:\s*(.*?)(?:\n\nSemantic topology:|$)", input_text, flags=re.S)
    return match.group(1).strip() if match else ""


def extract_topology(input_text: str) -> str:
    match = re.search(r"Semantic topology:\s*(.*?)(?:\n\nRegistered tools:|$)", input_text, flags=re.S)
    return match.group(1).strip() if match else ""


def build_rows(result_path: Path, eval_path: Path) -> list[dict[str, Any]]:
    result = json.loads(result_path.read_text(encoding="utf-8"))
    eval_records = {record["case_id"]: record for record in load_jsonl(eval_path)}
    rows: list[dict[str, Any]] = []

    for item in result["results"]:
        case_id = item["case_id"]
        eval_record = eval_records.get(case_id, {})
        expected_plan = eval_record.get("expected_plan")
        model_plan = item.get("plan")
        input_text = eval_record.get("input", "")
        row = {
            "case_id": case_id,
            "category": item.get("category", ""),
            "pass": bool(item.get("policy_valid")),
            "json_valid": bool(item.get("json_valid")),
            "schema_valid": bool(item.get("schema_valid")),
            "policy_valid": bool(item.get("policy_valid")),
            "mode_valid": bool(item.get("mode_valid")),
            "target_valid": item.get("target_valid"),
            "error": item.get("error") or "",
            "user_command": extract_user_command(input_text),
            "expected_mode": expected_plan.get("mode", "") if isinstance(expected_plan, dict) else "",
            "model_mode": model_plan.get("mode", "") if isinstance(model_plan, dict) else "",
            "expected_tools": tools(expected_plan),
            "model_tools": tools(model_plan),
            "expected_targets": target_nodes(expected_plan),
            "model_targets": target_nodes(model_plan),
            "expected_comm_mode": comm_mode(expected_plan),
            "model_comm_mode": comm_mode(model_plan),
            "expected_send": comm_send_drop(expected_plan, "send"),
            "model_send": comm_send_drop(model_plan, "send"),
            "expected_drop": comm_send_drop(expected_plan, "drop"),
            "model_drop": comm_send_drop(model_plan, "drop"),
            "expected_requires_human_ack": expected_plan.get("requires_human_ack", "") if isinstance(expected_plan, dict) else "",
            "model_requires_human_ack": model_plan.get("requires_human_ack", "") if isinstance(model_plan, dict) else "",
            "elapsed_s": item.get("elapsed_s"),
            "generation_speed_tps": item.get("generation_speed_tps"),
            "world_state": extract_world_state(input_text),
            "semantic_topology": extract_topology(input_text),
            "expected_plan_json": compact_json(expected_plan),
            "model_plan_json": compact_json(model_plan),
            "raw_answer": item.get("answer", ""),
            "full_input": input_text,
        }
        rows.append(row)
    return rows


def write_csv(path: Path, rows: list[dict[str, Any]], columns: list[str]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8-sig", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=columns, extrasaction="ignore")
        writer.writeheader()
        writer.writerows(rows)


MODE_ZH = {
    "mapped_navigation": "有图导航",
    "mapless_scout": "无图短距离侦察",
    "safe_hold": "安全等待/原地保持",
    "human_confirm": "请求人工确认",
}

TOOL_ZH = {
    "set_communication_policy": "设置通信策略",
    "create_navigation_subgoal": "创建导航子目标",
    "wait_until": "等待到达/状态满足",
    "capture_keyframe": "拍照/采集关键帧",
    "start_mapless_scout": "开始无图侦察",
    "request_human_confirm": "请求人工确认",
    "hold_position": "原地保持",
}

COMM_ZH = {
    "normal": "正常通信",
    "semantic_only": "弱网语义摘要模式",
    "keyframe_low_rate": "低频关键帧模式",
    "hold_remote": "暂停远程传输",
}

ITEM_ZH = {
    "task_state": "任务状态",
    "risk_events": "风险事件",
    "keyframe": "关键帧/照片",
    "semantic_topology": "语义拓扑",
    "navigation_feedback": "导航反馈",
    "world_state_summary": "世界状态摘要",
    "raw_video": "原始视频",
    "dense_pointcloud": "稠密点云",
    "full_log": "完整日志",
    "high_rate_images": "高频图像",
}

CATEGORY_ZH = {
    "normal_mapped_navigation": "正常有图导航",
    "weak_network_semantic_only": "弱网语义通信",
    "hard_weak_network_mapped_navigation": "弱网但仍需有图导航",
    "hard_weak_network_photo": "弱网拍照任务",
    "people_in_corridor_hold": "过道有人/拥堵需等待",
    "door_closed_confirm": "门关闭需人工确认",
    "localization_lost_hold": "定位丢失需保持/重定位",
    "target_not_found": "目标不存在",
    "hard_target_not_found": "目标不存在强化样本",
    "low_battery_confirm_charge": "低电量需确认回充",
    "hard_low_battery_confirm_charge": "低电量强化样本",
    "already_at_target_hold": "已在目标附近",
    "hard_already_at_target_hold": "已到达目标强化样本",
    "hard_charging_navigation": "充电点导航强化样本",
}


def zh_mode(mode: Any) -> str:
    if mode is None or mode == "":
        return ""
    return f"{MODE_ZH.get(str(mode), str(mode))}（{mode}）"


def zh_pipe(value: str, mapping: dict[str, str]) -> str:
    if not value:
        return ""
    parts = [part for part in str(value).split("|") if part]
    return "；".join(f"{mapping.get(part, part)}（{part}）" for part in parts)


def zh_bool(value: Any) -> str:
    if value is True or str(value).lower() == "true":
        return "是"
    if value is False or str(value).lower() == "false":
        return "否"
    if value in (None, ""):
        return ""
    return str(value)


def summarize_plan_zh(plan_json: str) -> str:
    if not plan_json:
        return ""
    try:
        plan = json.loads(plan_json)
    except Exception:
        return plan_json
    mode = zh_mode(plan.get("mode"))
    steps = []
    for step in plan.get("steps", []):
        if not isinstance(step, dict):
            continue
        tool = step.get("tool", "")
        args = step.get("arguments", {})
        desc = TOOL_ZH.get(tool, tool)
        if isinstance(args, dict):
            target = args.get("target_node") or args.get("target_node_id")
            if target:
                desc += f" -> {target}"
            if tool == "set_communication_policy":
                desc += f" -> {zh_mode(args.get('mode')) if args.get('mode') else args.get('mode', '')}"
        steps.append(desc)
    comm = plan.get("communication_policy", {})
    comm_text = ""
    if isinstance(comm, dict):
        comm_text = f"通信：{COMM_ZH.get(str(comm.get('mode', '')), comm.get('mode', ''))}；发送：{zh_pipe('|'.join(comm.get('send', [])), ITEM_ZH)}；丢弃：{zh_pipe('|'.join(comm.get('drop', [])), ITEM_ZH)}"
    return f"模式：{mode}。步骤：{'；'.join(steps)}。{comm_text}。需要人工确认：{zh_bool(plan.get('requires_human_ack'))}。原因：{plan.get('reason', '')}"


def build_chinese_rows(rows: list[dict[str, Any]]) -> list[dict[str, Any]]:
    out: list[dict[str, Any]] = []
    for row in rows:
        status = "通过" if row["pass"] else "未通过"
        item = {
            "用例编号": row["case_id"],
            "场景类别": CATEGORY_ZH.get(row["category"], row["category"]),
            "是否通过": status,
            "失败原因": row["error"],
            "用户命令": row["user_command"],
            "期望决策模式": zh_mode(row["expected_mode"]),
            "模型决策模式": zh_mode(row["model_mode"]),
            "期望工具步骤": zh_pipe(row["expected_tools"], TOOL_ZH),
            "模型工具步骤": zh_pipe(row["model_tools"], TOOL_ZH),
            "期望目标点": row["expected_targets"],
            "模型目标点": row["model_targets"],
            "期望通信模式": zh_mode(row["expected_comm_mode"]) if row["expected_comm_mode"] in MODE_ZH else COMM_ZH.get(row["expected_comm_mode"], row["expected_comm_mode"]),
            "模型通信模式": zh_mode(row["model_comm_mode"]) if row["model_comm_mode"] in MODE_ZH else COMM_ZH.get(row["model_comm_mode"], row["model_comm_mode"]),
            "期望发送内容": zh_pipe(row["expected_send"], ITEM_ZH),
            "模型发送内容": zh_pipe(row["model_send"], ITEM_ZH),
            "期望丢弃内容": zh_pipe(row["expected_drop"], ITEM_ZH),
            "模型丢弃内容": zh_pipe(row["model_drop"], ITEM_ZH),
            "期望是否人工确认": zh_bool(row["expected_requires_human_ack"]),
            "模型是否人工确认": zh_bool(row["model_requires_human_ack"]),
            "Schema是否合法": zh_bool(row["schema_valid"]),
            "Policy是否通过": zh_bool(row["policy_valid"]),
            "生成耗时秒": row["elapsed_s"],
            "生成速度tok每秒": row["generation_speed_tps"],
            "世界状态JSON": row["world_state"],
            "语义拓扑JSON": row["semantic_topology"],
            "期望计划中文摘要": summarize_plan_zh(row["expected_plan_json"]),
            "模型计划中文摘要": summarize_plan_zh(row["model_plan_json"]),
            "期望计划JSON": row["expected_plan_json"],
            "模型计划JSON": row["model_plan_json"],
            "模型原始回答": row["raw_answer"],
            "完整输入": row["full_input"],
        }
        out.append(item)
    return out


def main() -> None:
    parser = argparse.ArgumentParser(description="Export planner eval result JSON to CSV for analysis.")
    parser.add_argument("--result", required=True)
    parser.add_argument("--eval", default=str(ROOT / "data" / "local_llm_eval" / "go2w_floorplan_v4_eval.jsonl"))
    parser.add_argument("--out-dir", default=str(ROOT / "artifacts" / "local_llm_eval"))
    args = parser.parse_args()

    result_path = Path(args.result)
    eval_path = Path(args.eval)
    out_dir = Path(args.out_dir)
    stem = result_path.stem
    rows = build_rows(result_path, eval_path)

    summary_cols = [
        "case_id",
        "category",
        "pass",
        "error",
        "user_command",
        "expected_mode",
        "model_mode",
        "expected_tools",
        "model_tools",
        "expected_targets",
        "model_targets",
        "expected_comm_mode",
        "model_comm_mode",
        "expected_drop",
        "model_drop",
        "expected_requires_human_ack",
        "model_requires_human_ack",
        "schema_valid",
        "policy_valid",
        "elapsed_s",
        "generation_speed_tps",
    ]
    detail_cols = summary_cols + [
        "world_state",
        "semantic_topology",
        "expected_send",
        "model_send",
        "expected_plan_json",
        "model_plan_json",
        "raw_answer",
        "full_input",
    ]

    summary_path = out_dir / f"{stem}_summary.csv"
    detail_path = out_dir / f"{stem}_detail.csv"
    failures_path = out_dir / f"{stem}_failures.csv"
    chinese_path = out_dir / f"{stem}_中文详细分析.csv"
    chinese_failures_path = out_dir / f"{stem}_中文失败分析.csv"
    write_csv(summary_path, rows, summary_cols)
    write_csv(detail_path, rows, detail_cols)
    write_csv(failures_path, [row for row in rows if not row["pass"]], detail_cols)
    chinese_rows = build_chinese_rows(rows)
    chinese_cols = list(chinese_rows[0].keys()) if chinese_rows else []
    write_csv(chinese_path, chinese_rows, chinese_cols)
    write_csv(chinese_failures_path, [row for row in chinese_rows if row["是否通过"] != "通过"], chinese_cols)

    print(f"summary: {summary_path}")
    print(f"detail: {detail_path}")
    print(f"failures: {failures_path}")
    print(f"chinese detail: {chinese_path}")
    print(f"chinese failures: {chinese_failures_path}")
    print(f"rows: {len(rows)}")


if __name__ == "__main__":
    main()
