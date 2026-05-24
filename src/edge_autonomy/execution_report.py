from __future__ import annotations

import csv
import json
import time
from pathlib import Path
from typing import Any


def summarize_agent_output(output: dict[str, Any]) -> dict[str, Any]:
    summary: dict[str, Any] = {
        "command": output.get("command"),
        "execute": output.get("execute"),
        "nav_speed_mps": output.get("nav_speed_mps"),
        "nav_mode": output.get("nav_mode"),
        "target_node": None,
        "llm_elapsed_s": None,
        "plan_mode": None,
        "planner_route": None,
        "force_llm": None,
        "executed": False,
        "arrived": None,
        "paused": None,
        "final_distance_m": None,
        "preflight_allowed": None,
        "auto_relocated": False,
        "llm_plan_repaired": False,
        "resolve_matched": None,
        "resolved_target": None,
        "resolve_ambiguous": None,
        "resolve_reason": "",
        "node_count": None,
        "nodes": None,
        "preflight_reason": "",
        "slam_started": None,
        "pointcloud_alive": None,
        "slam_info_alive": None,
        "blocked_reason": "",
        "steps": [step.get("step") for step in output.get("steps", []) if isinstance(step, dict)],
    }
    for step in output.get("steps", []):
        if not isinstance(step, dict):
            continue
        name = step.get("step")
        result = step.get("result", {})
        if name == "go_route":
            summary["planner_route"] = step.get("prompt_mode")
            summary["force_llm"] = step.get("force_llm")
        elif name == "list_nodes":
            nodes = step.get("nodes", [])
            if isinstance(nodes, list):
                summary["node_count"] = len(nodes)
                summary["nodes"] = [
                    {
                        "node_id": node.get("node_id"),
                        "name": node.get("name"),
                        "needs_calibration": node.get("needs_calibration"),
                    }
                    for node in nodes
                    if isinstance(node, dict)
                ]
        elif name in {"go_preflight", "preflight_only"}:
            summary["preflight_allowed"] = step.get("allowed")
            summary["preflight_reason"] = step.get("reason", "")
        elif name in {"ensure_slam", "go_auto_ensure_slam"} and isinstance(result, dict):
            summary["slam_started"] = result.get("started")
            summary["pointcloud_alive"] = result.get("pointcloud_alive")
            summary["slam_info_alive"] = result.get("slam_info_alive")
            summary["preflight_allowed"] = result.get("preflight_allowed")
            summary["preflight_reason"] = result.get("preflight_reason", "")
        elif name == "resolve_target" and isinstance(result, dict):
            summary["resolve_matched"] = result.get("matched")
            selected = result.get("selected", {})
            if isinstance(selected, dict):
                summary["resolved_target"] = selected.get("node_id")
            summary["resolve_ambiguous"] = result.get("ambiguous")
            summary["resolve_reason"] = result.get("reason", "")
        elif name == "go_auto_relocate":
            summary["auto_relocated"] = True
        elif name == "closed_loop" and isinstance(result, dict):
            payload = result.get("result", {})
            if isinstance(payload, dict):
                planner = payload.get("planner", {})
                if isinstance(planner, dict):
                    summary["llm_elapsed_s"] = planner.get("llm_elapsed_s")
                    plan = planner.get("plan", {})
                    if isinstance(plan, dict):
                        summary["plan_mode"] = plan.get("mode")
                        reason = str(plan.get("reason", ""))
                        plan_id = str(plan.get("plan_id", ""))
                        summary["llm_plan_repaired"] = "repaired partial model output" in reason or plan_id.startswith("repaired_plan_")
                    slam_command = planner.get("slam_command", {})
                    if isinstance(slam_command, dict):
                        summary["target_node"] = slam_command.get("target_node")
                execution = payload.get("execution", {})
                if isinstance(execution, dict):
                    summary["executed"] = bool(execution.get("executed"))
                    summary["blocked_reason"] = execution.get("blocked_reason", "")
        elif name == "auto_pause_on_arrival" and isinstance(result, dict):
            summary["arrived"] = result.get("arrived")
            summary["paused"] = result.get("paused")
            samples = result.get("samples", [])
            if isinstance(samples, list) and samples:
                last = samples[-1]
                if isinstance(last, dict):
                    summary["final_distance_m"] = last.get("distance_to_target_m")
    return summary


def write_execution_log(output: dict[str, Any], log_dir: str | Path) -> dict[str, str]:
    root = Path(log_dir)
    root.mkdir(parents=True, exist_ok=True)
    ts = time.strftime("%Y%m%d_%H%M%S")
    json_path = root / f"go2w_agent_{ts}.json"
    csv_path = root / "go2w_agent_runs.csv"
    json_path.write_text(json.dumps(output, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")

    summary = summarize_agent_output(output)
    fieldnames = [
        "timestamp",
        "command",
        "execute",
        "nav_speed_mps",
        "nav_mode",
        "target_node",
        "llm_elapsed_s",
        "plan_mode",
        "planner_route",
        "force_llm",
        "executed",
        "arrived",
        "paused",
        "final_distance_m",
        "preflight_allowed",
        "auto_relocated",
        "llm_plan_repaired",
        "resolve_matched",
        "resolved_target",
        "resolve_ambiguous",
        "resolve_reason",
        "preflight_reason",
        "slam_started",
        "pointcloud_alive",
        "slam_info_alive",
        "blocked_reason",
    ]
    row = {"timestamp": ts, **{key: summary.get(key) for key in fieldnames if key != "timestamp"}}
    exists = csv_path.exists()
    with csv_path.open("a", newline="", encoding="utf-8-sig") as fh:
        writer = csv.DictWriter(fh, fieldnames=fieldnames)
        if not exists:
            writer.writeheader()
        writer.writerow(row)
    return {"json": str(json_path), "csv": str(csv_path)}
