from __future__ import annotations

import argparse
import json
import os
import re
import shlex
import statistics
import sys
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import paramiko


ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))
SRC_ROOT = ROOT / "src"
if str(SRC_ROOT) not in sys.path:
    sys.path.insert(0, str(SRC_ROOT))

from edge_autonomy.local_llm_planner import DEFAULT_SYSTEM_PROMPT, PROMPT_TEMPLATE  # noqa: E402
from scripts.validate_local_llm_dataset import validate_plan  # noqa: E402


DEFAULT_CASES = ROOT / "data" / "local_llm_eval" / "go2w_field_eval.jsonl"
DEFAULT_OUT = ROOT / "artifacts" / "local_llm_eval"
DEFAULT_MODEL = "/home/ysy/models/Qwen_Qwen3-4B-Instruct-2507-Q4_K_M.gguf"
DEFAULT_SYSTEM = DEFAULT_SYSTEM_PROMPT


@dataclass
class CaseResult:
    case_id: str
    category: str
    elapsed_s: float
    prompt_speed_tps: float | None
    generation_speed_tps: float | None
    json_valid: bool
    schema_valid: bool
    policy_valid: bool
    mode_valid: bool
    target_valid: bool | None
    danger_rejected: bool | None
    error: str | None
    answer: str
    plan: dict[str, Any] | None


def load_jsonl(path: Path) -> list[dict[str, Any]]:
    return [json.loads(line) for line in path.read_text(encoding="utf-8").splitlines() if line.strip()]


def parse_ask_qwen_answer(output: str) -> str:
    match = re.search(r"(?s)\bAnswer:\n(.*?)(?:\n\nStats:\n|$)", output)
    if match:
        return match.group(1).strip()
    return output.strip()


def parse_stats(output: str) -> tuple[float | None, float | None]:
    prompt = re.search(r"Prompt speed:\s*([0-9.]+)\s+tok/s", output)
    generation = re.search(r"Generation speed:\s*([0-9.]+)\s+tok/s", output)
    return (
        float(prompt.group(1)) if prompt else None,
        float(generation.group(1)) if generation else None,
    )


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
    required = {"plan_id", "mode", "confidence", "reason", "steps", "communication_policy", "requires_human_ack"}
    for candidate in candidates:
        if required.issubset(candidate.keys()):
            return candidate
    if candidates:
        return candidates[-1]
    raise ValueError("no JSON object found")


def collect_tools(plan: dict[str, Any]) -> list[str]:
    return [str(step.get("tool")) for step in plan.get("steps", []) if isinstance(step, dict)]


def collect_target_nodes(plan: dict[str, Any]) -> list[str]:
    nodes: list[str] = []
    for step in plan.get("steps", []):
        if not isinstance(step, dict):
            continue
        args = step.get("arguments", {})
        if isinstance(args, dict):
            node = args.get("target_node")
            if isinstance(node, str):
                nodes.append(node)
    return nodes


def iter_numbers_for_keys(value: Any, key_names: set[str]) -> list[float]:
    found: list[float] = []
    if isinstance(value, dict):
        for key, item in value.items():
            if key in key_names and isinstance(item, (int, float)):
                found.append(float(item))
            found.extend(iter_numbers_for_keys(item, key_names))
    elif isinstance(value, list):
        for item in value:
            found.extend(iter_numbers_for_keys(item, key_names))
    return found


def check_policy(plan: dict[str, Any], expected: dict[str, Any], raw_answer: str) -> tuple[bool, bool, bool | None, bool | None, list[str]]:
    errors: list[str] = []
    mode = plan.get("mode")
    tools = collect_tools(plan)
    target_nodes = collect_target_nodes(plan)
    comm = plan.get("communication_policy", {}) if isinstance(plan.get("communication_policy"), dict) else {}

    expected_mode = expected.get("expected_mode")
    expected_modes = expected.get("expected_mode_one_of")
    if expected_mode:
        mode_valid = mode == expected_mode
        if not mode_valid:
            errors.append(f"mode {mode!r} != {expected_mode!r}")
    elif isinstance(expected_modes, list):
        mode_valid = mode in expected_modes
        if not mode_valid:
            errors.append(f"mode {mode!r} not in {expected_modes!r}")
    else:
        mode_valid = True

    for tool in expected.get("must_include_tools", []):
        if tool not in tools:
            errors.append(f"missing tool {tool}")
    for tool in expected.get("must_not_include_tools", []):
        if tool in tools:
            errors.append(f"forbidden tool {tool}")

    first_tool = expected.get("first_tool")
    if first_tool and (not tools or tools[0] != first_tool):
        errors.append(f"first tool {tools[0] if tools else None!r} != {first_tool!r}")

    expected_target = expected.get("expected_target_node")
    target_valid: bool | None = None
    if expected_target:
        target_valid = expected_target in target_nodes
        if not target_valid:
            errors.append(f"target {expected_target!r} not in {target_nodes!r}")

    allowed_targets = expected.get("allowed_target_nodes")
    if isinstance(allowed_targets, list):
        allowed = set(str(item) for item in allowed_targets)
        for target in target_nodes:
            if target not in allowed:
                errors.append(f"target {target!r} not in allowed topology nodes")
                target_valid = False

    forbidden_target = expected.get("must_not_target_node")
    if forbidden_target and forbidden_target in target_nodes:
        errors.append(f"forbidden target {forbidden_target!r}")
        target_valid = False

    if "requires_human_ack" in expected and plan.get("requires_human_ack") != expected["requires_human_ack"]:
        errors.append(f"requires_human_ack {plan.get('requires_human_ack')!r} != {expected['requires_human_ack']!r}")

    if expected.get("communication_mode") and comm.get("mode") != expected["communication_mode"]:
        errors.append(f"communication mode {comm.get('mode')!r} != {expected['communication_mode']!r}")

    drops = set(comm.get("drop", [])) if isinstance(comm.get("drop"), list) else set()
    for item in expected.get("must_drop", []):
        if item not in drops:
            errors.append(f"missing drop item {item}")

    sends = set(comm.get("send", [])) if isinstance(comm.get("send"), list) else set()
    for item in expected.get("must_send", []):
        if item not in sends:
            errors.append(f"missing send item {item}")

    for text in expected.get("must_not_contain", []):
        if text in raw_answer:
            errors.append(f"forbidden text {text!r} present")

    limit = expected.get("max_linear_speed_mps_lte")
    if isinstance(limit, (int, float)):
        speeds = iter_numbers_for_keys(plan, {"max_linear_speed_mps", "speed_mps"})
        if speeds and max(speeds) > float(limit):
            errors.append(f"speed {max(speeds):.3f} > {float(limit):.3f}")

    danger_rejected: bool | None = None
    if expected.get("expected_mode") == "safe_hold" or expected.get("category") == "reject_unsafe_navigation":
        danger_rejected = "create_navigation_subgoal" not in tools and mode in {"safe_hold", "human_confirm"}

    return not errors, mode_valid, target_valid, danger_rejected, errors


def ssh_run(client: paramiko.SSHClient, command: str, timeout_s: int) -> tuple[int, str, str]:
    stdin, stdout, stderr = client.exec_command(command, timeout=timeout_s)
    out = stdout.read().decode(errors="replace")
    err = stderr.read().decode(errors="replace")
    return stdout.channel.recv_exit_status(), out, err


def run_case(
    client: paramiko.SSHClient,
    case: dict[str, Any],
    *,
    model_path: str,
    ask_script: str,
    system_prompt: str,
    max_tokens: int,
    timeout_s: int,
) -> CaseResult:
    start = time.time()
    prompt = PROMPT_TEMPLATE.format(planner_context=case["input"])
    env = f"MODEL_PATH={shlex.quote(model_path)}"
    command = (
        f"{env} {shlex.quote(ask_script)} "
        f"--max-tokens {int(max_tokens)} "
        f"--system {shlex.quote(system_prompt)} "
        f"{shlex.quote(prompt)}"
    )
    exit_code, out, err = ssh_run(client, command, timeout_s=timeout_s)
    elapsed_s = time.time() - start
    full_output = out + ("\nSTDERR:\n" + err if err else "")
    prompt_tps, gen_tps = parse_stats(full_output)
    answer = parse_ask_qwen_answer(full_output)

    plan: dict[str, Any] | None = None
    json_valid = False
    schema_valid = False
    policy_valid = False
    mode_valid = False
    target_valid: bool | None = None
    danger_rejected: bool | None = None
    error: str | None = None

    if exit_code != 0:
        error = f"remote command exit {exit_code}: {err.strip()}"
    else:
        try:
            plan = extract_json_object(answer)
            json_valid = True
            validate_plan(plan)
            schema_valid = True
            expected = dict(case.get("expected_policy", {}))
            expected["category"] = case.get("category")
            policy_valid, mode_valid, target_valid, danger_rejected, errors = check_policy(plan, expected, answer)
            if errors:
                error = "; ".join(errors)
        except Exception as exc:
            error = str(exc)

    return CaseResult(
        case_id=str(case["case_id"]),
        category=str(case["category"]),
        elapsed_s=elapsed_s,
        prompt_speed_tps=prompt_tps,
        generation_speed_tps=gen_tps,
        json_valid=json_valid,
        schema_valid=schema_valid,
        policy_valid=policy_valid,
        mode_valid=mode_valid,
        target_valid=target_valid,
        danger_rejected=danger_rejected,
        error=error,
        answer=answer,
        plan=plan,
    )


def pct(count: int, total: int) -> float:
    return (count / total * 100.0) if total else 0.0


def summarize(results: list[CaseResult]) -> dict[str, Any]:
    total = len(results)
    target_results = [r for r in results if r.target_valid is not None]
    danger_results = [r for r in results if r.category == "reject_unsafe_navigation"]
    gen_speeds = [r.generation_speed_tps for r in results if r.generation_speed_tps is not None]
    return {
        "total": total,
        "json_valid": {"count": sum(r.json_valid for r in results), "rate": pct(sum(r.json_valid for r in results), total)},
        "schema_valid": {"count": sum(r.schema_valid for r in results), "rate": pct(sum(r.schema_valid for r in results), total)},
        "policy_valid": {"count": sum(r.policy_valid for r in results), "rate": pct(sum(r.policy_valid for r in results), total)},
        "mode_valid": {"count": sum(r.mode_valid for r in results), "rate": pct(sum(r.mode_valid for r in results), total)},
        "target_match": {
            "count": sum(r.target_valid is True for r in target_results),
            "total": len(target_results),
            "rate": pct(sum(r.target_valid is True for r in target_results), len(target_results)),
        },
        "danger_rejected": {
            "count": sum(r.policy_valid for r in danger_results),
            "total": len(danger_results),
            "rate": pct(sum(r.policy_valid for r in danger_results), len(danger_results)),
        },
        "generation_tps_avg": statistics.mean(gen_speeds) if gen_speeds else None,
        "generation_tps_min": min(gen_speeds) if gen_speeds else None,
        "generation_tps_max": max(gen_speeds) if gen_speeds else None,
    }


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Run GO2W planner eval on a NX-hosted llama.cpp model.")
    parser.add_argument("--cases", default=str(DEFAULT_CASES))
    parser.add_argument("--out-dir", default=str(DEFAULT_OUT))
    parser.add_argument("--host", default="192.168.33.30")
    parser.add_argument("--username", default="ysy")
    parser.add_argument("--password", default=os.environ.get("NX_SSH_PASSWORD", ""))
    parser.add_argument("--model-path", default=DEFAULT_MODEL)
    parser.add_argument("--ask-script", default="/home/ysy/ask_qwen.sh")
    parser.add_argument("--system", default=DEFAULT_SYSTEM)
    parser.add_argument("--max-tokens", type=int, default=768)
    parser.add_argument("--timeout-s", type=int, default=180)
    parser.add_argument("--limit", type=int, default=0)
    return parser


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    cases = load_jsonl(Path(args.cases))
    if args.limit > 0:
        cases = cases[: args.limit]

    if not args.password:
        raise SystemExit("Password is required via --password or NX_SSH_PASSWORD.")

    out_dir = Path(args.out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    stamp = time.strftime("%Y%m%d_%H%M%S")
    out_path = out_dir / f"nx_planner_eval_{stamp}.json"

    client = paramiko.SSHClient()
    client.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    client.connect(
        hostname=args.host,
        username=args.username,
        password=args.password,
        timeout=15,
        banner_timeout=15,
        auth_timeout=15,
    )

    try:
        results: list[CaseResult] = []
        for index, case in enumerate(cases, 1):
            print(f"[{index}/{len(cases)}] {case['case_id']} ...", flush=True)
            result = run_case(
                client,
                case,
                model_path=args.model_path,
                ask_script=args.ask_script,
                system_prompt=args.system,
                max_tokens=args.max_tokens,
                timeout_s=args.timeout_s,
            )
            status = "PASS" if result.policy_valid else "FAIL"
            print(f"  {status} json={result.json_valid} schema={result.schema_valid} policy={result.policy_valid} gen_tps={result.generation_speed_tps} err={result.error or ''}", flush=True)
            results.append(result)
    finally:
        client.close()

    payload = {
        "model_path": args.model_path,
        "cases": str(Path(args.cases)),
        "summary": summarize(results),
        "results": [result.__dict__ for result in results],
    }
    out_path.write_text(json.dumps(payload, ensure_ascii=False, indent=2), encoding="utf-8")
    print(json.dumps(payload["summary"], ensure_ascii=False, indent=2))
    print(f"wrote {out_path}")
    return 0 if payload["summary"]["policy_valid"]["count"] == payload["summary"]["total"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
