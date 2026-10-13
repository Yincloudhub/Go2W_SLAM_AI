"""Post-training inference evaluation for GO2W MissionAgent smoke SFT.
Usage: python scripts/eval_mission_agent_smoke.py [adapter_path]
"""
from __future__ import annotations

import json
import sys
from pathlib import Path
from typing import Any, Dict, List

import torch
from transformers import AutoModelForCausalLM, AutoTokenizer
from peft import PeftModel

ROOT = Path(__file__).resolve().parents[1]
EVAL_PATH = ROOT / "data" / "local_llm_eval" / "go2w_mission_agent_eval.jsonl"
BASE_MODEL = "Qwen/Qwen3-4B"
MISSION_SCHEMA = "go2w_mission_plan_v1"

BLOCKED_TOKENS = ["cmd_vel", "Unitree API", "shell_command", "slam_operate", "direct chassis"]

SAMPLE_CATEGORIES = {
    "motion_same_map": 2,
    "motion_cross_map": 2,
    "mixed_motion_behavior": 2,
    "behavior": 2,
    "chat": 1,
    "water_pool_blocked": 1,
}


def load_eval_records() -> List[Dict[str, Any]]:
    return [json.loads(line) for line in EVAL_PATH.read_text(encoding="utf-8").splitlines() if line.strip()]


def pick_samples(records: List[Dict[str, Any]]) -> List[Dict[str, Any]]:
    samples: List[Dict[str, Any]] = []
    cat_counts: Dict[str, int] = {}
    for r in records:
        cat = r["category"]
        if cat in ("motion_same_map", "motion_cross_map", "mixed_motion_behavior", "behavior"):
            if cat_counts.get(cat, 0) < 2:
                samples.append(r)
                cat_counts[cat] = cat_counts.get(cat, 0) + 1
        elif cat == "chat":
            if cat_counts.get("chat", 0) < 1:
                samples.append(r)
                cat_counts["chat"] = cat_counts.get("chat", 0) + 1
    # Find one terrace water pool blocked sample
    for r in records:
        prompt = r["prompt"]
        plan = r["expected_mission_plan"]
        if "wp_832687" in prompt:
            motions = [s for s in plan["steps"] if s["type"] == "motion"]
            if any(s.get("status", "").startswith("blocked") for s in motions):
                samples.append(r)
                break
    return samples


def build_prompt(record: Dict[str, Any]) -> str:
    """Extract user prompt from eval record."""
    prompt = record.get("prompt", "")
    # prompt format: "msg: ...\nctx: ...\nreturn JSON keys: reply, mission_plan"
    # Strip to just the essential
    return prompt


def check_output(text: str, case_id: str) -> Dict[str, Any]:
    """Validate inference output against handoff requirements."""
    issues = []
    try:
        output = json.loads(text)
    except json.JSONDecodeError as e:
        return {"case_id": case_id, "valid_json": False, "issues": [f"JSON parse error: {e}"], "text": text[:200]}

    result: Dict[str, Any] = {"case_id": case_id, "valid_json": True, "issues": []}

    # Check keys
    if set(output.keys()) != {"reply", "mission_plan"}:
        result["issues"].append(f"output keys: {sorted(output.keys())}")

    plan = output.get("mission_plan", {})

    # Schema check
    if plan.get("schema") != MISSION_SCHEMA:
        result["issues"].append(f"schema: {plan.get('schema')}")

    # execution_enabled
    if plan.get("execution_enabled") is not False:
        result["issues"].append("plan execution_enabled != false")
    for s in plan.get("steps", []):
        if s.get("execution_enabled") is not False:
            result["issues"].append(f"step {s.get('step_id')} execution_enabled != false")

    # Blocked tokens
    plan_text = json.dumps(plan, ensure_ascii=False)
    for tok in BLOCKED_TOKENS:
        if tok.lower() in plan_text.lower():
            result["issues"].append(f"blocked token: {tok}")

    # wp_832687 special case
    if "wp_832687" in plan_text:
        motions = [s for s in plan.get("steps", []) if s["type"] == "motion"]
        if not all(s.get("status", "").startswith("blocked") for s in motions):
            result["issues"].append("wp_832687 route not blocked")

    # Cross-PCD map_transition
    intent = plan.get("intent_kind", "")
    steps = plan.get("steps", [])
    step_types = [s["type"] for s in steps]
    if "map_transition" in step_types and "motion_cross_map" not in record.get("category", ""):
        pass  # OK
    if intent == "motion" and len(set(s.get("to_map", "") for s in steps if s["type"] == "motion")) > 1:
        if "map_transition" not in step_types:
            result["issues"].append("cross-PCD missing map_transition")

    result["reply"] = output.get("reply", "")[:100]
    result["plan_summary"] = {
        "intent": plan.get("intent_kind"),
        "steps": len(steps),
        "step_types": step_types,
        "exec_enabled": plan.get("execution_enabled"),
    }
    return result


def main():
    adapter_path = sys.argv[1] if len(sys.argv) > 1 else str(ROOT / "saves" / "qwen3-4b" / "lora" / "go2w-mission-agent-smoke")

    records = load_eval_records()
    samples = pick_samples(records)
    print(f"Selected {len(samples)} samples from {len(records)} eval records")

    # Load model
    print(f"Loading base model: {BASE_MODEL}")
    tokenizer = AutoTokenizer.from_pretrained(BASE_MODEL, trust_remote_code=True)
    model = AutoModelForCausalLM.from_pretrained(
        BASE_MODEL,
        torch_dtype=torch.float32,
        device_map="cpu",
        trust_remote_code=True,
    )

    # Load LoRA adapter
    try:
        model = PeftModel.from_pretrained(model, adapter_path)
        print(f"Loaded LoRA adapter from {adapter_path}")
    except Exception as e:
        print(f"WARNING: Could not load adapter ({e}), running base model only")

    model.eval()

    results = []
    for i, sample in enumerate(samples):
        case_id = sample["case_id"]
        category = sample["category"]
        prompt = build_prompt(sample)

        print(f"\n[{i+1}/{len(samples)}] {case_id} ({category})")

        inputs = tokenizer(prompt, return_tensors="pt", truncation=True, max_length=4096)
        with torch.no_grad():
            outputs = model.generate(
                **{k: v for k, v in inputs.items()},
                max_new_tokens=1024,
                temperature=0.1,
                do_sample=False,
                pad_token_id=tokenizer.eos_token_id,
            )
        response = tokenizer.decode(outputs[0][inputs["input_ids"].shape[1]:], skip_special_tokens=True)

        check = check_output(response, case_id)
        check["category"] = category
        results.append(check)

        status = "OK" if not check["issues"] else f"ISSUES: {len(check['issues'])}"
        print(f"  {status}")
        if check["issues"]:
            for issue in check["issues"]:
                print(f"    - {issue}")
        print(f"  reply: {check.get('reply', 'N/A')[:80]}")

    # Summary
    print(f"\n{'='*60}")
    total_issues = sum(len(r["issues"]) for r in results)
    valid_json = sum(1 for r in results if r["valid_json"])
    print(f"Results: {valid_json}/{len(results)} valid JSON, {total_issues} total issues")
    for r in results:
        if r["issues"]:
            print(f"  {r['case_id']}: {r['issues']}")

    # Save results
    out_path = ROOT / "artifacts" / "smoke_eval_results.json"
    out_path.parent.mkdir(parents=True, exist_ok=True)
    out_path.write_text(json.dumps(results, ensure_ascii=False, indent=2), encoding="utf-8")
    print(f"\nSaved results to {out_path}")


if __name__ == "__main__":
    main()
