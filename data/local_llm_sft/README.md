# Local LLM SFT Data

This folder stores starter data for the local GO2W planner SFT workflow.

Files:

- `go2w_planner_sft_examples.jsonl`: small ShareGPT-style examples.
- `go2w_mission_agent_topology.json`: curated three-PCD topology used for MissionAgent samples.
- `go2w_mission_agent_sft.json` / `.jsonl`: full MissionAgent ShareGPT-style dataset.
- `go2w_mission_agent_train.json`, `go2w_mission_agent_val.json`, `go2w_mission_agent_test.json`: split files for LLaMA-Factory.

Legacy planner rules:

- Assistant output must be a JSON string matching `schemas/local_llm_plan.schema.json`.
- Do not train concrete map coordinates as permanent knowledge. Runtime prompts provide map/topology.
- Include both good cases and conservative safety cases.
- Keep motion tools gated by `SafetySupervisor`.

MissionAgent dataset:

- Assistant output is a JSON string with `reply` and `mission_plan`.
- `mission_plan.schema` is `go2w_mission_plan_v1`.
- All motion and behavior steps keep `execution_enabled=false`.
- `map_terrace_wc:wp_832687` is the terrace water pool, but its pose is intentionally marked invalid until recaptured.
- Current default generation writes 487 records: 391 train, 48 val, 48 test.

Regenerate and validate:

```powershell
$env:PYTHONIOENCODING='utf-8'
python scripts\generate_mission_agent_sft_dataset.py
python scripts\validate_mission_agent_sft_dataset.py
```

LLaMA-Factory training config:

```powershell
llamafactory-cli train configs\llamafactory\go2w_mission_agent_qwen3_lora_sft.yaml
```
