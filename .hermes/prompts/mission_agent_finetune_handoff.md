# GO2W MissionAgent SFT Handoff

Objective:
- Own the offline MissionAgent fine-tuning smoke run for the GO2W robot dog.
- Do not validate SLAM, do not move the robot, and do not send any chassis command.
- The robot is currently allowed to stay lying down. This task is only cloud/local LLM data, training, and evaluation.

Current repo state:
- Workspace: `E:\GO2W_0`
- Dataset generator: `scripts\generate_mission_agent_sft_dataset.py`
- Dataset validator: `scripts\validate_mission_agent_sft_dataset.py`
- LLaMA-Factory config: `configs\llamafactory\go2w_mission_agent_qwen3_lora_sft.yaml`
- LLaMA-Factory dataset registry snippet: `configs\llamafactory\dataset_info_snippet.json`
- Main generated data:
  - `data\local_llm_sft\go2w_mission_agent_topology.json`
  - `data\local_llm_sft\go2w_mission_agent_sft.json`
  - `data\local_llm_sft\go2w_mission_agent_sft.jsonl`
  - `data\local_llm_sft\go2w_mission_agent_train.json`
  - `data\local_llm_sft\go2w_mission_agent_val.json`
  - `data\local_llm_sft\go2w_mission_agent_test.json`
  - `data\local_llm_eval\go2w_mission_agent_eval.jsonl`

Latest dataset snapshot:
- Total SFT records: 487
- Split: train 391, val 48, test 48
- Categories:
  - `motion_same_map`: 194
  - `motion_cross_map`: 42
  - `motion_cross_map_reverse`: 26
  - `behavior`: 151
  - `mixed_motion_behavior`: 26
  - `mixed_cross_map_behavior`: 24
  - `chat`: 14
  - `edge_safety`: 10

What the model must learn:
- There are two different concepts:
  - Motion: go from one topology node to another, possibly across PCD maps.
  - Behavior: capture photo, speak, report, wait, or normal chat.
- Mixed requests must be decomposed into motion and behavior (order varies).
- Cross-PCD routing is layered:
  - choose involved PCD maps;
  - identify current map/current node/nearest node;
  - identify target map/target node;
  - use curated route validator output;
  - produce a read-only `MissionPlan`.
- LLM output must never execute motion. Every plan and every step keeps `execution_enabled=false`.
- Motion authority remains `robot_side_nav_core_and_safety_gate`.
- LLM must not output low-level velocity, raw motor, vendor SDK, shell, or direct drive commands.
- `map_terrace_wc:wp_832687` is the terrace water pool semantic target, but its pose is invalid because pose recording failed. Treat it as `blocked_needs_pose_repair` until recaptured.

Preflight on the Windows workspace:
```powershell
cd E:\GO2W_0
$env:PYTHONIOENCODING='utf-8'
python scripts\generate_mission_agent_sft_dataset.py
python scripts\validate_mission_agent_sft_dataset.py
$env:PYTHONPATH='src'
python -m unittest tests.test_mission_agent -v
python -m py_compile scripts\generate_mission_agent_sft_dataset.py scripts\validate_mission_agent_sft_dataset.py
```

Expected preflight result:
- Generator writes 487 records.
- Validator prints `MissionAgent SFT OK` and `MissionAgent EVAL OK`.
- `tests.test_mission_agent` passes.

Training target:
- First run only a smoke SFT/QLoRA job, not a full production training run.
- Base model in current config: `Qwen/Qwen3-4B`.
- Adapter output path in current config: `saves/qwen3-4b/lora/go2w-mission-agent`.
- If using LLaMA-Factory, register the three `go2w_mission_agent_*` datasets using the snippet in `configs\llamafactory\dataset_info_snippet.json`, or copy these JSON files into the LLaMA-Factory data directory with matching dataset names.

Suggested LLaMA-Factory command:
```powershell
llamafactory-cli train configs\llamafactory\go2w_mission_agent_qwen3_lora_sft.yaml
```

Smoke-run adjustment:
- Prefer a small `max_steps` smoke override or a temporary copy of the YAML with a small step limit.
- Confirm loss/logging works before spending on a long run.
- Do not launch a paid cloud GPU job unless the account, budget, and hardware target are explicitly confirmed.

Post-training evaluation:
- Run inference on representative records from `data\local_llm_eval\go2w_mission_agent_eval.jsonl`.
- Check that every response is valid JSON with keys `reply` and `mission_plan`.
- Check `mission_plan.schema == go2w_mission_plan_v1`.
- Check `execution_enabled=false` at plan and step levels.
- Check low-level blocked tokens are absent:
  - `cmd_vel`
  - `Unitree API`
  - `shell_command`
  - `slam_operate`
  - `direct chassis`
- Check cross-PCD outputs keep map-specific motion routes separated by `map_transition`.
- Check any route involving `map_terrace_wc:wp_832687` is blocked, even if the invalid node is the start or an intermediate route node.

Deliverables back to Codex/user:
- Exact environment used: local GPU, remote GPU, or Hugging Face job.
- Training command and config diff if any temporary smoke config was used.
- Validation logs.
- Adapter/output path.
- 10 sample inference outputs:
  - 2 same-map motion
  - 2 cross-PCD motion
  - 2 mixed motion+behavior
  - 2 behavior-only
  - 1 chat
  - 1 invalid terrace water pool case
- Any failures with the raw error text.

Hard boundaries:
- Do not SSH into the robot for this task unless asked.
- Do not start SLAM validation.
- Do not send motion commands.
- Do not change topology coordinates during training.
- Do not mark `map_terrace_wc:wp_832687` executable until the physical pose has been recaptured.
