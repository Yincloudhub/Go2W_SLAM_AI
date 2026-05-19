# Local LLM SFT Data

This folder stores starter data for the local GO2W planner SFT workflow.

Files:

- `go2w_planner_sft_examples.jsonl`: small ShareGPT-style examples.

Rules:

- Assistant output must be a JSON string matching `schemas/local_llm_plan.schema.json`.
- Do not train concrete map coordinates as permanent knowledge. Runtime prompts provide map/topology.
- Include both good cases and conservative safety cases.
- Keep motion tools gated by `SafetySupervisor`.

