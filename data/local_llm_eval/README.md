# Local LLM Evaluation Data

This folder stores offline evaluation scenarios for the GO2W local planner.

Generated files:

- `go2w_competition_eval.jsonl`: scenario inputs plus expected policy checks.

Use this before training:

1. Run the current Qwen3-4B model with JSON schema constraints.
2. Parse the generated plan.
3. Validate JSON schema.
4. Validate policy checks in `expected_policy`.
5. Record failures and convert important failures into SFT samples.

