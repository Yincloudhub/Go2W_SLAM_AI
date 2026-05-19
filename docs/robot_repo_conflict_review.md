# Robot Repository Conflict Review

Date: 2026-05-19

This note reviews the current robot-side layout after cloning `agent/llm-on-robot` to the GO2W machine.

## Current Robot Layout

```text
/home/unitree/Go2W_SLAM_AI
/home/unitree/slam_gateway_refactor
/home/unitree/llm_runtime
/home/unitree/models
```

## No Direct Path Conflict

There is no direct file overwrite conflict among the current paths:

- `/home/unitree/Go2W_SLAM_AI` is the git-managed project clone.
- `/home/unitree/slam_gateway_refactor` is the existing Unitree SDK-facing SLAM gateway.
- `/home/unitree/llm_runtime` is the local inference runtime directory.
- `/home/unitree/models` is for model weights and must stay out of git.

## Important Responsibility Boundaries

### `Go2W_SLAM_AI/cpp`

The current `cpp/` directory is a SDK-free dry-run executor.

It validates `LocalLlmPlan` and prints the planned execution sequence. It does not call Unitree SDK and must not be treated as a replacement for `slam_gateway_refactor`.

### `slam_gateway_refactor`

This is the real robot execution gateway. It owns:

- Unitree SDK2 service calls.
- `slam_llm_command_client`.
- `slam_keyboard_client`.
- `SlamGateway`.
- safety checks before real navigation.

LLM integration should call into this layer only after plan validation.

### `llm_runtime`

This should contain llama.cpp and small runtime scripts only.

Do not commit:

- llama.cpp build outputs.
- GGUF model weights.
- runtime logs.
- temporary prompt/output captures.

### `models`

This directory should contain GGUF or future model files only. It is intentionally ignored by git.

## Current Robot-Side LLM Status

Completed:

- llama.cpp source exists at `/home/unitree/llm_runtime/llama.cpp`.
- CPU `llama-cli` builds and exists at `/home/unitree/llm_runtime/llama.cpp/build/bin/llama-cli`.
- Git clone exists at `/home/unitree/Go2W_SLAM_AI`.
- Robot SSH key authenticates to GitHub as `ccj-bot`.

Not completed yet:

- GGUF model has not been uploaded to `/home/unitree/models`.
- Runtime wrapper `/home/unitree/llm_runtime/scripts/ask_qwen.sh` still needs to be installed.
- A first local inference smoke test on the robot has not been run.
- `slam_gateway_refactor` source has a known rebuild blocker: `pose.mode = j.value("mode", 0);`.

## Known Integration Risks

1. `slam_gateway_refactor` rebuild risk:

   In `/home/unitree/slam_gateway_refactor/src/slam_gateway.cpp`, `addCurrentPoseAsWaypoint()` references an undefined `j`.

   Recommended patch:

   ```cpp
   pose.mode = 0;
   ```

2. Do not let LLM output call raw Unitree API IDs.

   The gateway already rejects raw API IDs in `LlmCommandProcessor`.

3. Weak-network behavior should be deterministic.

   It should be triggered by local `world_state.link_quality`, not by user text.

4. The model may output partial or inner JSON objects.

   Schema validation must reject these before execution.

5. `LocalLlmPlan` to `slam_llm_command_client` conversion is not complete yet.

   Current path is:

   ```text
   local_llm_plan -> dry-run executor
   ```

   Target path is:

   ```text
   local_llm_plan -> C++ validator -> C++ executor -> slam_llm_command_client/SlamGateway
   ```

## Recommended Next Steps

1. Upload the GGUF model to `/home/unitree/models`.
2. Install `ask_qwen.sh` into `/home/unitree/llm_runtime/scripts`.
3. Run a local inference smoke test.
4. Fix and rebuild `slam_gateway_refactor`.
5. Add C++ plan validator/executor into the real gateway path.
6. Start with dry-run mode before any real navigation.

