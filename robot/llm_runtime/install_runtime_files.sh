#!/usr/bin/env bash
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
RUNTIME_ROOT="${GO2W_LLM_RUNTIME_ROOT:-/home/unitree/llm_runtime}"
MODEL_ROOT="${GO2W_MODEL_ROOT:-/home/unitree/models}"

mkdir -p "${RUNTIME_ROOT}/scripts" "${MODEL_ROOT}"

install -m 0755 \
  "${REPO_ROOT}/robot/llm_runtime/ask_qwen.sh" \
  "${RUNTIME_ROOT}/scripts/ask_qwen.sh"

cat <<EOF
Installed robot LLM runtime files:
  ${RUNTIME_ROOT}/scripts/ask_qwen.sh

External model directory:
  ${MODEL_ROOT}

Model weights are intentionally not managed by git.
EOF

