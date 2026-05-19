#!/usr/bin/env bash
set -euo pipefail

MODEL_PATH="${MODEL_PATH:-/home/unitree/models/Qwen_Qwen3-4B-Instruct-2507-Q4_K_M.gguf}"
LLAMA_CLI="${LLAMA_CLI:-/home/unitree/llm_runtime/llama.cpp/build/bin/llama-cli}"
MAX_TOKENS="${MAX_TOKENS:-768}"
THREADS="${THREADS:-6}"
CTX_SIZE="${CTX_SIZE:-4096}"
TEMP="${TEMP:-0.1}"

SYSTEM_PROMPT=""
if [[ "${1:-}" == "--system" ]]; then
  SYSTEM_PROMPT="$2"
  shift 2
fi

PROMPT="$*"

exec "$LLAMA_CLI" \
  -m "$MODEL_PATH" \
  -t "$THREADS" \
  -c "$CTX_SIZE" \
  -n "$MAX_TOKENS" \
  --temp "$TEMP" \
  -p "${SYSTEM_PROMPT}

${PROMPT}"
