#!/usr/bin/env bash
set -euo pipefail

MODEL_PATH="${MODEL_PATH:-/home/unitree/models/Qwen_Qwen3-4B-Instruct-2507-Q4_K_M.gguf}"
CUDA_LLAMA_CLI="${CUDA_LLAMA_CLI:-/home/unitree/llm_runtime/llama.cpp/build-cuda/bin/llama-cli}"
CPU_LLAMA_CLI="${CPU_LLAMA_CLI:-/home/unitree/llm_runtime/llama.cpp/build/bin/llama-cli}"
if [[ -z "${LLAMA_CLI:-}" ]]; then
  if [[ -x "$CUDA_LLAMA_CLI" ]]; then
    LLAMA_CLI="$CUDA_LLAMA_CLI"
  else
    LLAMA_CLI="$CPU_LLAMA_CLI"
  fi
fi

MAX_TOKENS="${MAX_TOKENS:-768}"
THREADS="${THREADS:-6}"
CTX_SIZE="${CTX_SIZE:-8192}"
TEMP="${TEMP:-0.1}"
GPU_LAYERS="${GPU_LAYERS:-99}"
FLASH_ATTN="${FLASH_ATTN:-1}"
REASONING_MODE="${REASONING_MODE:-off}"
SHOW_RAW="${SHOW_RAW:-0}"

SYSTEM_PROMPT=""
while [[ $# -gt 0 ]]; do
  case "$1" in
    --system)
      SYSTEM_PROMPT="$2"
      shift 2
      ;;
    --max-tokens)
      MAX_TOKENS="$2"
      shift 2
      ;;
    --threads)
      THREADS="$2"
      shift 2
      ;;
    --ctx)
      CTX_SIZE="$2"
      shift 2
      ;;
    --temp)
      TEMP="$2"
      shift 2
      ;;
    --show-raw)
      SHOW_RAW=1
      shift
      ;;
    --)
      shift
      break
      ;;
    *)
      break
      ;;
  esac
done

PROMPT="$*"

cmd=(
  "$LLAMA_CLI"
  -m "$MODEL_PATH"
  -t "$THREADS"
  -c "$CTX_SIZE"
  -n "$MAX_TOKENS"
  --temp "$TEMP"
  --single-turn
  --reasoning "$REASONING_MODE"
  --simple-io
  --no-display-prompt
  --color off
  --log-disable
)

if [[ "$LLAMA_CLI" == *"/build-cuda/"* ]]; then
  cmd+=(-ngl "$GPU_LAYERS" -fa "$FLASH_ATTN")
fi

if [[ -n "$SYSTEM_PROMPT" ]]; then
  cmd+=(-sys "$SYSTEM_PROMPT")
fi

cmd+=(-p "$PROMPT")

if [[ "$SHOW_RAW" == "1" ]]; then
  printf 'LLAMA_CLI=%s\n' "$LLAMA_CLI" >&2
  printf 'CMD=' >&2
  printf '%q ' "${cmd[@]}" >&2
  printf '\n' >&2
fi

exec "${cmd[@]}"
