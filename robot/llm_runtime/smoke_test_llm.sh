#!/usr/bin/env bash
set -euo pipefail

TIMEOUT_S="${TIMEOUT_S:-240}"
OUT_FILE="${OUT_FILE:-/tmp/go2w_llm_smoke.out}"
ERR_FILE="${ERR_FILE:-/tmp/go2w_llm_smoke.err}"

rm -f "$OUT_FILE" "$ERR_FILE"

start_s="$(date +%s)"
set +e
timeout "$TIMEOUT_S" \
  /home/unitree/llm_runtime/scripts/ask_qwen.sh \
  --max-tokens 32 \
  --system 'Output only one JSON object. Do not explain.' \
  '{"ok":true}' \
  > "$OUT_FILE" 2> "$ERR_FILE"
rc=$?
set -e
end_s="$(date +%s)"

echo "RC=$rc ELAPSED=$((end_s - start_s))s"
echo "--- stdout bytes ---"
wc -c "$OUT_FILE"
echo "--- stdout preview ---"
python3 - <<PY
from pathlib import Path
data = Path("$OUT_FILE").read_bytes()[:1000]
print(data.decode("utf-8", errors="replace"))
PY
echo "--- stderr tail ---"
tail -80 "$ERR_FILE" || true

exit "$rc"
