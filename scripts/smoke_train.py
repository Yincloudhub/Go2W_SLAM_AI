"""Smoke training: 10 steps, save LoRA adapter, use for inference eval.
Usage: python scripts/smoke_train.py
"""
import json, time, torch, os
from pathlib import Path
from transformers import AutoModelForCausalLM, AutoTokenizer
from peft import LoraConfig, get_peft_model, TaskType

ROOT = Path(__file__).resolve().parents[1]
MODEL_ID = "Qwen/Qwen3-0.6B"
TRAIN_PATH = ROOT / "data" / "local_llm_sft" / "go2w_mission_agent_train.json"
OUT_DIR = ROOT / "saves" / "qwen3-0.6b" / "lora" / "go2w-mission-agent-smoke"

def load_tokenized(tokenizer, path, max_records=50, max_len=2048):
    with open(path, encoding="utf-8") as f:
        data = json.load(f)
    records = []
    for rec in data[:max_records]:
        conv = rec["conversations"]
        text = (
            f"<|im_start|>system\n{rec['system']}<|im_end|>\n"
            f"<|im_start|>user\n{conv[0]['value']}<|im_end|>\n"
            f"<|im_start|>assistant\n{conv[1]['value']}<|im_end|>"
        )
        enc = tokenizer(text, truncation=True, max_length=max_len, return_tensors="pt")
        labels = enc["input_ids"].clone()
        # Mask all but assistant tokens: find <|im_start|>assistant position
        ids = enc["input_ids"][0].tolist()
        asst = None
        for i in range(len(ids) - 2):
            if ids[i] == 151644 and ids[i+1] == 77091:
                asst = i
                break
        if asst is not None:
            labels[0, :asst] = -100
        else:
            labels[0, :len(ids)//2] = -100
        records.append((enc["input_ids"][0], enc["attention_mask"][0], labels[0]))
    return records

def main():
    print("Loading model...", flush=True)
    model = AutoModelForCausalLM.from_pretrained(
        MODEL_ID, torch_dtype=torch.float32, device_map="cpu",
        trust_remote_code=True, local_files_only=True
    )
    tokenizer = AutoTokenizer.from_pretrained(MODEL_ID, trust_remote_code=True, local_files_only=True)
    if tokenizer.pad_token is None:
        tokenizer.pad_token = tokenizer.eos_token

    lora = LoraConfig(task_type=TaskType.CAUSAL_LM, r=8, lora_alpha=16, lora_dropout=0.05,
                      target_modules=["q_proj", "v_proj"])
    model = get_peft_model(model, lora)
    model.train()
    n_trainable = sum(p.numel() for p in model.parameters() if p.requires_grad)
    print(f"Trainable params: {n_trainable}", flush=True)

    records = load_tokenized(tokenizer, TRAIN_PATH, max_records=50)
    print(f"Loaded {len(records)} training records", flush=True)

    opt = torch.optim.AdamW(model.parameters(), lr=2e-5)
    steps = 10
    grad_accum = 4
    losses = []

    print(f"Training {steps} steps (grad_accum={grad_accum})...", flush=True)
    t0 = time.time()
    for step in range(steps):
        total_loss = 0.0
        for ga in range(grad_accum):
            idx = (step * grad_accum + ga) % len(records)
            ids, mask, labels = records[idx]
            ids = ids.unsqueeze(0)
            mask = mask.unsqueeze(0)
            labels = labels.unsqueeze(0)
            out = model(input_ids=ids, attention_mask=mask, labels=labels)
            loss = out.loss / grad_accum
            loss.backward()
            total_loss += out.loss.item()
        opt.step()
        opt.zero_grad()
        avg_loss = total_loss / grad_accum
        losses.append(avg_loss)
        elapsed = time.time() - t0
        print(f"  step {step+1}/{steps}: loss={avg_loss:.4f}, elapsed={elapsed:.0f}s", flush=True)

    total_time = time.time() - t0
    print(f"Training done in {total_time:.0f}s ({total_time/steps:.1f}s/step)", flush=True)
    print(f"Losses: {[f'{l:.4f}' for l in losses]}", flush=True)

    # Save adapter
    OUT_DIR.mkdir(parents=True, exist_ok=True)
    model.save_pretrained(str(OUT_DIR))
    tokenizer.save_pretrained(str(OUT_DIR))
    files = list(OUT_DIR.glob("*"))
    print(f"Saved to {OUT_DIR}: {[f.name for f in files]}", flush=True)

    # Verify loss trend
    if len(losses) >= 2 and losses[-1] < losses[0]:
        print("PASS: loss decreased", flush=True)
    else:
        print("WARN: loss did not decrease", flush=True)

if __name__ == "__main__":
    main()
