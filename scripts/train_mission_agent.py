"""GO2W MissionAgent SFT training — 100 steps CPU, logs to file.
Usage: python scripts/train_mission_agent.py [--steps 100] [--resume]
"""
import json, time, torch, sys, argparse
from pathlib import Path
from transformers import AutoModelForCausalLM, AutoTokenizer
from peft import LoraConfig, get_peft_model, TaskType

ROOT = Path(__file__).resolve().parents[1]
MODEL_ID = "Qwen/Qwen3-0.6B"
TRAIN_PATH = ROOT / "data" / "local_llm_sft" / "go2w_mission_agent_train.json"
OUT_DIR = ROOT / "saves" / "qwen3-0.6b" / "lora" / "go2w-mission-agent"
LOG_PATH = ROOT / "artifacts" / "training_log.jsonl"


def load_records(tokenizer, path, n=None, max_len=512):
    with open(path, encoding="utf-8") as f:
        data = json.load(f)
    if n:
        data = data[:n]
    records = []
    for rec in data:
        conv = rec["conversations"]
        text = (
            f"<|im_start|>system\n{rec['system']}<|im_end|>\n"
            f"<|im_start|>user\n{conv[0]['value']}<|im_end|>\n"
            f"<|im_start|>assistant\n{conv[1]['value']}<|im_end|>"
        )
        enc = tokenizer(text, truncation=True, max_length=max_len, return_tensors="pt")
        labels = enc["input_ids"].clone()
        ids = enc["input_ids"][0].tolist()
        asst = None
        for i in range(len(ids) - 2):
            if ids[i] == 151644 and ids[i + 1] == 77091:
                asst = i
                break
        if asst is not None:
            labels[0, :asst] = -100
        else:
            labels[0, : len(ids) // 2] = -100
        records.append((enc["input_ids"][0], enc["attention_mask"][0], labels[0]))
    return records


def log(msg):
    ts = time.strftime("%H:%M:%S")
    line = f"[{ts}] {msg}"
    print(line, flush=True)
    with open(LOG_PATH, "a", encoding="utf-8") as f:
        f.write(line + "\n")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--steps", type=int, default=100)
    parser.add_argument("--resume", action="store_true")
    args = parser.parse_args()

    LOG_PATH.parent.mkdir(parents=True, exist_ok=True)

    log("=== GO2W MissionAgent SFT Training ===")
    log(f"Model: {MODEL_ID}  |  Steps: {args.steps}  |  Output: {OUT_DIR}")

    # Load model
    log("Loading model...")
    t0 = time.time()
    model = AutoModelForCausalLM.from_pretrained(
        MODEL_ID, torch_dtype=torch.float32, device_map="cpu",
        trust_remote_code=True, local_files_only=True
    )
    tokenizer = AutoTokenizer.from_pretrained(MODEL_ID, trust_remote_code=True, local_files_only=True)
    if tokenizer.pad_token is None:
        tokenizer.pad_token = tokenizer.eos_token
    model.config.pad_token_id = tokenizer.pad_token_id
    log(f"  loaded in {time.time()-t0:.0f}s")

    # LoRA
    lora = LoraConfig(task_type=TaskType.CAUSAL_LM, r=8, lora_alpha=16, lora_dropout=0.05,
                      target_modules=["q_proj", "v_proj"])
    model = get_peft_model(model, lora)
    model.train()
    n_trainable = sum(p.numel() for p in model.parameters() if p.requires_grad)
    log(f"  trainable params: {n_trainable}")

    # Data
    log("Loading data...")
    records = load_records(tokenizer, TRAIN_PATH, n=200, max_len=512)
    log(f"  {len(records)} records")

    # Resume
    start_step = 0
    if args.resume and (OUT_DIR / "adapter_model.safetensors").exists():
        log("Resuming from saved adapter...")
        model = PeftModel.from_pretrained(model, str(OUT_DIR))
        model.train()
        # Try to find last step from log
        if LOG_PATH.exists():
            with open(LOG_PATH, encoding="utf-8") as f:
                for line in f:
                    if "step" in line.lower() and "/" in line:
                        try:
                            start_step = int(line.split("step")[1].split("/")[0].strip())
                        except:
                            pass
        log(f"  resuming from step {start_step}")

    opt = torch.optim.AdamW(model.parameters(), lr=2e-5)
    total_steps = args.steps
    losses = []

    log(f"Training {total_steps} steps (starting at {start_step})...")
    t_train = time.time()

    for step in range(start_step, total_steps):
        idx = step % len(records)
        ids, mask, labels = records[idx]

        t_step = time.time()
        out = model(input_ids=ids.unsqueeze(0), attention_mask=mask.unsqueeze(0), labels=labels.unsqueeze(0))
        out.loss.backward()
        opt.step()
        opt.zero_grad()

        loss_val = out.loss.item()
        losses.append(loss_val)
        elapsed = time.time() - t_train
        step_time = time.time() - t_step

        log(f"  step {step+1}/{total_steps}: loss={loss_val:.4f}  "
            f"({step_time:.0f}s/step, {elapsed:.0f}s total)")

        # Save checkpoint
        if (step + 1) % 50 == 0 or step == total_steps - 1:
            OUT_DIR.mkdir(parents=True, exist_ok=True)
            model.save_pretrained(str(OUT_DIR))
            tokenizer.save_pretrained(str(OUT_DIR))
            log(f"  checkpoint saved to {OUT_DIR}")

    total_time = time.time() - t_train
    log(f"Training complete in {total_time:.0f}s ({total_time/60:.1f} min)")
    avg_start = sum(losses[:5]) / min(5, len(losses))
    avg_end = sum(losses[-5:]) / min(5, len(losses))
    log(f"Loss: {losses[0]:.4f} → {losses[-1]:.4f}  (avg first 5: {avg_start:.4f} → avg last 5: {avg_end:.4f})")

    # Final save
    OUT_DIR.mkdir(parents=True, exist_ok=True)
    model.save_pretrained(str(OUT_DIR))
    tokenizer.save_pretrained(str(OUT_DIR))
    log(f"Final adapter saved to {OUT_DIR}")
    log("DONE")


if __name__ == "__main__":
    main()
