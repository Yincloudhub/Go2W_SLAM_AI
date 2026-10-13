# GO2W MissionAgent SFT — GPU 训练交付包

## 搬运清单

从 `E:\GO2W_0\` 拷到笔记本，以下文件必须带：

```
GO2W_0\
├── 📄 GPU_TRAINING_README.md          ← 本文件
├── 📁 data\local_llm_sft\             ← 全部 512 条训练数据
│   ├── go2w_mission_agent_train.json  (410 条)
│   ├── go2w_mission_agent_val.json    (51 条)
│   └── go2w_mission_agent_test.json   (51 条)
├── 📁 configs\llamafactory\
│   └── go2w_mission_agent_qwen3_lora_sft.yaml  ← 生产配置
├── 📁 scripts\
│   ├── generate_mission_agent_sft_dataset.py   ← 数据集生成器
│   └── validate_mission_agent_sft_dataset.py   ← 验证器
└── 📁 saves\qwen3-0.6b\lora\go2w-mission-agent\  ← 500步预热权重（可选）
```

## 笔记本环境

```powershell
# 确认 GPU 可用
python -c "import torch; print(torch.cuda.is_available(), torch.cuda.get_device_name(0))"
# 预期: True, NVIDIA GeForce RTX 5070

# 安装依赖
pip install llamafactory torch transformers peft datasets
# 或者用 uv:
uv pip install llamafactory
```

## 注册数据集

把 `data/local_llm_sft/` 下的三个 JSON 拷到 LLaMA-Factory 的 data 目录，并创建 `dataset_info.json`：

```powershell
# 找到 LLaMA-Factory 安装位置
python -c "import llamafactory, os; print(os.path.dirname(llamafactory.__file__))"
# 例如: C:\...\site-packages\llamafactory

# 拷贝数据
copy data\local_llm_sft\go2w_mission_agent_*.json <llamafactory>\data\

# 在 <llamafactory>\data\ 下创建 dataset_info.json：
```

`dataset_info.json` 内容：
```json
{
  "go2w_mission_agent_train": {
    "file_name": "go2w_mission_agent_train.json",
    "formatting": "sharegpt",
    "columns": {"messages": "conversations", "system": "system"},
    "tags": {"role_tag": "from", "content_tag": "value", "user_tag": "user", "assistant_tag": "assistant"}
  },
  "go2w_mission_agent_val": {
    "file_name": "go2w_mission_agent_val.json",
    "formatting": "sharegpt",
    "columns": {"messages": "conversations", "system": "system"},
    "tags": {"role_tag": "from", "content_tag": "value", "user_tag": "user", "assistant_tag": "assistant"}
  },
  "go2w_mission_agent_test": {
    "file_name": "go2w_mission_agent_test.json",
    "formatting": "sharegpt",
    "columns": {"messages": "conversations", "system": "system"},
    "tags": {"role_tag": "from", "content_tag": "value", "user_tag": "user", "assistant_tag": "assistant"}
  }
}
```

## 训练

```powershell
# 一行命令启动
llamafactory-cli train configs\llamafactory\go2w_mission_agent_qwen3_lora_sft.yaml

# 或者覆盖参数做烟雾测试：
llamafactory-cli train configs\llamafactory\go2w_mission_agent_qwen3_lora_sft.yaml \
    --max_steps 50 --output_dir saves/qwen3-4b/lora/go2w-mission-agent-smoke
```

### 配置说明 (`go2w_mission_agent_qwen3_lora_sft.yaml`)

| 参数 | 值 | 说明 |
|------|-----|------|
| model | Qwen/Qwen3-4B | 4B 参数基座 |
| finetuning_type | lora | LoRA 微调 |
| quantization_bit | 4 | QLoRA 4-bit（省显存） |
| lora_rank | 16 | LoRA 秩 |
| cutoff_len | 6144 | 上下文长度 |
| per_device_train_batch_size | 1 | 每卡 batch |
| gradient_accumulation_steps | 8 | 等效 batch=8 |
| num_train_epochs | 3 | 3 轮 |
| learning_rate | 2e-5 | |

**调整建议**：5070 12GB 跑 QLoRA 4B 没问题。如果想更快，把 `per_device_train_batch_size` 改成 2 或 4，`gradient_accumulation_steps` 相应减少。

## 数据集说明

- **总记录**：512 条（原始 308 + 扩充 204）
- **划分**：train 410 / val 51 / test 51
- **类别**：motion_same_map 207, behavior 163, motion_cross_map 42, motion_cross_map_reverse 26, mixed_motion_behavior 26, mixed_cross_map_behavior 24, chat 14, edge_safety 10
- **回复多样性**：每类 4-17 种表达
- **Provider 混合**：local 315 / cloud 197
- 含短上下文 schema 聚焦样本
- 含 `current_pose` 系统解析起点样本
- 含行为链、反向 mixed、多 PCD 过渡
- wp_832687（露台水池）在所有记录中标记为 blocked

### 重新生成（如需修改）

```powershell
$env:PYTHONIOENCODING='utf-8'
python scripts\generate_mission_agent_sft_dataset.py
python scripts\validate_mission_agent_sft_dataset.py
$env:PYTHONPATH='src'
python -m unittest tests.test_mission_agent -v
```

## 训练后推理评估

```powershell
python -u -c "
import json, torch
from transformers import AutoModelForCausalLM, AutoTokenizer
from peft import PeftModel

model = AutoModelForCausalLM.from_pretrained('Qwen/Qwen3-4B', torch_dtype=torch.float16, device_map='auto', trust_remote_code=True)
tokenizer = AutoTokenizer.from_pretrained('Qwen/Qwen3-4B', trust_remote_code=True)
model = PeftModel.from_pretrained(model, 'saves/qwen3-4b/lora/go2w-mission-agent')
model.eval()

with open('data/local_llm_eval/go2w_mission_agent_eval.jsonl', encoding='utf-8') as f:
    rec = json.loads(f.readline())

prompt = rec['prompt']
inputs = tokenizer(prompt, return_tensors='pt', truncation=True, max_length=4096).to('cuda')
with torch.no_grad():
    outputs = model.generate(**inputs, max_new_tokens=512, do_sample=False)
resp = tokenizer.decode(outputs[0][inputs['input_ids'].shape[1]:], skip_special_tokens=True)

try:
    out = json.loads(resp)
    print('✅ Valid JSON')
    plan = out['mission_plan']
    print(f'   schema: {plan.get(\"schema\")}')
    print(f'   exec_enabled: {plan.get(\"execution_enabled\")}')
    print(f'   intent: {plan.get(\"intent_kind\")}')
    print(f'   steps: {len(plan.get(\"steps\",[]))}')
except:
    print('❌ Not valid JSON')
    print(resp[:200])
"
```

## 硬边界

- LLM 不直接控制底盘
- `execution_enabled=false` 贯穿所有 plan 和 step
- 禁止输出 cmd_vel / Unitree API / shell_command / slam_operate
- `map_terrace_wc:wp_832687` 在 pose 修复前不可执行
- `current_pose` 场景下 from_node 必须 = route[0].node_id
- 跨 PCD 路由必须含 map_transition 步骤

## CPU 预热权重（可选参考）

`saves/qwen3-0.6b/lora/go2w-mission-agent/` 是在 CPU 上跑了 500 步的 Qwen3-0.6B LoRA。Loss 从 2.29→1.12（512 上下文）+ 3.06→1.19（1024 上下文）。模型已学会拓扑术语但 schema 不正确。GPU 上应从头用 4B 训练，不需要这个权重。

## 预估时间

| GPU | 模型 | 1000 步 | 3 epochs |
|-----|------|---------|----------|
| RTX 5070 12GB | Qwen3-4B QLoRA | ~20 min | ~40 min |

最终适配器约 50MB（LoRA 权重 + config），可拷回机器人部署。
