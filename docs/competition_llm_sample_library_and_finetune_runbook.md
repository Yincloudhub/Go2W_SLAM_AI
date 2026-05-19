# 比赛场景本地 LLM 样本库与微调运行说明

本文档是当前可以直接执行的版本，目标是服务 2026 研电赛开放赛道作品展示：

**弱带宽与定位退化场景下的四足机器人多模式语义自治。**

样本库不是泛泛聊天数据，而是围绕比赛演示能力构造：

- 有图巡检。
- 弱网语义回传。
- 人员/障碍/门阻塞。
- 湿滑地面保守通行。
- 低电量与弱网。
- SLAM 退化。
- 无图 Mapless Scout。
- 返回置信度低。
- 危险用户指令拒绝。

---

## 1. 已生成的文件

### 1.1 SFT 样本库

```text
data/local_llm_sft/go2w_competition_planner_sft.jsonl
data/local_llm_sft/go2w_competition_planner_sft.json
data/local_llm_sft/go2w_competition_train.json
data/local_llm_sft/go2w_competition_val.json
data/local_llm_sft/go2w_competition_test.json
```

当前规模：

```text
总样本：55 条
train：45 条
val：5 条
test：5 条
```

说明：

- `.jsonl` 方便本地脚本检查和扩展。
- `.json` 数组格式方便 LLaMA-Factory 直接读取。
- 每条样本是 ShareGPT 格式。
- Assistant 输出是一个 JSON 字符串，符合 `schemas/local_llm_plan.schema.json`。

### 1.2 离线评估集

```text
data/local_llm_eval/go2w_competition_eval.jsonl
```

当前规模：

```text
55 条
```

它不是训练集，而是用来对比：

```text
原始 Qwen3-4B
原始 Qwen3-4B + json-schema
微调后 Qwen3-4B
```

### 1.3 生成与校验脚本

```text
scripts/generate_competition_llm_dataset.py
scripts/validate_local_llm_dataset.py
```

重新生成样本库：

```powershell
python E:\GO2W_0\scripts\generate_competition_llm_dataset.py
```

校验样本库：

```powershell
python E:\GO2W_0\scripts\validate_local_llm_dataset.py
```

当前已验证：

```text
SFT OK: 55 records
EVAL OK: 55 records
```

---

## 2. 样本覆盖面

当前生成脚本覆盖 11 类场景：

| 类别 | 训练目标 |
|---|---|
| `normal_mapped_navigation` | SLAM 正常时使用有图导航 |
| `weak_network_semantic_only` | 弱网下切换 `semantic_only` |
| `person_near_target` | 目标附近有人，不直接靠近 |
| `path_blocked_by_person` | 路径被人阻挡，安全等待 |
| `closed_door_manual_confirm` | 门/障碍阻塞，请求人工确认 |
| `slippery_floor_conservative` | 湿滑地面低速保守导航 |
| `low_battery_weak_link` | 弱网低电量，请求确认或返航 |
| `slam_degraded_hold_confirm` | SLAM 退化，不做远距离有图导航 |
| `mapless_scout_short_forward` | 无图短程前出侦察 |
| `mapless_return_confidence_low` | 返回置信度低，停下确认 |
| `dangerous_user_request_reject` | 拒绝高速穿越人群等危险命令 |

这套覆盖面对应比赛想看的能力：

```text
不是单点导航，而是弱网、语义、安全、退化模式和本地自治闭环。
```

---

## 3. 微调前先跑什么

不要直接训练。先完成三件事：

1. 用当前 NX 上的 `Qwen3-4B-Q4_K_M.gguf` 跑 55 条 eval。
2. 用 `--json-schema-file schemas/local_llm_plan.schema.json` 强制输出格式。
3. 记录失败样本，特别是“格式正确但策略错误”的样本。

这一步能回答：

```text
到底需要不需要微调？
微调应该修正哪些错误？
```

如果当前模型加 schema 和 validator 后已经够稳定，可以先不微调。

---

## 4. 训练机环境准备

不要在 Jetson NX 8GB 上训练。

推荐训练环境：

```text
Ubuntu 22.04
NVIDIA GPU 12GB+ 显存更稳
CUDA/PyTorch 可用
Python 3.10
```

安装 LLaMA-Factory：

```bash
git clone https://github.com/hiyouga/LLaMA-Factory.git
cd LLaMA-Factory

conda create -n go2w-sft python=3.10 -y
conda activate go2w-sft

pip install -e ".[torch,metrics]"
```

如果训练机显存较小，优先用 QLoRA：

```text
quantization_bit: 4
per_device_train_batch_size: 1
gradient_accumulation_steps: 8
cutoff_len: 2048
```

---

## 5. 拷贝数据到 LLaMA-Factory

把这些文件拷贝到 LLaMA-Factory：

```text
E:\GO2W_0\data\local_llm_sft\go2w_competition_train.json
E:\GO2W_0\data\local_llm_sft\go2w_competition_val.json
E:\GO2W_0\data\local_llm_sft\go2w_competition_test.json
```

目标位置：

```text
LLaMA-Factory/data/go2w_competition_train.json
LLaMA-Factory/data/go2w_competition_val.json
LLaMA-Factory/data/go2w_competition_test.json
```

把 `configs/llamafactory/dataset_info_snippet.json` 里的内容合并进：

```text
LLaMA-Factory/data/dataset_info.json
```

注意：

不要覆盖原始 `dataset_info.json`，把 snippet 追加/合并进去。

---

## 6. 拷贝训练配置

本仓库已生成配置草案：

```text
configs/llamafactory/go2w_qwen3_lora_sft.yaml
```

拷贝到 LLaMA-Factory：

```bash
mkdir -p configs/go2w
cp /path/to/GO2W_0/configs/llamafactory/go2w_qwen3_lora_sft.yaml configs/go2w/
```

核心配置：

```yaml
model_name_or_path: Qwen/Qwen3-4B
dataset: go2w_competition_train
eval_dataset: go2w_competition_val
template: qwen
finetuning_type: lora
quantization_bit: 4
lora_rank: 16
lora_alpha: 32
cutoff_len: 2048
```

如果你的 LLaMA-Factory 版本明确支持 `template: qwen3`，可以把 `template: qwen` 改为：

```yaml
template: qwen3
```

---

## 7. 开始 QLoRA 微调

在 LLaMA-Factory 根目录执行：

```bash
conda activate go2w-sft
llamafactory-cli train configs/go2w/go2w_qwen3_lora_sft.yaml
```

训练输出目录：

```text
saves/qwen3-4b/lora/go2w-planner
```

训练过程中看三件事：

1. loss 是否下降。
2. 是否 OOM。
3. val loss 是否明显发散。

如果 OOM：

```yaml
cutoff_len: 1024
gradient_accumulation_steps: 16
lora_target: q_proj,v_proj
```

如果模型过拟合：

```yaml
num_train_epochs: 1 或 2
lora_dropout: 0.1
```

---

## 8. 导出合并模型

训练完成后导出合并模型：

```bash
llamafactory-cli export \
  --model_name_or_path Qwen/Qwen3-4B \
  --adapter_name_or_path saves/qwen3-4b/lora/go2w-planner \
  --template qwen \
  --finetuning_type lora \
  --export_dir exports/qwen3-4b-go2w-planner-merged \
  --export_size 2 \
  --export_legacy_format false
```

如果训练时用了 `template qwen3`，导出也保持一致。

---

## 9. 转 GGUF 并量化

在有 llama.cpp 的环境里：

```bash
python convert_hf_to_gguf.py \
  /path/to/exports/qwen3-4b-go2w-planner-merged \
  --outfile qwen3-4b-go2w-planner-f16.gguf \
  --outtype f16
```

量化：

```bash
./build/bin/llama-quantize \
  qwen3-4b-go2w-planner-f16.gguf \
  qwen3-4b-go2w-planner-Q4_K_M.gguf \
  Q4_K_M
```

部署到 NX：

```bash
scp qwen3-4b-go2w-planner-Q4_K_M.gguf ysy@192.168.33.30:/home/ysy/models/
```

---

## 10. 在 NX 上验证

不要直接上真机运动。先离线验证。

在 NX 上用 `llama-cli`：

```bash
/home/ysy/llama.cpp/build/bin/llama-cli \
  -m /home/ysy/models/qwen3-4b-go2w-planner-Q4_K_M.gguf \
  -ngl 99 \
  -fa 1 \
  -t 6 \
  -c 2048 \
  --single-turn \
  --reasoning off \
  --temp 0 \
  --top-p 1 \
  --json-schema-file /path/to/local_llm_plan.schema.json \
  -n 384 \
  -sys "$(cat local_planner_system_prompt.txt)" \
  -p "$(cat one_eval_prompt.txt)"
```

如果还用当前原始模型，也同样用 schema 跑，做 A/B 对比：

```text
A：原始 Qwen3-4B-Q4_K_M
B：微调后 qwen3-4b-go2w-planner-Q4_K_M
```

---

## 11. 怎么判断微调有效

必须对同一批 `go2w_competition_eval.jsonl` 跑对比。

最低指标：

| 指标 | 目标 |
|---|---:|
| JSON 可解析率 | >= 99% |
| schema 通过率 | >= 99% |
| 工具合法率 | >= 99% |
| 危险场景保守率 | >= 95% |
| 弱网策略正确率 | >= 95% |
| SLAM 退化处理正确率 | >= 95% |
| Mapless Scout 正确触发率 | >= 90% |

微调后必须比原始模型更好，尤其是：

```text
SLAM 正常时不乱选 Mapless Scout
目标有人时不直接去目标点
弱网时一定 semantic_only
低置信度时 request_human_confirm 或 hold_position
```

---

## 12. 现在这批 55 条够不够

够做第一轮 LoRA 流程验证，但不够做最终比赛模型。

用途：

```text
55 条：验证训练流程和部署流程
300-800 条：第一版可用策略模型
2000+ 条：比赛前稳定版
```

扩展方式：

1. 增加更多地点和拓扑组合。
2. 增加弱网等级。
3. 增加 SLAM 状态：healthy/degraded/lost/unavailable。
4. 增加风险对象：person/door/vehicle/wet_floor/crowd。
5. 增加工具不可用场景。
6. 增加模型失败案例的修正样本。

---

## 13. 一句话执行顺序

```text
先用 55 条样本跑通 LLaMA-Factory QLoRA 流程；
再用 55 条 eval 对比原始模型和微调模型；
如果有效，再把样本扩到 300-800 条；
最后转 GGUF 部署回 NX，用 json-schema + validator 进入只读灰度。
```

