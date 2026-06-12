# GO2W 底盘能力简化路线

本文档记录 2026-05-23 现场测试后的结论和下一步工程路线。目标是把机器狗从“靠人拼命令调试”推进到“一个入口、自动预检、可解释、可恢复”的底盘闭环。

## 1. 现场结论

这次赵博办公室门口测试证明了主链路已经能跑通：

```text
用户指令
  -> 统一入口 --go
  -> live 状态预检
  -> 真实 LLM light 模式解析
  -> LLM 半截 JSON 自动修复为标准 plan
  -> registry 目标点 zhao_bo_office_front
  -> SLAM 网关 navigate_to_pose
  -> 到点监控
  -> pause_navigation
```

当时的历史发现（已被后续统一地图决策取代）：

- `/home/unitree/test.pcd` 在尹思园工位重定位失败，ICP 分数略高于阈值。
- `/home/unitree/test513.pcd` 用同一尹思园位姿重定位成功。
- 当时曾临时切换到 `/home/unitree/test513.pcd`。该临时结论现已废止；当前唯一主地图固定为 `/home/unitree/test.pcd`，`test513.pcd` 不得进入运行链路。
- 真实 LLM 会输出包含 `target_node` 的半截 JSON，之前会报 schema 错误；现在已增加 `repair_partial_navigation_plan` 修复层。
- 赵博点导航时距离已经到 `0.043m`，但 yaw 没满足导致未自动暂停；现在默认改成“距离达标就暂停，yaw 只记录”，需要强制朝向时再加 `--require-arrival-yaw`。

## 2. 现场最小命令

以后现场优先只用一条入口：

```bash
cd /home/unitree/Go2W_SLAM_AI
python3 scripts/go2w_agent_entry.py \
  --go "去赵博老师的办公室门前，到了就站住" \
  --current-node yin_siyuan_station \
  --pretty
```

含义：

- `--go`：一键闭环，默认执行，不再手动拼 `--execute --no-live-snapshot --prompt-mode light`。
- `--current-node yin_siyuan_station`：仅表示规划器当前导航拓扑点，不再用于重定位。
- 默认会先查安全状态，不允许导航时不会硬走。
- 默认距离达标自动暂停。

编码不稳定时用 base64：

```bash
python3 scripts/go2w_agent_entry.py --go-b64 <utf8_base64> --current-node yin_siyuan_station --pretty
```

## 3. 当前入口职责

`scripts/go2w_agent_entry.py` 现在应承担现场层能力：

- `--go`：一键自然语言闭环。
- `--status`：查定位、健康、安全、导航状态。
- `--pause`：软件暂停导航。
- `--calibrate-node <node_id>`：用当前 live pose 覆盖某个 registry 点。
- `--relocate --relocation-anchor mapping_origin --confirm-relocation mapping_origin`：显式人工确认的活动锚点重定位。
- `--current-node <node_id>`：只提供导航拓扑上下文。

原则：

- 现场人员不要直接调用 C++ 网关。
- LLM 失败不能导致底盘盲动。
- 所有能导航的目标必须来自 registry。
- registry 中 `status=simulation` 的地图禁止实机执行。

## 4. 底盘能力分层

建议把底盘能力分成三层，不要把所有事情都塞给 LLM。

### 第一层：底盘原语

这些命令必须确定、短延迟、可测试：

- `get_world_state`
- `start_slam`
- `relocate_to_anchor`
- `navigate_to_node`
- `pause_navigation`
- `hold_position`
- `calibrate_node_from_current_pose`
- `speak`
- `capture_keyframe`

下一步建议继续补：

- `return_to_last_safe_node`
- `nudge_forward / nudge_back / nudge_left / nudge_right`
- `turn_to_yaw`
- `face_node`
- `dock_or_charge`
- `emergency_stop_soft`

这些不应该依赖 LLM 生成复杂 JSON，而应该由入口直接调用确定性函数。

### 第二层：任务状态机

把常见任务做成固定状态机：

```text
preflight
  -> ensure_localized
  -> plan_target
  -> execute_navigation
  -> arrival_monitor
  -> post_action
  -> final_status
```

其中：

- `ensure_localized`：未定位时阻断自动任务，并要求人工选择与当前物理位置一致的活动 verified 重定位锚点。`mapping_origin` 只是唯一建图原点和当前唯一已验证锚点，不是未来所有重定位的固定位置。
- `plan_target`：先 deterministic alias 匹配，再用 LLM。
- `execute_navigation`：只接收 registry 目标。
- `arrival_monitor`：距离优先，到点就暂停；yaw 只作为可选条件。
- `post_action`：拍照、说话、等待、返回等。

### 第三层：LLM 语义层

LLM 只负责把人话变成任务意图：

```json
{
  "intent": "navigate",
  "target_node": "zhao_bo_office_front",
  "post_actions": ["hold_position", "capture_keyframe"]
}
```

不要让 LLM 直接决定底盘安全策略。安全策略由本地规则覆盖：

- 目标不存在：`human_confirm`
- 未定位：拒绝导航并提示人工执行活动 verified 锚点重定位
- 低电量：拒绝非回充任务或要求确认
- 弱网：降级通信策略
- 到点：距离达标优先暂停

## 5. 后续开发路线

### 阶段 A：把现场闭环变短

目标：一条命令可用。

- 完成并稳定 `--go`。
- `--go` 默认使用真实 LLM light；当前已增加 `--fast`，已知点位可优先走 deterministic/hybrid 匹配，减少等待。
- 输出人能看懂的摘要，不默认打印超长 JSON。当前 `--go` 已默认输出摘要。
- 增加 `--log-dir`，保存完整 JSON 到文件。当前 `--go` 已默认写入 `artifacts/robot_runs`。
- 每次执行生成一行 CSV：输入、目标、LLM 耗时、是否执行、终点误差、是否暂停。当前已落到 `go2w_agent_runs.csv`，后续再补“是否修复 LLM 输出”等字段。

### 阶段 B：把底盘能力做成可复用 API

目标：LLM 和人都调用同一组底盘 API。

- 新建 `src/edge_autonomy/chassis_controller.py`
- 封装 `ensure_localized`、`navigate_to_node`、`arrival_monitor`、`pause`
- `go2w_agent_entry.py` 只做 CLI 薄封装
- C++ 网关保持底层通信，不承载业务逻辑

当前已新增：

- `src/edge_autonomy/chassis_controller.py`：底盘网关、registry 节点、重定位、暂停等确定性能力的封装。
- `src/edge_autonomy/execution_report.py`：把完整执行 trace 汇总成简洁摘要，并写 JSON/CSV 日志。

下一步继续迁移：

- `auto_pause_on_arrival`
- `calibrate_node_from_current_pose`
- `go_preflight / go_auto_relocate_blocked`

### 阶段 C：把重定位做稳

目标：关机重启后少依赖人工。

- `mapping_origin_anchor_id` 始终只指向一个建图坐标原点
- 同一 PCD 可登记多个独立重定位锚点；只有现场验证为 `verified_*` 的锚点进入活动列表
- registry 每个关键点增加 `relocalization_quality`
- 记录每次重定位 ICP 结果
- 如果重定位失败，保持不运动并要求检查物理对齐、地图身份和 ICP；不自动尝试其他锚点
- 支持操作员从活动 verified 锚点中明确选择，不把 `--current-node` 当作重定位锚点
- 当前主地图固定为 `/home/unitree/test.pcd`，不再混用 `test513.pcd`

### 阶段 D：让 LLM 常驻

目标：把 13 秒级推理降下来。

- 不再每次拉起模型进程
- 上 `llama.cpp server` 或本地常驻 Python service
- `go2w_agent_entry.py` 通过 HTTP/IPC 调用
- 失败时回退 deterministic intent

### 阶段 E：任务组合

目标：自然语言组合动作可控。

示例：

```text
去赵博老师办公室门口，到了拍一张照，然后小声说我到了。
```

拆成：

```text
navigate_to_node(zhao_bo_office_front)
hold_position()
capture_keyframe()
speak()
```

## 6. 近期最值得做的三件事

1. 做 `chassis_controller.py`，把 `go2w_agent_entry.py` 里的底盘逻辑下沉，入口变薄。
2. 做执行摘要和 CSV 日志，否则每次 JSON 太长，很难复盘。
3. 做 `--fast` 模式：已知点位不用等 LLM，复杂指令才调用 LLM。

## 7. 当前推荐现场 SOP

开机后：

```bash
python3 scripts/go2w_agent_entry.py --status --pretty
```

如果未定位但机器狗在尹思园工位：

```bash
python3 scripts/go2w_agent_entry.py --go "回到初始点" --current-node yin_siyuan_station --pretty
```

正常任务：

```bash
python3 scripts/go2w_agent_entry.py --go "去赵博老师的办公室门前，到了就站住" --current-node yin_siyuan_station --pretty
```

偏了以后：

```bash
python3 scripts/go2w_agent_entry.py --pause --pretty
python3 scripts/go2w_agent_entry.py --calibrate-node zhao_bo_office_front --pretty
```

这条路线的核心是：现场只需要知道“我现在大概在哪个已知点”和“我要去哪”，其余由入口自动处理。
