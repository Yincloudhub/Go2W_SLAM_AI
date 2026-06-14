# CommunicationPolicyExecutor v1

更新：2026-06-14

## 目标

P0-4 把弱网从 Planner 标签收口为可执行的本地通信策略：

```text
TaskQueue / MissionDecision / execution feedback
  -> append-only journal
  -> monotonic event sequence
  -> pending outbox
  -> cumulative ack
  -> reconnect replay of missing state events only
```

网络不可用时，本地 `PerceptionContext -> TaskQueue -> MissionDecisionEngine ->
Gateway` 链不等待远端。通信层不拥有运动执行权，也不调用 Gateway。

## 实现

核心实现：

```text
src/edge_autonomy/communication_policy.py
schemas/communication_journal_v1.schema.json
scripts/go2w_communication_journal.py
```

执行入口：

```text
scripts/run_robot_closed_loop.py
scripts/go2w_agent_entry.py
```

默认 journal：

```text
artifacts/communication/communication_journal_v1.jsonl
```

每条事件记录包含：

```text
source_id / event_id / sequence / timestamp_ms / event_type /
queue_id / replay_safe=true / execution_directive=false / payload
```

ack 也是追加写记录。只有收到累计 `ack_sequence` 后，对应前缀才从待补传集合
移除。重启时扫描 journal，恢复 last sequence、last ack、待补传事件以及每个
queue 的任务、决定、执行占用和终态。

## 幂等与安全边界

- 真实执行前必须原子追加 `execution_claim`。
- 同一 `queue_id` 已有真实执行占用或终态时，不得自动再次执行。
- 进程在占用后中断时，恢复策略固定为
  `restore_state_without_automatic_motion_replay`，需要人工核对，不自动续跑。
- 重连补传只发送状态事件，不发送 `slam_command`、`target_pose`、
  `operator_ack`、Unitree API、原始视频、稠密点云或图像字节。
- 远端只允许提交合法 `TaskQueue` proposal；入口返回
  `route=mission_decision_engine`，不会直接调用 Gateway。
- 重复 `message_id`、已占用 queue、已完成 queue 和包含直接运动字段的远端消息
  全部拒绝。

## 链路状态

| link_state | 本地 journal | 补传 |
|---|---|---|
| `normal` | 追加写 | 可发送 pending |
| `weak` | 追加写 | 仅有界语义事件 |
| `disconnected` | 追加写 | transport 零调用 |
| `recovered` | 追加写 | 按 sequence 补发未 ack 事件 |

## 无运动操作

查看状态：

```bash
cd /home/unitree/Go2W_SLAM_AI
PYTHONPATH=src python3 scripts/go2w_communication_journal.py status --pretty
```

查看断线状态下不会补传：

```bash
PYTHONPATH=src python3 scripts/go2w_communication_journal.py replay \
  --link-state disconnected --pretty
```

查看重连待补传：

```bash
PYTHONPATH=src python3 scripts/go2w_communication_journal.py replay \
  --link-state recovered --pretty
```

追加 ack：

```bash
PYTHONPATH=src python3 scripts/go2w_communication_journal.py ack \
  --ack-sequence <remote_ack_sequence> --pretty
```

上述命令只读写本地 journal，不启动 SLAM、Gateway、传感器或底盘运动。

## 本地验收

2026-06-14 本地无运动验收：

```text
python_full_unittest: 327/327 passed
disconnected_dry_run: passed
gateway_checked: false
motion_executed: false
journal_sequence: 1..6
pending_events: 6
disconnected_replay_events: 0
recovered_replay_sequence: 1..6
ack_sequence: 6
pending_after_ack: 0
automatic_resume_allowed: false
replay_execution_directive: false
journal_slam_command: absent
journal_target_pose: absent
```

机器人无运动验收必须在 Git fast-forward 部署后执行，且仍不得启动 SLAM、
Gateway、D435、XT16 或底盘运动。
