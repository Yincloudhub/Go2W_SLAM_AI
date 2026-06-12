# 机器狗 LLM 闭环框架说明

> **停用说明（2026-06-12）**：本文保留为历史实现记录，包含
> `test513.pcd`、`--current-node` 自动重定位等已废止逻辑。现场操作以
> `go2w_runtime_readiness.md` 和 `go2w_field_test_and_material_plan_20260612.md`
> 为准。当前唯一主地图是 `/home/unitree/test.pcd`。

本文档说明当前机器狗本地 LLM 闭环的代码结构、数据流和人工操作入口。目标是避免每次都依赖开发者手动下发命令，让现场人员可以自己验证、暂停、校准和继续扩展点位。

## 0. 推荐简化入口

现场优先使用 `--go`，不要再手动拼接 `--execute`、`--prompt-mode`、`--no-live-snapshot` 等参数：

```bash
cd /home/unitree/Go2W_SLAM_AI
python3 scripts/go2w_agent_entry.py \
  --go "去赵博老师的办公室门前，到了就站住" \
  --current-node yin_siyuan_station \
  --pretty
```

`--go` 会自动完成：

- 读取 live world_state。
- SLAM 未就绪时尝试启动 SLAM。
- `--current-node` 只提供导航拓扑上下文，不能用于重定位。
- 调用真实 LLM light 模式解析目标。
- 修复轻量模型常见的半截 JSON 输出。
- 通过 registry 和 gateway 安全门控。
- 执行导航。
- 到点距离达标后自动暂停。
- 默认打印简洁摘要，并把完整 JSON/CSV 日志写到 `artifacts/robot_runs`。

默认到点策略是“距离优先暂停”。yaw 只记录，不再默认卡住自动暂停。确实需要严格朝向时再加：

```bash
--require-arrival-yaw --arrival-yaw-rad 0.18
```

当前真实现场唯一主地图使用 `/home/unitree/test.pcd`。`test513.pcd` 只保留为历史文件，不得进入运行链路。

如果需要完整调试输出：

```bash
python3 scripts/go2w_agent_entry.py \
  --go "去赵博老师的办公室门前，到了就站住" \
  --current-node yin_siyuan_station \
  --full-output \
  --pretty
```

如果目标是已知点位、只想快速验证底盘，不想等真实 LLM：

```bash
python3 scripts/go2w_agent_entry.py \
  --go "去赵博老师的办公室门前，到了就站住" \
  --current-node yin_siyuan_station \
  --fast \
  --pretty
```

`--fast` 会优先走 deterministic/hybrid 目标匹配，已知别名能直接匹配到 registry 节点；复杂命令再回退到 LLM。

## 1. 当前闭环链路

当前链路是：

```text
用户中文指令
  -> scripts/go2w_agent_entry.py
  -> scripts/run_robot_closed_loop.py
  -> src/edge_autonomy/local_llm_planner.py
  -> configs/maps/go2w_real_site_map_registry.json
  -> /home/unitree/slam_gateway_refactor/build/slam_llm_command_client
  -> Unitree SLAM / navigation
  -> 到点监控
  -> pause_navigation
```

各层职责：

- `go2w_agent_entry.py`：现场统一入口。负责接收用户指令、可选执行、读取机器人状态、执行后到点暂停、语音播报。
- `run_robot_closed_loop.py`：规划和执行桥接层。负责调用 LLM/确定性规划、校验 registry、调用 SLAM 网关。
- `local_llm_planner.py`：LLM 规划器。把自然语言变成结构化 plan，再变成 `navigate_to_pose` 等网关命令。
- `go2w_real_site_map_registry.json`：真实现场地图和拓扑点表。所有可执行导航目标必须在这里。
- `slam_llm_command_client`：C++ SLAM 网关客户端。真正向 Unitree SLAM 发送导航、暂停、查询状态等命令。

## 2. 真实地图点位表

真实点位文件：

```text
E:\GO2W_0\configs\maps\go2w_real_site_map_registry.json
/home/unitree/Go2W_SLAM_AI/configs/maps/go2w_real_site_map_registry.json
```

当前原则：

- 只有 `status=real` 的地图允许实机执行。
- `status=simulation` 的演示地图禁止实机执行。
- 只要 `target_node` 不在 registry，禁止下发导航。
- PCD 点击点只是候选点，必须写入 registry 后才能执行。
- 到点后使用 `arrival_distance_m` 和 `arrival_yaw_rad` 判断是否暂停。

当前已知点：

- `initial_point`：真实初始点，来自早期录制 `wp_0`。
- `yin_siyuan_station`：尹思园工位，已根据现场真实停靠位置校准。
- `nie_guoli_office_front`：聂国篱办公室门口，来自录制 `wp_1`，现场已确认名称，但仍建议重新标点校准。
- `zhao_bo_office_front`：赵博办公室门口，PCD 点击候选点。
- `room_701_corridor`：701 门外走廊，PCD 点击候选点。

## 3. 为什么 HTML 初始点会看起来不一致

旧的 HTML 标注页只显示 PCD 俯视图，不叠加 registry 已知点。这样你看到的“初始点”可能是视觉判断出来的，但代码执行使用的是 registry 中的 `initial_point` 坐标。

新的标注页会叠加红色已知点，尤其是 `initial_point`，重新标点时要以红点为代码实际坐标参考。

新页面：

```text
E:\GO2W_0\artifacts\real_site_pcd\annotate_test_pcd_with_registry.html
E:\GO2W_0\artifacts\real_site_pcd\annotate_test513_pcd_with_registry.html
```

使用方式：

1. 打开带 `_with_registry.html` 的页面。
2. 红点是当前代码会使用的点位。
3. 蓝点是本次点击的新点。
4. 右侧填写 `node_id`、中文名称、别名和 yaw。
5. 点击“加入当前点击点”。
6. 点击“复制 JSON”，把 JSON 交给程序写入 registry。

## 4. 用户自己下发指令

在机器狗上执行：

```bash
cd /home/unitree/Go2W_SLAM_AI
python3 scripts/go2w_agent_entry.py \
  --command "去尹思园工位，到点后站住" \
  --registry configs/maps/go2w_real_site_map_registry.json \
  --map-id go2w_real_site \
  --prompt-mode hybrid \
  --no-live-snapshot \
  --execute \
  --arrival-distance-m 0.25 \
  --arrival-yaw-rad 0.22 \
  --arrival-monitor-s 60 \
  --pretty
```

推荐现场优先使用 `hybrid`：

- 已知点位：会快速确定性匹配，不等 LLM。
- 复杂语义：才回退到 LLM。
- 安全性更稳定，不容易因为 LLM JSON 格式错误导致失败。

如果要强制真实 LLM 验证：

```bash
python3 scripts/go2w_agent_entry.py \
  --command "去尹思园工位，到点后站住" \
  --registry configs/maps/go2w_real_site_map_registry.json \
  --map-id go2w_real_site \
  --prompt-mode light \
  --no-live-snapshot \
  --execute \
  --pretty
```

注意：当前真实 LLM `light` 模式偶尔会输出不合规 JSON，所以实机验证优先用 `hybrid`，LLM 质量优化单独推进。

## 5. 查看状态和暂停

查看状态：

```bash
cd /home/unitree/Go2W_SLAM_AI
python3 scripts/go2w_agent_entry.py --status --pretty
```

紧急停止优先用遥控器。

软件暂停可以直接调用网关客户端，或通过后续封装的 pause 入口执行。当前闭环执行后会自动监控到点并调用 `pause_navigation`。

现在也可以用统一入口暂停：

```bash
python3 scripts/go2w_agent_entry.py --pause --pretty
```

## 6. 校准流程

如果导航点偏了：

1. 用遥控器把机器狗停到正确位置。
2. 读取当前状态：

```bash
python3 scripts/go2w_agent_entry.py --status --pretty
```

3. 用返回的 `current_pose.pose` 覆盖 registry 中对应 `node_id` 的 `pose`。
4. 删除该点的 `needs_calibration` 标签。
5. 同步 registry 到机器狗。
6. 再跑一次同目标导航验证误差。

现在也可以直接用统一入口校准：

```bash
python3 scripts/go2w_agent_entry.py \
  --calibrate-node nie_guoli_office_front \
  --registry configs/maps/go2w_real_site_map_registry.json \
  --map-id go2w_real_site \
  --pretty
```

这个命令会读取当前 live pose，覆盖指定 node 的 pose，并删除 `needs_calibration` 标签。执行前要确保机器狗已经被遥控器停在你认为正确的位置。

## 7. 后续要补的工程化能力

优先级建议：

1. 修复真实 LLM 不合规 JSON 的兜底逻辑：目标不存在或格式错误时返回 `human_confirm`，而不是抛异常。
2. 把常用命令做成一个小型本地 Web/CLI 菜单，让现场人员不需要记长命令。
3. 增加 CSV 日志：记录用户输入、LLM 输出、目标点、执行结果、到点误差、耗时。
