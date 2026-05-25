# 机器狗本地 Agent 总入口

`scripts/go2w_agent_entry.py` 是机器狗本地闭环的统一入口，建议后续现场调试都优先从这个脚本进入，而不是手动拼多个命令。

## 常用命令

查看当前 SLAM / 定位 / 安全门状态：

```bash
python3 scripts/go2w_agent_entry.py --status --pretty
```

当前真实执行默认使用真实现场注册表：

```bash
configs/maps/go2w_real_site_map_registry.json
```

模拟平面图注册表只允许训练、评测和 dry-run。真实执行时，如果地图 `status` 是 `simulation`，闭环脚本会直接拒绝执行。

启动 SLAM 后端并发起重定位：

```bash
python3 scripts/go2w_agent_entry.py --start-slam --relocate --status --pretty
```

只规划和检查，不执行移动：

```bash
python3 scripts/go2w_agent_entry.py --command "去赵博办公室门口" --no-live-snapshot --status --pretty
```

真实执行，并在执行后监控 20 秒：

```bash
python3 scripts/go2w_agent_entry.py --command "去赵博办公室门口" --no-live-snapshot --execute --monitor-s 20 --pretty
```

如果从 Windows/SSH 下发中文出现乱码，使用 UTF-8 base64：

```bash
python3 scripts/go2w_agent_entry.py --command-b64 "5Y676LW15Y2a5Yqe5YWs5a6k6Zeo5Y+j" --no-live-snapshot --execute --monitor-s 20 --pretty
```

真实两点拓扑的例子：

```bash
python3 scripts/go2w_agent_entry.py --command "去第二个点" --no-live-snapshot --pretty
python3 scripts/go2w_agent_entry.py --command "wp_1" --no-live-snapshot --pretty
```

## PCD点选标注

可以从机器狗拉取 `/home/unitree/test.pcd`，渲染成俯视图后人工点击目标点，再把点击点转换为 map 坐标写入真实拓扑。

```bash
python scripts/render_pcd_topdown.py artifacts/real_site_pcd/test.pcd --output-png artifacts/real_site_pcd/test_topdown.png --output-meta artifacts/real_site_pcd/test_topdown_meta.json
python scripts/make_pcd_annotation_page.py --meta artifacts/real_site_pcd/test_topdown_meta.json --image artifacts/real_site_pcd/test_topdown.png --output artifacts/real_site_pcd/annotate_test_pcd.html
```

打开 `artifacts/real_site_pcd/annotate_test_pcd.html` 后点击地图即可得到节点 JSON。新增点进入真实注册表前需要人工确认名字、用途和是否允许作为重定位锚点。

## 重定位锚点

重定位不必只用初始点。Unitree 重定位接口接受 `map_path` 和初始位姿，所以任何已知且可识别的真实 map 位姿都可以作为重定位初值。建议把重定位点和普通导航点区分管理：

- 普通导航点：用于 `navigate_to_pose`。
- 重定位锚点：用于 `relocate` 的初始位姿，应选择几何特征明显、遮挡少、机器人能稳定站定的位置。
- 一个点可以同时是导航点和重定位锚点，但必须标注允许半径和方向误差。

## 当前约束

- `--execute` 不加时默认只做 dry-run，不会移动。
- 网关安全门仍会实时检查 `slam_health`、`localization` 和 `safety.allow_navigation`。
- `--go` 现场入口默认使用 `--nav-speed-mps 0.3 --nav-mode 1`，保持低速、地形/保守运动模式，声音和步态更轻；如果要完全使用 registry 中每个点自己的速度和 mode，可传 `--nav-speed-mps 0 --nav-mode -1`。
- 当前 C++ 网关状态仍主要是短进程内存态，所以执行后的进度监控不要只依赖新开的 `get_world_state` 里的 `navigation` 字段，后续需要把导航任务状态持久化或改成长驻 agent。

## 语义链路摘要

`--go` 的 brief 输出和 JSON 日志会包含 `semantic_trace`，用于解释这次闭环到底如何从自然语言走到底层命令：

```text
command
-> requested_target_guess
-> target.name / target.node_id / target.distance_from_robot_m
-> planner.mode / planner.tools
-> slam_command.action / slam_command.target_pose
```

这部分是答辩和现场排障的关键证据：它能证明 LLM 没有凭空编坐标，而是先匹配 registry 中的语义拓扑点，再由 `plan_to_slam_command()` 转成 `navigate_to_pose`。

多目标中文命令现在会进入串行任务队列。例如“去 701 门外走廊拍照，然后回尹思园工位”会在 `semantic_trace.matched_targets` 中列出两个目标，并在 `planner.task_queue` / `queue_execution` 中展开为 `navigate -> wait_until -> capture_keyframe -> navigate -> wait_until -> report`。真实执行时每段导航前仍会重新做 gateway preflight；任一步失败会停止队列，不继续执行后续目标。

如果 Windows/PowerShell 或 SSH 对中文命令不稳定，先生成 UTF-8 base64 命令：

```bash
python3 scripts/go2w_encode_command.py --mode go "去701门外走廊拍照，然后回尹思园工位" --pretty
```

然后使用输出中的 `--go-b64` 命令现场执行。

## 源码打包

源码包使用：

```bash
python3 scripts/package_source_release.py --pretty
```

该脚本会打包 git 已跟踪和未忽略的新代码，并排除：

```text
artifacts/
models/
*.gguf
*.pcd
*.bag
venv/
build/
```

默认输出到当前用户桌面，包内会包含 `PACKAGE_MANIFEST.json`，记录分支、commit、dirty status 和排除规则。
