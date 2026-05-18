# go2w_edge_autonomy

GO2W SLAM/LLM 边缘自治层。该目录负责在机器狗本机完成状态采集、地图/拓扑点管理、planner input 构造、自然语言指令到导航命令的转换，并调用 C++ SLAM gateway 组件执行命令。

## 目录结构

```text
go2w_edge_autonomy/
  bin/
    slam_llm_command_client          C++ gateway 编译产物，Python 执行器会调用它

  configs/
    maps/
      go2w_map_registry.example.json 地图、PCD、拓扑点、重定位锚点配置

  edge_autonomy/
    models.py                        基础数据模型
    map_registry.py                  地图集、拓扑节点、重定位锚点管理
    runtime_state.py                 本机/远程运行状态解析
    slam_topics.py                   /slam_info、/slam_key_info、ctrl_info 解析
    slam_state.py                    SLAM 位姿、健康、导航任务状态模型
    slam_health_monitor.py           根据位姿更新时间估计定位/SLAM 健康状态
    slam_adapter.py                  离线 replay / 内存适配器
    llm_context.py                   planner input 构造、模拟 planner、命令转换
    slam_gateway_executor.py         启动并调用 bin/slam_llm_command_client

  schemas/
    local_llm_plan.schema.json       planner JSON 结构约束
    map_registry.schema.json         地图配置结构约束

  scripts/
    start_go2w_slam_stack.sh         启动 xt16_driver 和 unitree_slam
    run_llm_nav_local.py             主入口：自然语言 -> planner -> C++ gateway
    slam_runtime_snapshot.py         采集 SLAM/LiDAR/进程状态
    map_registry_cli.py              查看地图、生成 relocate/nav JSON

  logs/
    slam_stack/                      start_go2w_slam_stack.sh 默认日志目录，运行后生成

  requirements.txt                   Python 依赖
```

## 推荐启动流程

在机器狗本机进入本目录：

```bash
cd go2w_edge_autonomy
```

先启动底层雷达和 SLAM：

```bash
./scripts/start_go2w_slam_stack.sh
```

再下发自然语言导航指令：

```bash
python3 scripts/run_llm_nav_local.py --command "去 wp_1" --network-interface eth0 --pretty
```

只查看 planner 会生成什么，不真正发给 C++ gateway：

```bash
python3 scripts/run_llm_nav_local.py --command "去 wp_1" --dry-run --pretty
```

## 脚本说明

### start_go2w_slam_stack.sh

启动 GO2W SLAM 底层运行栈：

```text
xt16_driver
unitree_slam
```

该脚本不会启动：

```text
keyDemo
slam_keyboard_client
rviz2
slam_llm_command_client
```

其中 `slam_llm_command_client` 由 `run_llm_nav_local.py` 按需启动。

默认日志目录：

```text
logs/slam_stack/
```

查看日志：

```bash
tail -f logs/slam_stack/xt16_driver.log
tail -f logs/slam_stack/unitree_slam.log
```

可覆盖环境变量：

```bash
UNITREE_SLAM_DIR=/unitree/module/unitree_slam/bin \
CYCLONEDDS_CONFIG=/unitree/module/unitree_slam/config/cyclonedds.xml \
GO2W_SLAM_LOG_DIR=./logs/slam_stack \
GO2W_SLAM_STARTUP_WAIT_S=8 \
./scripts/start_go2w_slam_stack.sh
```

常用含义：

```text
UNITREE_SLAM_DIR           unitree_slam 和 xt16_driver 所在目录
CYCLONEDDS_CONFIG          CycloneDDS 配置文件
GO2W_SLAM_LOG_DIR          日志输出目录
GO2W_SLAM_STARTUP_WAIT_S   等待进程启动的秒数
```

### run_llm_nav_local.py

本机自然语言导航主入口。当前还没有接真实 LLM，使用 `simulate_local_llm_plan()` 做规则模拟；后续接入真实 LLM 时，主要替换 planner 生成部分。

运行链路：

```text
自然语言 command
  -> 本机 runtime snapshot
  -> build_planner_context()
  -> simulate_local_llm_plan()
  -> plan_to_slam_command()
  -> bin/slam_llm_command_client
  -> Unitree SLAM API
```

示例：

```bash
python3 scripts/run_llm_nav_local.py --command "去 wp_1" --network-interface eth0 --pretty
```

使用已有 snapshot JSON 调试：

```bash
python3 scripts/run_llm_nav_local.py \
  --command "回到起点" \
  --snapshot-json /path/to/snapshot.json \
  --dry-run \
  --pretty
```

指定 C++ gateway 路径：

```bash
python3 scripts/run_llm_nav_local.py \
  --command "去 wp_1" \
  --gateway ./bin/slam_llm_command_client \
  --network-interface eth0
```

### slam_runtime_snapshot.py

采集当前机器狗运行状态，包括进程、LiDAR、点云摘要、重定位 odom、SLAM 文本 topic。

本机采集：

```bash
python3 scripts/slam_runtime_snapshot.py --local --pretty
```

远程 SSH 采集：

```bash
python3 scripts/slam_runtime_snapshot.py \
  --host 192.168.123.18 \
  --username unitree \
  --password "你的密码" \
  --pretty
```

### map_registry_cli.py

查看地图配置和生成可发送给 C++ gateway 的 JSON 命令。

列出地图、锚点、拓扑节点：

```bash
python3 scripts/map_registry_cli.py list
```

查看某张地图：

```bash
python3 scripts/map_registry_cli.py show --map-id test_current_main
```

生成重定位命令：

```bash
python3 scripts/map_registry_cli.py relocate \
  --map-id test_current_main \
  --anchor-id mapping_origin \
  --pretty
```

生成导航到拓扑点命令：

```bash
python3 scripts/map_registry_cli.py navigate-node \
  --map-id test_current_main \
  --node-id nie_guoli_office_front \
  --pretty
```

## 当前 LLM 接入状态

当前版本还没有调用真实 LLM。自然语言指令通过 `--command` 进入 `run_llm_nav_local.py`，再由本地规则函数 `simulate_local_llm_plan()` 模拟 planner 输出。

后续接真实 LLM 时，建议替换：

```python
plan = simulate_local_llm_plan(context, registry)
```

为：

```python
plan = call_real_llm(context)
```

后面的命令转换和执行逻辑可以继续复用：

```python
slam_command = plan_to_slam_command(plan, registry)
executor.send_command(slam_command)
```

## Python 依赖

安装依赖：

```bash
pip3 install -r requirements.txt
```

当前主要第三方依赖：

```text
paramiko    远程 SSH snapshot 采集使用；本机主链路基本使用标准库
```

