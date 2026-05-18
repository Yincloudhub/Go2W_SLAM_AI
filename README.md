# Go2W_SLAM_AI

本项目的目标是开发一个由 AI 调用机器狗 SLAM 服务的导航系统。

系统将自然语言任务转换为结构化导航计划，再通过 C++ SLAM gateway 调用 Unitree GO2W 的官方 SLAM / LiDAR / 导航能力，实现“AI 下达任务，机器狗执行 SLAM 导航”的闭环。

## 项目目标

```text
自然语言指令
  -> AI / planner
  -> 结构化 planner JSON
  -> 本地安全校验与地图拓扑查询
  -> C++ SLAM gateway JSON 命令
  -> Unitree SLAM 服务
  -> 机器狗执行重定位、导航、暂停、恢复等动作
```

当前版本还没有接入真实 LLM，`go2w_edge_autonomy` 中使用规则函数模拟 planner 输出。后续接入真实 LLM 时，主要替换 planner 生成部分。

## 目录结构

```text
go2w_slam/
  README.md

  go2w_edge_autonomy/
    Python 边缘自治层。负责自然语言入口、状态采集、地图/拓扑点管理、
    planner input 构造、planner plan 到 SLAM 命令的转换，并调用 C++ gateway。

  slam_gateway_cpp_sdk/
    C++ SLAM gateway 源码工程。负责对接 Unitree SDK2、订阅 SLAM topic、
    调用 Unitree SLAM API，并提供 slam_llm_command_client 可执行文件。
```

## 核心组件

### go2w_edge_autonomy

运行时主目录。主要内容：

```text
bin/slam_llm_command_client
  C++ gateway 编译产物，Python 执行器会调用它。

configs/maps/go2w_map_registry.example.json
  地图、PCD 路径、拓扑节点、重定位锚点配置。

scripts/start_go2w_slam_stack.sh
  启动 xt16_driver 和 unitree_slam。

scripts/run_llm_nav_local.py
  自然语言导航主入口。

edge_autonomy/
  Python 核心源码包。
```

### slam_gateway_cpp_sdk

C++ 源码目录。主要输出：

```text
build/slam_llm_command_client
  给 AI / planner 使用的结构化 JSON 命令入口。

build/slam_keyboard_client
  保留手动键盘调试入口。
```

重新编译后，需要将新的：

```text
slam_gateway_cpp_sdk/build/slam_llm_command_client
```

复制到：

```text
go2w_edge_autonomy/bin/slam_llm_command_client
```

## 推荐运行流程

在机器狗本机进入边缘自治目录：

```bash
cd go2w_edge_autonomy
```

启动底层 SLAM 栈：

```bash
./scripts/start_go2w_slam_stack.sh
```

查看日志：

```bash
tail -f logs/slam_stack/xt16_driver.log
tail -f logs/slam_stack/unitree_slam.log
```

下发自然语言导航指令：

```bash
python3 scripts/run_llm_nav_local.py --command "去 wp_1" --network-interface eth0 --pretty
```

只查看将生成的 planner 和 SLAM 命令，不真正执行：

```bash
python3 scripts/run_llm_nav_local.py --command "去 wp_1" --dry-run --pretty
```

## C++ Gateway 编译

在机器狗本机或具备 Unitree SDK2 的环境中：

```bash
cd slam_gateway_cpp_sdk
./scripts/build_on_go2.sh
```

如果 Unitree SDK2 不在默认路径：

```bash
./scripts/build_on_go2.sh -DUNITREE_SDK2_ROOT=/home/unitree/unitree_sdk2
```

更新运行组件：

```bash
cp -f build/slam_llm_command_client ../go2w_edge_autonomy/bin/slam_llm_command_client
```

## 团队协作建议

建议按职责分工：

```text
slam_gateway_cpp_sdk/
  C++ / Unitree SDK / SLAM API 适配

go2w_edge_autonomy/edge_autonomy/
  Python planner、状态解析、命令转换

go2w_edge_autonomy/configs/
  地图、拓扑点、重定位锚点配置

docs / README
  使用说明、现场部署流程、调试记录
```

建议分支模型：

```text
main
  稳定版本，可部署到机器狗。

dev
  日常集成分支。

feature/xxx
  单个功能开发分支。

fix/xxx
  bug 修复分支。
```

建议提交信息格式：

```text
feat: add local slam stack startup script
fix: prevent gateway destructor from stopping slam
docs: update edge autonomy readme
config: update map registry
refactor: reorganize edge autonomy package
```

## 版本管理注意事项

建议提交到 Git：

```text
源码
脚本
README / 文档
schema
配置样例
```

不建议提交到 Git：

```text
build/
logs/
__pycache__/
*.log
*.pid
*.pcd
*.bag
go2w_edge_autonomy/bin/slam_llm_command_client
```

二进制文件建议通过 GitHub Release 或现场部署脚本管理，而不是长期提交到源码仓库。

