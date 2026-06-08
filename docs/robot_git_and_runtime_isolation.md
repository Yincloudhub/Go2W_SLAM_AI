# 机器狗 Git 与运行目录隔离

## Git 仓库

PC 工作区：

```text
E:\GO2W_0
```

机器狗唯一 Git 工作区：

```text
/home/unitree/Go2W_SLAM_AI
```

当前开发分支：

```text
agent/llm-on-robot
```

Unitree gateway 源码也在主仓库中维护：

```text
/home/unitree/Go2W_SLAM_AI/robot/slam_gateway_refactor
```

不要再从 `/home/unitree/go2w_slam_agent` 或
`/home/unitree/slam_gateway_refactor` 启动。旧目录只允许归档到
`/home/unitree/_archive/`。

## 仓库外运行数据

以下内容不进入 Git：

```text
/home/unitree/llm_runtime
/home/unitree/models
/home/unitree/test.pcd
/home/unitree/topology_points.json
/home/unitree/Go2W_SLAM_AI/artifacts
```

- `llm_runtime` 保存 llama.cpp 构建产物和安装后的推理脚本。
- `models` 保存 GGUF 权重。
- `test.pcd` 和 `topology_points.json` 是 Unitree SLAM 运行文件。
- `artifacts` 保存日志、图像和传感器摘要。

## LLM 安装

版本化脚本位于：

```text
robot/llm_runtime/ask_qwen.sh
```

安装到仓库外运行目录：

```bash
cd /home/unitree/Go2W_SLAM_AI
bash robot/llm_runtime/install_runtime_files.sh
```

模型文件：

```text
/home/unitree/models/Qwen_Qwen3-4B-Instruct-2507-Q4_K_M.gguf
```

## 同步原则

1. PC 和机器狗在同一 Git 分支、同一提交上工作。
2. 源码修改先提交和推送，再让机器狗拉取；现场临时同步只用于编译验证。
3. 地图 registry 由 Git 管理；PCD 和 Unitree topology runtime 文件不进 Git。
4. 运行入口只使用主仓库内的 launcher 和 gateway 二进制。
5. 每次同步后运行 `python3 scripts/check_go2w_runtime_layout.py`。
