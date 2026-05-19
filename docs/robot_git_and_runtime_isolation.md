# 机器狗 Git 与运行目录隔离说明

更新时间：2026-05-19

## 目标

把“可版本管理的代码”和“机器狗本地运行状态”分开。

机器狗可以用自己的 SSH key 拉取和推送代码，但模型、构建产物、日志、临时输出这些本地文件不进入 git。

## Git 仓库位置

PC 端工作仓库：

```text
E:\GO2W_0
```

机器狗端 git clone：

```text
/home/unitree/Go2W_SLAM_AI
```

当前分支：

```text
agent/llm-on-robot
```

机器狗端 remote：

```text
git@github-go2w-robot:Yincloudhub/Go2W_SLAM_AI.git
```

## 机器狗运行目录

以下目录不属于 git 仓库：

```text
/home/unitree/llm_runtime
/home/unitree/models
/home/unitree/slam_gateway_refactor
```

职责划分：

- `/home/unitree/llm_runtime`：存放 llama.cpp 源码、build 产物和已安装的推理脚本。
- `/home/unitree/models`：存放 GGUF 模型文件。
- `/home/unitree/slam_gateway_refactor`：现有 Unitree SDK2 / SLAM 执行网关。

## 机器人侧可版本管理文件

机器人部署辅助文件统一放在：

```text
robot/
```

在机器狗上安装这些脚本：

```bash
cd /home/unitree/Go2W_SLAM_AI
bash robot/llm_runtime/install_runtime_files.sh
```

安装脚本会把：

```text
robot/llm_runtime/ask_qwen.sh
```

复制到：

```text
/home/unitree/llm_runtime/scripts/ask_qwen.sh
```

这样脚本由 git 管理，但实际运行位置和仓库隔离。

## 明确不进 Git 的内容

不要提交：

- `*.gguf`
- llama.cpp 的 `build/`
- `/home/unitree/models`
- `/home/unitree/llm_runtime/llama.cpp/build`
- 运行日志
- 评测运行产物
- 机器狗生成的点云、bag、db3、pcd、地图文件

顶层 `.gitignore` 已经排除了模型、构建输出、artifacts 和常见机器人运行时文件。

## 当前 LLM 文件状态

已完成：

- 机器狗上已有 llama.cpp CPU 版 `llama-cli`。
- `ask_qwen.sh` 已纳入 git。
- `ask_qwen.sh` 已安装到 `/home/unitree/llm_runtime/scripts`。
- GGUF 模型已上传到 `/home/unitree/models`。
- 模型 SHA256 校验通过。

待完成：

- 需要重新做一次干净的短推理 smoke test。
- CUDA 版 llama.cpp 还没有评估。
- C++ 层还没有完整接入 `slam_gateway_refactor`。

