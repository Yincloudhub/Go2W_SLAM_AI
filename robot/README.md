# 机器狗侧文件说明

`robot/` 目录只放适合版本管理的机器狗部署辅助文件。

这里不放：

- GGUF 模型权重。
- llama.cpp build 产物。
- 运行日志。
- 临时推理输出。
- 机器狗生成的地图、点云、bag、db3。

## 机器狗上的实际目录

git clone 位于：

```text
/home/unitree/Go2W_SLAM_AI
```

运行时文件安装在仓库外部：

```text
/home/unitree/llm_runtime
/home/unitree/models
/home/unitree/slam_gateway_refactor
```

这样可以把 PC 端工作仓库、机器狗端 git clone、模型目录和可执行运行时隔离开。

## 当前受 Git 管理的机器狗文件

```text
robot/
  README.md
  llm_runtime/
    ask_qwen.sh
    install_runtime_files.sh
```

`ask_qwen.sh` 在这里版本管理，然后安装到：

```text
/home/unitree/llm_runtime/scripts/ask_qwen.sh
```

安装命令：

```bash
cd /home/unitree/Go2W_SLAM_AI
bash robot/llm_runtime/install_runtime_files.sh
```

## 外部模型文件

模型不进入 git。

机器狗上的模型路径：

```text
/home/unitree/models/Qwen_Qwen3-4B-Instruct-2507-Q4_K_M.gguf
```

期望 SHA256：

```text
2fde00ce69dd4899c70d020845e2638353015bba0fdf161b3eb965f2bca4464e
```

