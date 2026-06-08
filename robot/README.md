# 机器狗侧版本化文件

`robot/` 保存适合 Git 管理、需要部署到机器狗的源码和脚本：

```text
robot/
  llm_runtime/
  slam_gateway_refactor/
```

## Gateway

Unitree SLAM gateway 的唯一源码位于：

```text
/home/unitree/Go2W_SLAM_AI/robot/slam_gateway_refactor
```

构建目录：

```text
/home/unitree/Go2W_SLAM_AI/robot/slam_gateway_refactor/build
```

旧目录 `/home/unitree/slam_gateway_refactor` 不再是运行模块，应归档。

## LLM runtime

版本化推理脚本位于 `robot/llm_runtime/`，安装到仓库外的
`/home/unitree/llm_runtime`。模型权重保存在 `/home/unitree/models`。

## 不进入 Git 的内容

- GGUF 模型权重
- CMake 和 llama.cpp 构建产物
- 运行日志与传感器摘要
- PCD、bag、db3 和 Unitree topology runtime 文件
