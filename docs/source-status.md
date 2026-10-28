# 整机能力与源码范围

[返回项目首页](../README.md)

## 源码组成

本次整理以仓库默认 `master` 分支的机器狗代码为基础，并加入维护者提供的 `fft_image_NX_live` 雷达源码快照。保留原有功能模块路径，避免影响已有部署方式。

雷达模块从本地源码压缩包整理而来，移除了构建产物与 Python 缓存。C++ 与 Python 实现保留原样，部署说明按当前仓库路径重新编写。

## 能力对应表

| 能力 | 当前公开内容 | 说明 |
| --- | --- | --- |
| 宇树 SLAM 服务适配 | `slam_gateway_cpp_sdk/` | 建图、重定位、航点、暂停与恢复接口 |
| 地图与拓扑节点管理 | `go2w_edge_autonomy/` | 地图配置、锚点、状态采集与任务转换 |
| 自然语言任务入口 | `run_llm_nav_local.py` | 默认分支使用规则 planner |
| 雷达原始数据解析 | `radar/fft_image_NX_live/` | 原始 bin、流式输入与环形缓冲 |
| CPU FFT 与重映射 | `src/core_pipeline.cpp` 等 | FFTW3 / Eigen 实现 |
| 雷达采集启动 | `scripts/live_dca1000_launcher.py` | 依赖外部 pyRadar、原生接收扩展与设备配置 |
| 运动参数输入 | `FrameVelocityProvider` | 固定速度或 `Vfit` 文件 |
| 双设备 ROS2 雷达 Topic | 整机接口说明 | 通信节点源码未收录 |
| CUDA FFT 优化 | 整机测试汇总 | CUDA FFT 源码未收录 |
| 障碍物坐标转换与辅助航点 | 整机功能说明 | 对应模块及传感器外参未收录 |

## 阅读代码时需要区分的状态

### 雷达 CUDA 后端

[`src/backend.cpp`](../radar/fft_image_NX_live/src/backend.cpp) 中的 CUDA 分支会抛出 `CUDA backend is not implemented yet.`。当前公开版本应使用 `cpu_serial`；首页展示的 CUDA 优化结果来自维护者提供的整机测试汇总。

### 实时速度来源

[`app/fft_imaging_live.cpp`](../radar/fft_image_NX_live/app/fft_imaging_live.cpp) 读取固定速度或 `Vfit`，没有订阅 `/unitree/slam_mapping/odom`。整机 ROS2 接口说明不能直接当作当前快照的运行指令。

### 障碍摘要与健康状态

机器狗 Gateway 中的 `LidarGeometryPerception` 是早期占位实现，部分健康状态也使用初始假设。它们尚未形成可由本仓库独立复现的雷达辅助避障闭环。真实感知反馈应接入后再用于执行判断。

### LLM 分支

默认分支的 Python 任务入口调用 `simulate_local_llm_plan()`。[`agent/llm-on-robot`](https://github.com/Yincloudhub/Go2W_SLAM_AI/tree/agent/llm-on-robot) 保存了进一步的 LLM 集成开发，目录结构与默认分支不同。

## 复现整机结果需要补充的资产

- ROS2 雷达通信节点、消息定义或消息类型说明，以及启动配置。
- CUDA FFT 源码、编译选项和对应测试环境。
- 雷达安装外参、运动参数与回波的时间对齐方法。
- 障碍物位置提取、坐标转换和辅助航点模块。
- 与首页数据对应的原始采集日志、性能 CSV 和测试参数。

以上清单说明整机复现所需内容，不影响阅读与构建当前已公开的模块。
