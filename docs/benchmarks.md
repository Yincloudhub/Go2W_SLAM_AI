# 性能与测试口径

[返回项目首页](../README.md)

## 系统测试汇总

以下数据由项目维护者提供，测试平台为 NVIDIA Orin NX。它们描述整机集成版本的测试结果。当前仓库不包含该次原始 CSV、CUDA FFT 版本或完整的测试环境快照。

| 项目 | 汇总值 | 统计范围 |
| --- | ---: | --- |
| 原始数据块 | 5,199 个 | 单次采集测试累计值 |
| 雷达数据量 | 约 10.90 GB | 该次采集累计数据量 |
| 数据吞吐率 | 约 121.29 Mbps | 维护者提供的采集吞吐统计 |
| `bad_chirp` | 0 | 维护者提供的采集记录字段 |
| 有效 SAR 图像 | 356 帧 | 该次成像输出统计 |
| 优化前 FFT | 290.33 ms/帧 | 距离向与方位向 FFT 平均耗时之和 |
| CUDA 优化后 FFT | 81.23 ms/帧 | 相同阶段的优化后平均耗时 |

由以上 FFT 数据计算：

```text
FFT 阶段加速比 = 290.33 / 81.23 ≈ 3.57
FFT 阶段耗时下降 = (290.33 - 81.23) / 290.33 ≈ 72.02%
```

<img src="assets/fft-performance.svg" alt="两级 FFT 阶段平均耗时比较" width="820" />

## FFT 耗时与完整链路耗时

| 阶段 | 包含内容 |
| --- | --- |
| 两级 FFT | 距离向 FFT 与方位向 FFT |
| 成像计算 | FFT 及距离／方位重映射等计算 |
| 输出处理 | 伪彩色渲染、编码、图像或视频写盘 |
| 跨设备交付 | ROS2 发布、传输与订阅端接收 |

首页的 **81.23 ms/帧仅对应 FFT 阶段**，不能直接换算为系统端到端成像帧率。`--video-fps` 是视频播放参数，也不能用作处理吞吐率。

## 数据质量字段

维护者提供的测试汇总使用 `bad_chirp`。当前公开采集脚本的 CSV 字段为 `bad_chunk`，对应采集数据块，二者统计对象不同，不能互换。

`bad_chirp = 0` 也不能单独推出 UDP 零丢包。公开脚本的某些 pyRadar 接收模式下，`received_packets`、`lost_packets`、`loss_rate` 可能为 `-1`，需要结合接收器日志判断。

## 公开模块可记录的字段

[`live_dca1000_launcher.py`](../radar/fft_image_NX_live/scripts/live_dca1000_launcher.py) 与 [`fft_imaging_live.cpp`](../radar/fft_image_NX_live/app/fft_imaging_live.cpp) 包含以下日志输出：

| 文件 | 代表字段 | 用途 |
| --- | --- | --- |
| `raw_chunks.csv` | `written_bytes`、`raw_write_mbps`、`raw_cumulative_mbps`、`bad_chunk` | 采集量、吞吐与异常块统计 |
| `fft_live_frames.csv` | `range_fft_ms`、`azimuth_fft_ms`、`remap_ms`、`render_ms`、`mjpeg_encode_ms`、`video_write_ms` | 分阶段性能分析 |
| `resource_usage.csv` | CPU 使用率、进程 RSS | 边缘计算资源占用 |

重新采集性能结果时，应同时保存设备功耗模式、JetPack/CUDA 版本、代码提交、雷达配置、窗口长度、滑窗步长、输入速度来源与输出开关，使不同测试能够比较。

## 尚未提供的量化结果

现有材料没有提供定位误差、障碍物距离误差、避障成功率、完整链路延迟分位数或长时间运行的统计结果。项目首页因此只展示已有采集与 FFT 性能数据。
