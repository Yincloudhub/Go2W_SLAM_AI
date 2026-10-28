# 雷达采集与边缘成像

[返回项目首页](../../README.md) · [部署指南](../../docs/getting-started.md) · [源码范围](../../docs/source-status.md)

本模块来自项目提供的 `fft_image_NX_live` 源码快照，面向 TI AWR2243 / DCA1000 与 Jetson Orin NX，包含 C++ CPU 成像、流式回放、Python 采集启动、视频输出和 MJPEG 显示。

## 公开实现

- 原始雷达 bin 读取与 DCA1000 流式输入。
- 滑窗回波环形缓冲、距离向 FFT、方位向 FFT、距离／方位重映射。
- 固定运动速度或 `Vfit` MAT 文件输入。
- JPG、OpenCV 视频与 MJPEG HTTP 输出。
- 采集、分阶段耗时与资源占用日志。

当前后端使用 FFTW3 / Eigen。整机项目的 CUDA FFT、ROS2 里程计订阅及雷达 Topic 发布不包含在此快照中。

## 目录

| 路径 | 用途 |
| --- | --- |
| `app/fft_imaging_cli.cpp` | 离线成像与 JPG 输出 |
| `app/fft_imaging_live.cpp` | 回放、流式成像、视频与 MJPEG |
| `include/fft_imaging/` | 配置、数据源、速度提供器与成像接口 |
| `src/core_pipeline.cpp` | 两级 FFT 主链路 |
| `src/realtime_stream.cpp` | 数据流解析与环形缓冲 |
| `src/range_angle_imager.cpp` | 距离／方位重映射 |
| `scripts/live_dca1000_launcher.py` | pyRadar 配置、接收与成像进程桥接 |
| `scripts/live_radar_camera_launcher.py` | 雷达与相机联合录制入口 |
| `release/orin_nx_min/config/` | 成像运行配置 |

## Linux 构建

依赖：C++17、CMake、Eigen3、FFTW3、MATIO、OpenCV。

在本模块目录执行：

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j4
```

回放输入文件并生成一帧：

```bash
./build/fft_imaging_live \
  --config release/orin_nx_min/config/fft_imaging_orin.cfg \
  --source-bin /data/radar_raw.bin \
  --velocity-mps 1.0 \
  --max-live-frames 1
```

完整设备配置、硬件采集依赖与启动参数见[部署指南](../../docs/getting-started.md)。成像参数必须与实际 ADC 数据匹配，示例速度应替换为采集时的运动速度。

## 日志口径

采集启动器生成 `raw_chunks.csv`、`fft_live_frames.csv` 和 `resource_usage.csv`。`bad_chunk` 是数据块字段，与整机测试汇总中的 `bad_chirp` 不同。FFT、重映射、渲染、编码与写盘应分别计时，详见[性能说明](../../docs/benchmarks.md)。
