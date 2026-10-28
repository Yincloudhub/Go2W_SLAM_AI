# 运行与部署

[返回项目首页](../README.md)

## 普通电脑：离线任务预览

需要 Python 3.10 或更新版本。从仓库根目录执行：

```bash
python go2w_edge_autonomy/scripts/run_llm_nav_local.py \
  --command "去 wp_1" \
  --snapshot-json examples/offline_snapshot.json \
  --dry-run --pretty
```

`offline_snapshot.json` 是合成状态，表示用于演示的定位正常状态。程序读取地图样例中的 `wp_1`，输出 `planner_input`、`mock_llm_plan` 和 `slam_command`。命令不采集机器人状态，也不向机器人发送指令。

查看地图与节点：

```bash
python go2w_edge_autonomy/scripts/map_registry_cli.py list
```

## 机器狗端：SLAM Gateway

需要机器狗 Linux 环境、Unitree SDK2，以及设备已有的 SLAM / LiDAR 服务。当前 C++ 工程使用 C++17 和 CycloneDDS 相关库，具体依赖路径由构建脚本与设备环境决定。

从仓库根目录构建：

```bash
cd slam_gateway_cpp_sdk
bash scripts/build_on_go2.sh -DUNITREE_SDK2_ROOT=/home/unitree/unitree_sdk2
cp build/slam_llm_command_client ../go2w_edge_autonomy/bin/slam_llm_command_client
```

进入边缘自治目录，启动设备已有的 SLAM 栈：

```bash
cd ../go2w_edge_autonomy
bash scripts/start_go2w_slam_stack.sh
```

先检查实时状态，再预览指令：

```bash
python3 scripts/slam_runtime_snapshot.py --local --pretty
python3 scripts/run_llm_nav_local.py --command "去 wp_1" --dry-run --pretty
```

地图样例包含原测试现场的 PCD 路径、节点和重定位锚点。实际部署前替换为当前设备和场地配置。配置正常后，运行以下命令会向设备发送任务：

```bash
python3 scripts/run_llm_nav_local.py \
  --command "去 wp_1" --network-interface eth0 --pretty
```

默认分支的 planner 为规则实现。Gateway 部分感知与健康反馈仍为早期实现，见[源码范围](source-status.md)。

## 雷达端：CPU 成像构建

在 Orin NX Linux 环境安装依赖：

```bash
sudo apt-get update
sudo apt-get install -y \
  build-essential cmake pkg-config \
  libeigen3-dev libfftw3-dev libmatio-dev libopencv-dev
```

从仓库根目录构建：

```bash
cd radar/fft_image_NX_live
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j4
```

产物为 `build/fft_imaging_cli` 和 `build/fft_imaging_live`。当前源码使用 CPU 后端，不需要 CUDA 编译环境。

### 原始数据回放

以下命令在 `radar/fft_image_NX_live/` 下执行，输入文件由使用者提供：

```bash
./build/fft_imaging_live \
  --config release/orin_nx_min/config/fft_imaging_orin.cfg \
  --source-bin /data/radar_raw.bin \
  --velocity-mps 1.0 \
  --max-live-frames 1
```

这里的 `1.0 m/s` 是回放示例值，应按实际采集运动修改。也可以使用 `--vfit-mat /data/velocity.mat`，文件中需包含代码读取的 `Vfit` 变量。ADC 排列、通道和采样参数应与实际数据及配置保持一致。

离线输出 JPG：

```bash
./build/fft_imaging_cli \
  --config release/orin_nx_min/config/fft_imaging_orin.cfg \
  /data/radar_raw.bin /data/velocity.mat /data/output_jpg
```

默认 `runtime.max_frames = 1`，可在配置中调整处理帧数。

### 实时雷达采集

硬件采集依赖外部 pyRadar、TI 雷达控制环境、`fpga_udp` 原生扩展、DCA1000 配置文件与匹配的雷达配置。以上第三方运行环境没有打包在仓库内。

原项目采集记录使用 `30f128_safe` 配置，但该配置文件未包含在提供的源码快照中。仓库附带的 `mmwaveconfig_0508_50f128_test.txt` 是测试配置，不能当作上述性能测试的同一配置。

以下为启动模板，在 `radar/fft_image_NX_live/` 下运行。将 `/path/to/pyRadar` 和配置路径替换为设备上的实际路径：

```bash
python3 scripts/live_dca1000_launcher.py \
  --pyradar-root /path/to/pyRadar \
  --radar-config /path/to/validated_radar_config.txt \
  --dca-config /path/to/pyRadar/configFiles/cf.json \
  --fft-config release/orin_nx_min/config/fft_imaging_orin.cfg \
  --fft-bin build/fft_imaging_live \
  --radar-interface eno1 \
  --radar-numframes 0 \
  --velocity-mps 1.0 \
  --metrics-dir ./outputs/metrics \
  --mjpeg-port 8090
```

采集脚本会请求设备管理所需的权限，并按参数配置雷达网口。网卡名称、IP、速度和控制配置均应与设备实际连接一致。运行后可访问设备的 `8090` 端口下 `/radar.mjpg` 查看图像，使用 `Ctrl+C` 停止采集。

## 双 Orin NX 的 ROS2 联调

整机使用以下接口：

```text
机器狗 → 雷达：/unitree/slam_mapping/odom
雷达 → 机器狗：/radar/data_status
雷达 → 机器狗：/radar/sar_image
```

当前仓库没有这些雷达桥接节点的构建包和 launch 文件。接入整机通信版本后，再检查两端 DDS 网络、ROS domain、消息类型、QoS 及时间戳配置，并按[架构文档](architecture.md)核对 Topic。

## 验证范围

项目**已经进行 Orin NX 实机测试**，测试结果见[性能与测试口径](benchmarks.md)。普通电脑可以验证 Python 离线任务转换；雷达原始数据回放需要匹配的输入文件，机器狗部署和真实雷达采集分别需要对应硬件与依赖。
