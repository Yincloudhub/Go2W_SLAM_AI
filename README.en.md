<p align="center"><img src="docs/assets/hero.svg" alt="Go2-W perception and localization with dual Orin NX devices" width="100%" /></p>

<h1 align="center">Go2-W Perception & Localization</h1>

<p align="center">Millimeter-wave radar perception, LiDAR SLAM and edge imaging for a wheeled quadruped</p>
<p align="center"><a href="README.md">中文</a> · <a href="docs/getting-started.md">Getting started</a> · <a href="docs/source-status.md">Source scope</a></p>

## Overview

This project connects a **Unitree Go2-W** robot controller and a radar perception subsystem running on **two NVIDIA Orin NX devices**. Robot odometry supplies motion information for sliding-window SAR imaging. Radar acquisition status and images return to the robot for monitoring, obstacle position analysis and assisted waypoint selection.

The robot uses Unitree's existing **LiDAR SLAM, relocalization and navigation** capabilities. Millimeter-wave radar supplies additional perception information.

**Public source scope:** this repository contains the C++ SLAM Gateway, a Python edge autonomy layer, and a CPU radar imaging snapshot. The ROS2 radar bridge, obstacle coordinate conversion and CUDA FFT implementation used in the integrated system are not included. See the [source map](docs/source-status.md) for details.

## Hardware and output

<table>
  <tr><th>Go2-W platform</th><th>Radar imaging output</th></tr>
  <tr>
    <td align="center"><img src="docs/assets/go2w-platform.png" alt="Go2-W robot with edge compute hardware" height="300" /></td>
    <td align="center"><img src="docs/assets/radar-imaging.gif" alt="Radar imaging output" width="430" /></td>
  </tr>
</table>

The images and animations come from project recordings. They illustrate the hardware and output rather than measuring localization accuracy or processing frame rate.

## Integrated system

| Component | Role |
| --- | --- |
| Robot-side Orin NX | SLAM state, odometry, waypoint management and navigation service access |
| Radar-side Orin NX | Radar acquisition, sliding-window imaging and result publication |
| SPI | Radar configuration and control |
| Python / UDP / TCP | Acquisition and data reception |
| C++ / SAR RDA | Range FFT, azimuth FFT and range/azimuth remapping |
| CUDA | FFT optimization in the integrated version |

| ROS2 topic | Direction | Purpose |
| --- | --- | --- |
| `/unitree/slam_mapping/odom` | Robot → radar | Motion information for imaging |
| `/radar/data_status` | Radar → robot | Acquisition status |
| `/radar/sar_image` | Radar → robot | Images for perception and assisted waypoint selection |

See the [architecture document](docs/architecture.md) for the full data flow.

## Reported measurements

These figures were supplied by the project maintainer for Orin NX system testing. FFT latency covers the **two FFT stages**, excluding remapping, rendering, transport and disk writes.

| Metric | Result |
| --- | ---: |
| Raw data blocks | 5,199 |
| Radar data volume | Approximately 10.90 GB |
| Data throughput | Approximately 121.29 Mbps |
| Recorded `bad_chirp` | 0 |
| Valid SAR images | 356 |
| Two-stage FFT before optimization | 290.33 ms/frame |
| Two-stage FFT after CUDA optimization | 81.23 ms/frame |
| FFT speedup | Approximately 3.57× |

<img src="docs/assets/fft-performance.svg" alt="FFT latency comparison: 290.33 versus 81.23 ms per frame" width="820" />

Raw measurement logs and CUDA source are not bundled. The public capture script records `bad_chunk`, which should not be treated as equivalent to `bad_chirp`. See [measurement scope](docs/benchmarks.md).

## Quick start

Clone the repository and preview a task using a synthetic robot snapshot:

```bash
git clone https://github.com/Yincloudhub/Go2W_SLAM_AI.git
cd Go2W_SLAM_AI
python go2w_edge_autonomy/scripts/run_llm_nav_local.py \
  --command "去 wp_1" \
  --snapshot-json examples/offline_snapshot.json \
  --dry-run --pretty
```

This requires Python 3.10 or later. It prints a plan and a SLAM command without connecting to a robot. The default branch uses a rule-based planner. Additional LLM integration work is maintained on the [`agent/llm-on-robot`](https://github.com/Yincloudhub/Go2W_SLAM_AI/tree/agent/llm-on-robot) branch.

Hardware build and deployment instructions are in [getting started](docs/getting-started.md). Module documentation:

- [C++ SLAM Gateway](slam_gateway_cpp_sdk/README.md)
- [Python edge autonomy](go2w_edge_autonomy/README.md)
- [Radar acquisition and CPU imaging](radar/fft_image_NX_live/README.md)

**Stack:** C++17, Python, ROS2, Linux, NVIDIA Orin NX, CUDA, SPI, UDP/TCP, SAR RDA.
