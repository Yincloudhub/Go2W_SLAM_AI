#!/usr/bin/env bash
set -euo pipefail

SRC_DIR="${GO2W_DEEPYOLO_SRC_DIR:-/home/unitree/librealsense/examples/DeepYolo_test}"
WORK_DIR="${GO2W_DEEPYOLO_HEADLESS_DIR:-${SRC_DIR}/go2w_headless}"
ENGINE_PATH="${GO2W_DEEPYOLO_ENGINE:-${SRC_DIR}/yolo11m.engine}"
OUTPUT_DIR="${GO2W_DEEPYOLO_OUTPUT_DIR:-${SRC_DIR}/output}"

if [ ! -f "${SRC_DIR}/main.cpp" ]; then
  echo "DeepYOLO source not found: ${SRC_DIR}/main.cpp" >&2
  exit 2
fi
if [ ! -f "${ENGINE_PATH}" ]; then
  echo "DeepYOLO TensorRT engine not found: ${ENGINE_PATH}" >&2
  exit 2
fi

mkdir -p "${WORK_DIR}" "${OUTPUT_DIR}"

SRC_DIR="${SRC_DIR}" WORK_DIR="${WORK_DIR}" ENGINE_PATH="${ENGINE_PATH}" OUTPUT_DIR="${OUTPUT_DIR}" python3 - <<'PY'
import os
from pathlib import Path

src_dir = Path(os.environ["SRC_DIR"])
work_dir = Path(os.environ["WORK_DIR"])
engine_path = os.environ["ENGINE_PATH"]
output_dir = os.environ["OUTPUT_DIR"]

text = (src_dir / "main.cpp").read_text(encoding="utf-8", errors="replace")
if "#include <cstdlib>" not in text:
    text = text.replace("#include <ctime>\n", "#include <ctime>\n#include <cstdlib>\n", 1)

capture_marker = """    for (int mode : {2, 1, 0}) {
        rs2::config cfg;"""
capture_replacement = """    const char* input_fps_env = std::getenv("GO2W_DEEPYOLO_INPUT_FPS");
    int input_fps = input_fps_env ? std::atoi(input_fps_env) : 15;
    if (input_fps <= 0) input_fps = 15;

    for (int mode : {2, 1, 0}) {
        rs2::config cfg;"""
if capture_marker not in text:
    raise SystemExit("failed to locate DeepYOLO RealSense capture config")
text = text.replace(capture_marker, capture_replacement, 1)
text = text.replace("RS2_FORMAT_BGR8, 30);", "RS2_FORMAT_BGR8, input_fps);")
text = text.replace("RS2_FORMAT_Z16, 30);", "RS2_FORMAT_Z16, input_fps);")
text = text.replace("RS2_FORMAT_Y8, 30);", "RS2_FORMAT_Y8, input_fps);")
text = text.replace(
    'std::cout << " 流启动成功，depth_scale=" << depth_scale << std::endl;',
    'std::cout << " 流启动成功，depth_scale=" << depth_scale'
    ' << " input_fps=" << input_fps << std::endl;',
    1,
)
capture_loop_marker = """    rs2::align align_to_color(RS2_STREAM_COLOR);

    std::cout << "[RealSense] RGBD";"""
capture_loop_replacement = """    rs2::align align_to_color(RS2_STREAM_COLOR);
    const char* capture_every_n_env = std::getenv("GO2W_DEEPYOLO_CAPTURE_EVERY_N");
    int capture_every_n = capture_every_n_env ? std::atoi(capture_every_n_env) : 3;
    if (capture_every_n <= 0) capture_every_n = 3;
    uint64_t capture_frame_count = 0;

    std::cout << "[GO2W] capture_every_n=" << capture_every_n << std::endl;
    std::cout << "[RealSense] RGBD";"""
if capture_loop_marker not in text:
    raise SystemExit("failed to locate DeepYOLO capture loop setup")
text = text.replace(capture_loop_marker, capture_loop_replacement, 1)
capture_wait_marker = """            raw_frames = pipe.wait_for_frames();
            aligned_frames = align_to_color.process(raw_frames);"""
capture_wait_replacement = """            raw_frames = pipe.wait_for_frames();
            capture_frame_count++;
            if (capture_every_n > 1 && capture_frame_count % capture_every_n != 0) {
                continue;
            }
            aligned_frames = align_to_color.process(raw_frames);"""
if capture_wait_marker not in text:
    raise SystemExit("failed to locate DeepYOLO capture frame wait")
text = text.replace(capture_wait_marker, capture_wait_replacement, 1)

old_engine = 'std::string engine_path = "../yolo11m.engine";'
new_engine = (
    'const char* engine_env = std::getenv("GO2W_DEEPYOLO_ENGINE");\n'
    f'    std::string engine_path = engine_env ? engine_env : "{engine_path}";'
)
text = text.replace(old_engine, new_engine, 1)

old_output = 'std::string semantic_path = "../output/semantic_stream_" + session_id + ".jsonl";'
new_output = (
    'const char* output_env = std::getenv("GO2W_DEEPYOLO_OUTPUT_DIR");\n'
    f'    std::string semantic_dir = output_env ? output_env : "{output_dir}";\n'
    '    std::string semantic_path = semantic_dir + "/semantic_stream_" + session_id + ".jsonl";'
)
text = text.replace(old_output, new_output, 1)

inference_marker = """    // ----------------------------------------
    // 3. 推理主循环
    // ----------------------------------------
    while (is_running) {
        {
            std::lock_guard<std::mutex> lock(frame_mutex);"""
inference_replacement = """    // ----------------------------------------
    // 3. 推理主循环
    // ----------------------------------------
    const char* inference_interval_env = std::getenv("GO2W_DEEPYOLO_INFERENCE_INTERVAL_MS");
    int inference_interval_ms = inference_interval_env ? std::atoi(inference_interval_env) : 200;
    if (inference_interval_ms < 0) inference_interval_ms = 200;
    auto next_inference_time = std::chrono::steady_clock::now();
    std::cout << "[GO2W] inference_interval_ms=" << inference_interval_ms << std::endl;

    while (is_running) {
        if (inference_interval_ms > 0) {
            std::this_thread::sleep_until(next_inference_time);
            next_inference_time = std::chrono::steady_clock::now()
                + std::chrono::milliseconds(inference_interval_ms);
        }
        {
            std::lock_guard<std::mutex> lock(frame_mutex);"""
if inference_marker not in text:
    raise SystemExit("failed to locate DeepYOLO inference loop")
text = text.replace(inference_marker, inference_replacement, 1)

for needle in ("cv::namedWindow", "cv::resizeWindow"):
    lines = []
    for line in text.splitlines():
        if needle in line:
            lines.append("// GO2W headless: " + line)
        else:
            lines.append(line)
    text = "\n".join(lines) + "\n"

start = text.find('        cv::imshow("YOLO RGB", frame);')
end_marker = "        int key = cv::waitKey(1);\n        if (key == 27 || key == 'q' || key == 'Q') {\n            is_running = false;\n        }"
end = text.find(end_marker, start)
if start < 0 or end < 0:
    raise SystemExit("failed to locate DeepYOLO GUI block for headless patch")
replacement = '''        const char* max_frames_env = std::getenv("GO2W_DEEPYOLO_MAX_FRAMES");
        if (max_frames_env) {
            int max_frames = std::atoi(max_frames_env);
            if (max_frames > 0 && perf_frame_count >= max_frames) {
                is_running = false;
            }
        }'''
text = text[:start] + replacement + text[end + len(end_marker):]

(work_dir / "main_headless.cpp").write_text(text, encoding="utf-8")
(work_dir / "CMakeLists.txt").write_text(f"""cmake_minimum_required(VERSION 3.10)
project(Go2YoloRealsenseHeadless)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

find_package(OpenCV REQUIRED)
find_package(CUDA REQUIRED)
find_package(Threads REQUIRED)
find_package(realsense2 REQUIRED)

include_directories(
    ${{OpenCV_INCLUDE_DIRS}}
    ${{CUDA_INCLUDE_DIRS}}
    /usr/include/aarch64-linux-gnu
    /usr/local/cuda/include
    {src_dir.parent.parent}/common
    {src_dir.parent.parent}/third-party/imgui
    {src_dir}
)

link_directories(
    /usr/lib/aarch64-linux-gnu
    /usr/local/cuda/lib64
)

add_executable(yolo_test_realsense_headless main_headless.cpp)
target_compile_options(yolo_test_realsense_headless PRIVATE -O3 -Wall -Wextra)
target_link_libraries(yolo_test_realsense_headless
    PRIVATE
    ${{OpenCV_LIBS}}
    ${{CUDA_LIBRARIES}}
    ${{realsense2_LIBRARY}}
    nvinfer
    cudart
    Threads::Threads
)
""", encoding="utf-8")
print(work_dir)
PY

cmake -S "${WORK_DIR}" -B "${WORK_DIR}/build"
cmake --build "${WORK_DIR}/build" -j2
echo "headless_binary=${WORK_DIR}/build/yolo_test_realsense_headless"
