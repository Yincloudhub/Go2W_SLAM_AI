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
for header in (
    "#include <algorithm>",
    "#include <cmath>",
    "#include <cstdio>",
    "#include <iomanip>",
    "#include <limits>",
):
    if header not in text:
        text = text.replace("#include <cstdlib>\n", "#include <cstdlib>\n" + header + "\n", 1)

helper_code = r'''

static uint64_t go2wEpochMs() {
    return static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::system_clock::now().time_since_epoch()).count());
}

static std::string go2wJsonNumberOrNull(double value) {
    if (!std::isfinite(value) || value <= 0.0) return "null";
    std::ostringstream out;
    out << std::fixed << std::setprecision(3) << value;
    return out.str();
}

static double go2wPercentile(std::vector<float>& values, double q) {
    if (values.empty()) return std::numeric_limits<double>::quiet_NaN();
    std::sort(values.begin(), values.end());
    const double pos = std::max(0.0, std::min(100.0, q)) * 0.01 * (values.size() - 1);
    const size_t lo = static_cast<size_t>(pos);
    const size_t hi = std::min(lo + 1, values.size() - 1);
    const double frac = pos - lo;
    return values[lo] * (1.0 - frac) + values[hi] * frac;
}

static void go2wWriteDepthPacket(
    const rs2::depth_frame& depth,
    const std::string& output_path,
    uint64_t captured_at_ms
) {
    if (!depth || output_path.empty()) return;
    const int width = depth.get_width();
    const int height = depth.get_height();
    const int y0 = height / 3;
    const int y1 = (height * 2) / 3;
    const int sample_step = 4;
    std::vector<float> left;
    std::vector<float> front;
    std::vector<float> right;
    size_t valid_total = 0;
    size_t sampled_total = 0;
    for (int y = y0; y < y1; y += sample_step) {
        for (int x = 0; x < width; x += sample_step) {
            const float distance_m = depth.get_distance(x, y);
            sampled_total++;
            if (distance_m < 0.15f || distance_m > 8.0f) continue;
            valid_total++;
            if (x < width / 3) left.push_back(distance_m);
            else if (x < (width * 2) / 3) front.push_back(distance_m);
            else right.push_back(distance_m);
        }
    }
    const double center_m = depth.get_distance(width / 2, height / 2);
    const double front_m = go2wPercentile(front, 10.0);
    const double left_m = go2wPercentile(left, 10.0);
    const double right_m = go2wPercentile(right, 10.0);
    const double confidence = sampled_total > 0
        ? static_cast<double>(valid_total) / static_cast<double>(sampled_total)
        : 0.0;
    const size_t sector_total = std::max<size_t>(1, sampled_total / 3);
    std::ostringstream json;
    json << "{"
         << "\"source\":\"d435_capture_owner\","
         << "\"timestamp_ms\":" << captured_at_ms << ","
         << "\"captured_at_ms\":" << captured_at_ms << ","
         << "\"frame_sequence\":" << depth.get_frame_number() << ","
         << "\"sensor_timestamp_ms\":" << std::fixed << std::setprecision(3) << depth.get_timestamp() << ","
         << "\"sensor_timestamp_domain\":\"" << static_cast<int>(depth.get_frame_timestamp_domain()) << "\","
         << "\"frame_id\":\"camera_depth_optical_frame\","
         << "\"coverage\":\"forward_fov\","
         << "\"center_distance_m\":" << go2wJsonNumberOrNull(center_m) << ","
         << "\"center_window_m\":" << go2wJsonNumberOrNull(front_m) << ","
         << "\"front_clearance_m\":" << go2wJsonNumberOrNull(front_m) << ","
         << "\"left_clearance_m\":" << go2wJsonNumberOrNull(left_m) << ","
         << "\"right_clearance_m\":" << go2wJsonNumberOrNull(right_m) << ","
         << "\"rear_clearance_m\":null,"
         << "\"confidence\":" << std::setprecision(3) << confidence << ","
         << "\"roi_confidence\":{"
         << "\"front\":" << static_cast<double>(front.size()) / sector_total << ","
         << "\"left\":" << static_cast<double>(left.size()) / sector_total << ","
         << "\"right\":" << static_cast<double>(right.size()) / sector_total << ","
         << "\"center_window\":" << static_cast<double>(front.size()) / sector_total
         << "},\"stale\":false,"
         << "\"summary\":{\"shape\":[" << height << "," << width
         << "],\"percentile\":10.0,\"valid_range_m\":[0.15,8.0],\"sample_step_px\":"
         << sample_step << "}}";
    const std::string tmp_path = output_path + ".tmp";
    {
        std::ofstream output(tmp_path, std::ios::out | std::ios::trunc);
        if (!output.is_open()) return;
        output << json.str() << std::endl;
    }
    std::rename(tmp_path.c_str(), output_path.c_str());
}

static std::string go2wAttachCaptureMetadata(
    std::string json,
    uint64_t frame_sequence,
    double sensor_timestamp_ms,
    uint64_t captured_at_ms
) {
    const size_t end = json.rfind('}');
    if (end == std::string::npos) return json;
    std::ostringstream metadata;
    metadata << ",\"frame_sequence\":" << frame_sequence
             << ",\"sensor_timestamp_ms\":" << std::fixed << std::setprecision(3) << sensor_timestamp_ms
             << ",\"captured_at_ms\":" << captured_at_ms;
    json.insert(end, metadata.str());
    return json;
}
'''
if "static void go2wWriteDepthPacket(" not in text:
    capture_thread_index = text.find("void captureThread()")
    if capture_thread_index < 0:
        raise SystemExit("failed to locate captureThread insertion for D435 helpers")
    text = text[:capture_thread_index] + helper_code + "\n" + text[capture_thread_index:]

capture_marker = """    for (int mode : {2, 1, 0}) {
        rs2::config cfg;"""
capture_replacement = """    const char* input_fps_env = std::getenv("GO2W_DEEPYOLO_INPUT_FPS");
    int input_fps = input_fps_env ? std::atoi(input_fps_env) : 15;
    if (input_fps <= 0) input_fps = 15;
    const char* ir_mode_env = std::getenv("GO2W_DEEPYOLO_IR_MODE");
    int requested_ir_mode = ir_mode_env ? std::atoi(ir_mode_env) : 0;
    if (requested_ir_mode < 0 || requested_ir_mode > 2) requested_ir_mode = 0;
    std::vector<int> ir_modes = requested_ir_mode >= 2
        ? std::vector<int>{2, 1, 0}
        : (requested_ir_mode == 1 ? std::vector<int>{1, 0} : std::vector<int>{0});
    std::cout << "[GO2W] requested_ir_mode=" << requested_ir_mode << std::endl;

    for (int mode : ir_modes) {
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
    int capture_every_n = capture_every_n_env ? std::atoi(capture_every_n_env) : 5;
    if (capture_every_n <= 0) capture_every_n = 5;
    const char* depth_every_n_env = std::getenv("GO2W_D435_DEPTH_EVERY_N");
    int depth_every_n = depth_every_n_env ? std::atoi(depth_every_n_env) : 2;
    if (depth_every_n <= 0) depth_every_n = 2;
    const char* depth_packet_path_env = std::getenv("GO2W_D435_DEPTH_PACKET_PATH");
    std::string depth_packet_path = depth_packet_path_env ? depth_packet_path_env : "";
    uint64_t capture_frame_count = 0;

    std::cout << "[GO2W] capture_every_n=" << capture_every_n << std::endl;
    std::cout << "[GO2W] depth_every_n=" << depth_every_n
              << " depth_packet_path=" << depth_packet_path << std::endl;
    std::cout << "[RealSense] RGBD";"""
if capture_loop_marker not in text:
    raise SystemExit("failed to locate DeepYOLO capture loop setup")
text = text.replace(capture_loop_marker, capture_loop_replacement, 1)
capture_wait_marker = """            raw_frames = pipe.wait_for_frames();
            aligned_frames = align_to_color.process(raw_frames);"""
capture_wait_replacement = """            raw_frames = pipe.wait_for_frames();
            capture_frame_count++;
            rs2::depth_frame go2w_capture_depth = raw_frames.get_depth_frame();
            go2w_captured_at_ms = go2wEpochMs();
            go2w_frame_sequence = go2w_capture_depth
                ? go2w_capture_depth.get_frame_number()
                : capture_frame_count;
            go2w_sensor_timestamp_ms = go2w_capture_depth
                ? go2w_capture_depth.get_timestamp()
                : 0.0;
            if (go2w_capture_depth && capture_frame_count % depth_every_n == 0) {
                go2wWriteDepthPacket(go2w_capture_depth, depth_packet_path, go2w_captured_at_ms);
            }
            if (capture_every_n > 1 && capture_frame_count % capture_every_n != 0) {
                continue;
            }
            aligned_frames = align_to_color.process(raw_frames);"""
if capture_wait_marker not in text:
    raise SystemExit("failed to locate DeepYOLO capture frame wait")
text = text.replace(capture_wait_marker, capture_wait_replacement, 1)
capture_scope_marker = """        rs2::frameset raw_frames;
        rs2::frameset aligned_frames;

        try {"""
capture_scope_replacement = """        rs2::frameset raw_frames;
        rs2::frameset aligned_frames;
        uint64_t go2w_captured_at_ms = 0;
        uint64_t go2w_frame_sequence = 0;
        double go2w_sensor_timestamp_ms = 0.0;

        try {"""
if capture_scope_marker not in text:
    raise SystemExit("failed to locate DeepYOLO capture metadata scope")
text = text.replace(capture_scope_marker, capture_scope_replacement, 1)
shared_frame_marker = """    uint64_t timestamp_ms = 0;

    bool valid_color = false;"""
shared_frame_replacement = """    uint64_t timestamp_ms = 0;
    uint64_t frame_id = 0;
    uint64_t captured_at_ms = 0;
    double sensor_timestamp_ms = 0.0;

    bool valid_color = false;"""
if shared_frame_marker not in text:
    raise SystemExit("failed to locate DeepYOLO shared sensor timestamp")
text = text.replace(shared_frame_marker, shared_frame_replacement, 1)
capture_publish_marker = """            shared_sensor.timestamp_ms = nowMs();
            shared_sensor.valid_color = true;"""
capture_publish_replacement = """            shared_sensor.timestamp_ms = go2w_captured_at_ms;
            shared_sensor.frame_id = go2w_frame_sequence;
            shared_sensor.captured_at_ms = go2w_captured_at_ms;
            shared_sensor.sensor_timestamp_ms = go2w_sensor_timestamp_ms;
            shared_sensor.valid_color = true;"""
if capture_publish_marker not in text:
    raise SystemExit("failed to locate DeepYOLO shared sensor publish")
text = text.replace(capture_publish_marker, capture_publish_replacement, 1)

old_engine = 'std::string engine_path = "../yolo11m.engine";'
new_engine = (
    'const char* engine_env = std::getenv("GO2W_DEEPYOLO_ENGINE");\n'
    f'    std::string engine_path = engine_env ? engine_env : "{engine_path}";'
)
if old_engine not in text:
    raise SystemExit("failed to locate DeepYOLO TensorRT engine path")
text = text.replace(old_engine, new_engine, 1)

old_output = 'std::string semantic_path = "../output/semantic_stream_" + session_id + ".jsonl";'
new_output = (
    'const char* output_env = std::getenv("GO2W_DEEPYOLO_OUTPUT_DIR");\n'
    f'    std::string semantic_dir = output_env ? output_env : "{output_dir}";\n'
    '    std::string semantic_path = semantic_dir + "/semantic_stream_" + session_id + ".jsonl";'
)
if old_output not in text:
    raise SystemExit("failed to locate DeepYOLO semantic JSONL path")
text = text.replace(old_output, new_output, 1)
semantic_open_marker = """    std::ofstream semantic_out(semantic_path, std::ios::out);

    if (!semantic_out.is_open()) {"""
semantic_open_replacement = """    std::ofstream semantic_out(semantic_path, std::ios::out);
    const char* heartbeat_env = std::getenv("GO2W_DEEPYOLO_HEARTBEAT_MS");
    uint64_t heartbeat_ms = heartbeat_env ? std::strtoull(heartbeat_env, nullptr, 10) : 1000;
    const char* max_jsonl_bytes_env = std::getenv("GO2W_DEEPYOLO_MAX_JSONL_BYTES");
    uint64_t max_jsonl_bytes = max_jsonl_bytes_env ? std::strtoull(max_jsonl_bytes_env, nullptr, 10) : 16777216;
    const char* max_jsonl_files_env = std::getenv("GO2W_DEEPYOLO_MAX_JSONL_FILES");
    uint64_t max_jsonl_files = max_jsonl_files_env ? std::strtoull(max_jsonl_files_env, nullptr, 10) : 4;
    if (max_jsonl_files == 0) max_jsonl_files = 4;
    uint64_t last_semantic_write_ms = 0;
    uint64_t semantic_file_index = 0;
    std::cout << "[GO2W] heartbeat_ms=" << heartbeat_ms
              << " max_jsonl_bytes=" << max_jsonl_bytes
              << " max_jsonl_files=" << max_jsonl_files << std::endl;

    if (!semantic_out.is_open()) {"""
if semantic_open_marker not in text:
    raise SystemExit("failed to locate DeepYOLO semantic JSONL open")
text = text.replace(semantic_open_marker, semantic_open_replacement, 1)
semantic_write_marker = """        if (semantic_out.is_open()) {
            if (shouldWriteEventPacket(packet, has_prev_packet, prev_packet)) {
            semantic_out << packetToJson(packet) << std::endl;
            prev_packet = packet;
            has_prev_packet = true;
        }
}"""
semantic_write_replacement = """        if (semantic_out.is_open()) {
            uint64_t semantic_write_ts = nowMs();
            bool heartbeat_due = heartbeat_ms > 0 && semantic_write_ts - last_semantic_write_ms >= heartbeat_ms;
            if (shouldWriteEventPacket(packet, has_prev_packet, prev_packet) || heartbeat_due) {
                const std::streamoff semantic_bytes = static_cast<std::streamoff>(semantic_out.tellp());
                if (max_jsonl_bytes > 0 && semantic_bytes >= 0
                    && static_cast<uint64_t>(semantic_bytes) >= max_jsonl_bytes) {
                    semantic_out.close();
                    semantic_file_index = (semantic_file_index + 1) % max_jsonl_files;
                    semantic_path = semantic_dir + "/semantic_stream_" + session_id;
                    if (semantic_file_index > 0) {
                        semantic_path += "_" + std::to_string(semantic_file_index);
                    }
                    semantic_path += ".jsonl";
                    semantic_out.open(semantic_path, std::ios::out);
                }
                if (semantic_out.is_open()) {
                    semantic_out << go2wAttachCaptureMetadata(
                        packetToJson(packet),
                        inference_frame_sequence,
                        inference_sensor_timestamp_ms,
                        inference_captured_at_ms
                    ) << std::endl;
                    last_semantic_write_ms = semantic_write_ts;
                    prev_packet = packet;
                    has_prev_packet = true;
                }
            }
        }"""
if semantic_write_marker not in text:
    raise SystemExit("failed to locate DeepYOLO semantic JSONL write")
text = text.replace(semantic_write_marker, semantic_write_replacement, 1)

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
    int inference_interval_ms = inference_interval_env ? std::atoi(inference_interval_env) : 333;
    if (inference_interval_ms < 0) inference_interval_ms = 333;
    auto next_inference_time = std::chrono::steady_clock::now();
    std::cout << "[GO2W] inference_interval_ms=" << inference_interval_ms << std::endl;
    uint64_t last_inference_sensor_frame_id = 0;

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
overlay_setup_marker = """    uint64_t last_inference_sensor_frame_id = 0;

    while (is_running) {"""
overlay_setup_replacement = """    uint64_t last_inference_sensor_frame_id = 0;
    uint64_t inference_frame_sequence = 0;
    uint64_t inference_captured_at_ms = 0;
    double inference_sensor_timestamp_ms = 0.0;
    const char* render_overlay_env = std::getenv("GO2W_DEEPYOLO_RENDER_OVERLAY");
    bool render_overlay = render_overlay_env && std::atoi(render_overlay_env) != 0;
    std::cout << "[GO2W] render_overlay=" << (render_overlay ? 1 : 0) << std::endl;

    while (is_running) {"""
if overlay_setup_marker not in text:
    raise SystemExit("failed to locate DeepYOLO overlay setup insertion")
text = text.replace(overlay_setup_marker, overlay_setup_replacement, 1)
inference_copy_marker = """            if (!shared_sensor.valid_color || shared_sensor.color_bgr.empty()) continue;

            shared_sensor.color_bgr.copyTo(frame);"""
inference_copy_replacement = """            if (!shared_sensor.valid_color || shared_sensor.color_bgr.empty()) continue;
            if (shared_sensor.frame_id == last_inference_sensor_frame_id) continue;
            last_inference_sensor_frame_id = shared_sensor.frame_id;
            inference_frame_sequence = shared_sensor.frame_id;
            inference_captured_at_ms = shared_sensor.captured_at_ms;
            inference_sensor_timestamp_ms = shared_sensor.sensor_timestamp_ms;

            shared_sensor.color_bgr.copyTo(frame);"""
if inference_copy_marker not in text:
    raise SystemExit("failed to locate DeepYOLO inference frame copy")
text = text.replace(inference_copy_marker, inference_copy_replacement, 1)
perf_marker = """    int perf_frame_count = 0;
    double total_latency = 0.0;
    double total_infer_time = 0.0;"""
perf_replacement = """    int perf_frame_count = 0;
    double total_latency = 0.0;
    double total_infer_time = 0.0;
    auto perf_window_start = std::chrono::steady_clock::now();"""
if perf_marker not in text:
    raise SystemExit("failed to locate DeepYOLO performance counters")
text = text.replace(perf_marker, perf_replacement, 1)
fps_marker = "            double current_fps = 1000.0 / avg_latency;"
fps_replacement = """            auto perf_window_end = std::chrono::steady_clock::now();
            std::chrono::duration<double> perf_window_duration = perf_window_end - perf_window_start;
            double semantic_fps = perf_window_duration.count() > 0.0
                ? 30.0 / perf_window_duration.count()
                : 0.0;"""
if fps_marker not in text:
    raise SystemExit("failed to locate DeepYOLO performance FPS calculation")
text = text.replace(fps_marker, fps_replacement, 1)
if '<< current_fps << " FPS | "' not in text:
    raise SystemExit("failed to locate DeepYOLO performance FPS output")
text = text.replace('<< current_fps << " FPS | "', '<< semantic_fps << " semantic FPS | "', 1)
perf_reset_marker = """            total_latency = 0.0;
            total_infer_time = 0.0;"""
perf_reset_replacement = """            total_latency = 0.0;
            total_infer_time = 0.0;
            perf_window_start = perf_window_end;"""
if perf_reset_marker not in text:
    raise SystemExit("failed to locate DeepYOLO performance reset")
text = text.replace(perf_reset_marker, perf_reset_replacement, 1)
overlay_block_marker = """            // 绘框
            {"""
overlay_block_replacement = """            // Optional overlay is useful for a short diagnostic session only.
            if (render_overlay) {"""
if overlay_block_marker not in text:
    raise SystemExit("failed to locate DeepYOLO render overlay block")
text = text.replace(overlay_block_marker, overlay_block_replacement, 1)

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
