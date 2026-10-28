#pragma once

#include <filesystem>
#include <string>

#include "fft_imaging/backend.hpp"
#include "fft_imaging/image_writer.hpp"
#include "fft_imaging/range_angle_imager.hpp"

namespace fft_imaging {

enum class RuntimeMode {
    Offline = 0,
    Realtime = 1,
};

struct AcquisitionConfig {
    ReaderConfig reader;
};

struct FftPipelineConfig {
    std::size_t nr = 1024;
    FrameSelection frame_selection;
};

struct ImagingRuntimeConfig {
    ChannelSelection channel_selection;
    SceneGridConfig scene_grid;
    InterpolationMode interpolation_mode = InterpolationMode::Bicubic;
    BackendKind backend = BackendKind::CpuSerial;
};

struct RuntimeExecutionConfig {
    RuntimeMode mode = RuntimeMode::Offline;
    std::size_t max_frames = 0;
    std::size_t realtime_poll_interval_ms = 0;
    bool continue_on_frame_error = false;
    std::size_t max_consecutive_frame_errors = 1;
};

enum class StreamSourceKind {
    BinReplay = 0,
    Dca1000 = 1,
};

struct StreamConfig {
    StreamSourceKind source = StreamSourceKind::BinReplay;
    double replay_rate_hz = 9660.0;
    std::size_t ring_columns = 4096;
};

struct DisplayConfig {
    bool enabled = true;
    std::size_t save_every_n = 0;
};

struct AppConfig {
    AcquisitionConfig acquisition;
    FftPipelineConfig fft_pipeline;
    ImagingRuntimeConfig imaging;
    RuntimeExecutionConfig runtime;
    StreamConfig stream;
    DisplayConfig display;
    RenderConfig render;
};

AppConfig make_matlab_equivalent_app_config();

AppConfig load_app_config_file(
    const std::filesystem::path& config_path,
    const AppConfig& base_config = make_matlab_equivalent_app_config());

void write_app_config_template(
    const std::filesystem::path& output_path,
    const AppConfig& config = make_matlab_equivalent_app_config());

std::string serialize_app_config_with_comments(const AppConfig& config = make_matlab_equivalent_app_config());

}  // namespace fft_imaging
