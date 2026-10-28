#include "fft_imaging/config.hpp"

#include <algorithm>
#include <cctype>
#include <fstream>
#include <map>
#include <sstream>
#include <stdexcept>

namespace fft_imaging {

namespace {

std::string trim(std::string value) {
    auto not_space = [](unsigned char ch) { return !std::isspace(ch); };
    value.erase(value.begin(), std::find_if(value.begin(), value.end(), not_space));
    value.erase(std::find_if(value.rbegin(), value.rend(), not_space).base(), value.end());
    return value;
}

bool parse_bool(const std::string& value) {
    if (value == "true" || value == "1") {
        return true;
    }
    if (value == "false" || value == "0") {
        return false;
    }
    throw std::invalid_argument("Invalid boolean value: " + value);
}

int parse_int(const std::string& value) {
    std::size_t parsed = 0;
    const int result = std::stoi(value, &parsed);
    if (parsed != value.size()) {
        throw std::invalid_argument("Invalid integer value: " + value);
    }
    return result;
}

std::size_t parse_size(const std::string& value) {
    std::size_t parsed = 0;
    const auto result = std::stoull(value, &parsed);
    if (parsed != value.size()) {
        throw std::invalid_argument("Invalid size_t value: " + value);
    }
    return static_cast<std::size_t>(result);
}

double parse_double(const std::string& value) {
    std::size_t parsed = 0;
    const double result = std::stod(value, &parsed);
    if (parsed != value.size()) {
        throw std::invalid_argument("Invalid double value: " + value);
    }
    return result;
}

ChannelSelectionMode parse_channel_mode(const std::string& value) {
    if (value == "single") {
        return ChannelSelectionMode::Single;
    }
    if (value == "explicit_list") {
        return ChannelSelectionMode::ExplicitList;
    }
    if (value == "combined_reserved") {
        return ChannelSelectionMode::CombinedReserved;
    }
    throw std::invalid_argument("Invalid channel selection mode: " + value);
}

InterpolationMode parse_interpolation_mode(const std::string& value) {
    if (value == "bilinear") {
        return InterpolationMode::Bilinear;
    }
    if (value == "bicubic") {
        return InterpolationMode::Bicubic;
    }
    if (value == "spline") {
        return InterpolationMode::Spline;
    }
    throw std::invalid_argument("Invalid interpolation mode: " + value);
}

RuntimeMode parse_runtime_mode(const std::string& value) {
    if (value == "offline") {
        return RuntimeMode::Offline;
    }
    if (value == "realtime") {
        return RuntimeMode::Realtime;
    }
    throw std::invalid_argument("Invalid runtime mode: " + value);
}

StreamSourceKind parse_stream_source_kind(const std::string& value) {
    if (value == "bin_replay") {
        return StreamSourceKind::BinReplay;
    }
    if (value == "dca1000") {
        return StreamSourceKind::Dca1000;
    }
    throw std::invalid_argument("Invalid stream source: " + value);
}

std::string to_string(ChannelSelectionMode mode) {
    switch (mode) {
        case ChannelSelectionMode::Single:
            return "single";
        case ChannelSelectionMode::ExplicitList:
            return "explicit_list";
        case ChannelSelectionMode::CombinedReserved:
            return "combined_reserved";
    }
    throw std::invalid_argument("Unknown ChannelSelectionMode.");
}

std::string to_string(InterpolationMode mode) {
    switch (mode) {
        case InterpolationMode::Bilinear:
            return "bilinear";
        case InterpolationMode::Bicubic:
            return "bicubic";
        case InterpolationMode::Spline:
            return "spline";
    }
    throw std::invalid_argument("Unknown InterpolationMode.");
}

std::string to_string(RuntimeMode mode) {
    switch (mode) {
        case RuntimeMode::Offline:
            return "offline";
        case RuntimeMode::Realtime:
            return "realtime";
    }
    throw std::invalid_argument("Unknown RuntimeMode.");
}

std::string to_string(StreamSourceKind kind) {
    switch (kind) {
        case StreamSourceKind::BinReplay:
            return "bin_replay";
        case StreamSourceKind::Dca1000:
            return "dca1000";
    }
    throw std::invalid_argument("Unknown StreamSourceKind.");
}

BackendKind parse_backend_kind(const std::string& value) {
    if (value == "cpu_serial") {
        return BackendKind::CpuSerial;
    }
    if (value == "cpu_parallel") {
        return BackendKind::CpuParallel;
    }
    if (value == "cuda") {
        return BackendKind::Cuda;
    }
    throw std::invalid_argument("Invalid backend kind: " + value);
}

std::string to_string(BackendKind kind) {
    switch (kind) {
        case BackendKind::CpuSerial:
            return "cpu_serial";
        case BackendKind::CpuParallel:
            return "cpu_parallel";
        case BackendKind::Cuda:
            return "cuda";
    }
    throw std::invalid_argument("Unknown BackendKind.");
}

void apply_config_value(AppConfig& config, const std::string& key, const std::string& value) {
    if (key == "acquisition.num_adc_bits") {
        config.acquisition.reader.num_adc_bits = parse_int(value);
        return;
    }
    if (key == "acquisition.num_lanes") {
        config.acquisition.reader.num_lanes = parse_int(value);
        return;
    }
    if (key == "acquisition.is_real") {
        config.acquisition.reader.is_real = parse_bool(value);
        return;
    }
    if (key == "fft.nr") {
        config.fft_pipeline.nr = parse_size(value);
        return;
    }
    if (key == "fft.frag") {
        config.fft_pipeline.frame_selection.frag = parse_size(value);
        return;
    }
    if (key == "fft.step") {
        config.fft_pipeline.frame_selection.step = parse_size(value);
        return;
    }
    if (key == "fft.zero_range_bins") {
        config.fft_pipeline.frame_selection.zero_range_bins = parse_size(value);
        return;
    }
    if (key == "fft.circshift_range_bins") {
        config.fft_pipeline.frame_selection.circshift_range_bins = parse_size(value);
        return;
    }
    if (key == "fft.zero_head_after_shift") {
        config.fft_pipeline.frame_selection.zero_head_after_shift = parse_size(value);
        return;
    }
    if (key == "fft.zero_tail_after_shift") {
        config.fft_pipeline.frame_selection.zero_tail_after_shift = parse_size(value);
        return;
    }
    if (key == "imaging.channel_mode") {
        config.imaging.channel_selection.mode = parse_channel_mode(value);
        return;
    }
    if (key == "imaging.channel_index") {
        config.imaging.channel_selection.channel_index = parse_int(value);
        return;
    }
    if (key == "imaging.interpolation_mode") {
        config.imaging.interpolation_mode = parse_interpolation_mode(value);
        return;
    }
    if (key == "imaging.backend") {
        config.imaging.backend = parse_backend_kind(value);
        return;
    }
    if (key == "runtime.mode") {
        config.runtime.mode = parse_runtime_mode(value);
        return;
    }
    if (key == "runtime.max_frames") {
        config.runtime.max_frames = parse_size(value);
        return;
    }
    if (key == "runtime.realtime_poll_interval_ms") {
        config.runtime.realtime_poll_interval_ms = parse_size(value);
        return;
    }
    if (key == "runtime.continue_on_frame_error") {
        config.runtime.continue_on_frame_error = parse_bool(value);
        return;
    }
    if (key == "runtime.max_consecutive_frame_errors") {
        config.runtime.max_consecutive_frame_errors = parse_size(value);
        return;
    }
    if (key == "stream.source") {
        config.stream.source = parse_stream_source_kind(value);
        return;
    }
    if (key == "stream.replay_rate_hz") {
        config.stream.replay_rate_hz = parse_double(value);
        return;
    }
    if (key == "stream.ring_columns") {
        config.stream.ring_columns = parse_size(value);
        return;
    }
    if (key == "display.enabled") {
        config.display.enabled = parse_bool(value);
        return;
    }
    if (key == "display.save_every_n") {
        config.display.save_every_n = parse_size(value);
        return;
    }
    if (key == "imaging.scene.rmax_m") {
        config.imaging.scene_grid.rmax_m = parse_double(value);
        return;
    }
    if (key == "imaging.scene.prf_hz") {
        config.imaging.scene_grid.prf_hz = parse_double(value);
        return;
    }
    if (key == "imaging.scene.c_mps") {
        config.imaging.scene_grid.c_mps = parse_double(value);
        return;
    }
    if (key == "imaging.scene.fc_hz") {
        config.imaging.scene_grid.fc_hz = parse_double(value);
        return;
    }
    if (key == "imaging.scene.fs_hz") {
        config.imaging.scene_grid.fs_hz = parse_double(value);
        return;
    }
    if (key == "imaging.scene.beam_width_deg") {
        config.imaging.scene_grid.beam_width_deg = parse_double(value);
        return;
    }
    if (key == "imaging.scene.squint_angle_deg") {
        config.imaging.scene_grid.squint_angle_deg = parse_double(value);
        return;
    }
    if (key == "imaging.scene.grid_spacing_m") {
        config.imaging.scene_grid.grid_spacing_m = parse_double(value);
        return;
    }
    if (key == "render.jet_levels") {
        config.render.jet_levels = parse_int(value);
        return;
    }
    if (key == "render.apply_myorder") {
        config.render.apply_myorder = parse_bool(value);
        return;
    }
    if (key == "render.low_value_white") {
        config.render.low_value_white = parse_bool(value);
        return;
    }
    if (key == "render.output_gamma") {
        config.render.output_gamma = parse_double(value);
        return;
    }
    throw std::invalid_argument("Unknown config key: " + key);
}

}  // namespace

AppConfig make_matlab_equivalent_app_config() {
    AppConfig config;
    config.acquisition.reader.num_adc_bits = 16;
    config.acquisition.reader.num_lanes = 4;
    config.acquisition.reader.is_real = false;

    config.fft_pipeline.nr = 1024;
    config.fft_pipeline.frame_selection.frag = 2000;
    config.fft_pipeline.frame_selection.step = 200;
    config.fft_pipeline.frame_selection.zero_range_bins = 100;
    config.fft_pipeline.frame_selection.circshift_range_bins = 512;
    config.fft_pipeline.frame_selection.zero_head_after_shift = 20;
    config.fft_pipeline.frame_selection.zero_tail_after_shift = 20;

    config.imaging.channel_selection.mode = ChannelSelectionMode::Single;
    config.imaging.channel_selection.channel_index = 0;
    config.imaging.interpolation_mode = InterpolationMode::Spline;
    config.imaging.backend = BackendKind::CpuSerial;
    config.runtime.mode = RuntimeMode::Offline;
    config.runtime.max_frames = 0;
    config.runtime.realtime_poll_interval_ms = 0;
    config.runtime.continue_on_frame_error = false;
    config.runtime.max_consecutive_frame_errors = 1;
    config.stream.source = StreamSourceKind::BinReplay;
    config.stream.replay_rate_hz = 9660.0;
    config.stream.ring_columns = 4096;
    config.display.enabled = true;
    config.display.save_every_n = 0;

    config.render.jet_levels = 15000;
    config.render.apply_myorder = true;
    config.render.low_value_white = true;
    config.render.output_gamma = 1.0;

    return config;
}

AppConfig load_app_config_file(
    const std::filesystem::path& config_path,
    const AppConfig& base_config) {
    std::ifstream input(config_path);
    if (!input) {
        throw std::runtime_error("Failed to open config file: " + config_path.string());
    }

    AppConfig config = base_config;
    std::string line;
    std::size_t line_no = 0;
    while (std::getline(input, line)) {
        ++line_no;
        const auto comment_pos = line.find('#');
        if (comment_pos != std::string::npos) {
            line = line.substr(0, comment_pos);
        }
        line = trim(line);
        if (line.empty()) {
            continue;
        }

        const auto separator = line.find('=');
        if (separator == std::string::npos) {
            throw std::runtime_error(
                "Invalid config line " + std::to_string(line_no) + ": expected key=value.");
        }

        const std::string key = trim(line.substr(0, separator));
        const std::string value = trim(line.substr(separator + 1));
        try {
            apply_config_value(config, key, value);
        } catch (const std::exception& ex) {
            throw std::runtime_error(
                "Invalid config line " + std::to_string(line_no) + " for key '" + key + "': " + ex.what());
        }
    }

    return config;
}

void write_app_config_template(
    const std::filesystem::path& output_path,
    const AppConfig& config) {
    if (!output_path.parent_path().empty()) {
        std::filesystem::create_directories(output_path.parent_path());
    }
    std::ofstream output(output_path);
    if (!output) {
        throw std::runtime_error("Failed to write config template: " + output_path.string());
    }
    output << serialize_app_config_with_comments(config);
}

std::string serialize_app_config_with_comments(const AppConfig& config) {
    std::ostringstream out;
    out << "# MATLAB-equivalent default configuration for fft_imaging_cpp\n";
    out << "# Edit values here to override the built-in defaults without changing code.\n\n";
    out << "# Acquisition config\n";
    out << "acquisition.num_adc_bits = " << config.acquisition.reader.num_adc_bits << "\n";
    out << "acquisition.num_lanes = " << config.acquisition.reader.num_lanes << "\n";
    out << "acquisition.is_real = " << (config.acquisition.reader.is_real ? "true" : "false") << "\n\n";
    out << "# FFT pipeline config\n";
    out << "fft.nr = " << config.fft_pipeline.nr << "\n";
    out << "fft.frag = " << config.fft_pipeline.frame_selection.frag << "\n";
    out << "fft.step = " << config.fft_pipeline.frame_selection.step << "\n";
    out << "fft.zero_range_bins = " << config.fft_pipeline.frame_selection.zero_range_bins << "\n";
    out << "fft.circshift_range_bins = " << config.fft_pipeline.frame_selection.circshift_range_bins << "\n";
    out << "fft.zero_head_after_shift = " << config.fft_pipeline.frame_selection.zero_head_after_shift << "\n";
    out << "fft.zero_tail_after_shift = " << config.fft_pipeline.frame_selection.zero_tail_after_shift << "\n\n";
    out << "# Imaging config\n";
    out << "imaging.channel_mode = " << to_string(config.imaging.channel_selection.mode) << "\n";
    out << "imaging.channel_index = " << config.imaging.channel_selection.channel_index << "\n";
    out << "imaging.interpolation_mode = " << to_string(config.imaging.interpolation_mode) << "\n";
    out << "imaging.backend = " << to_string(config.imaging.backend) << "\n";
    out << "imaging.scene.rmax_m = " << config.imaging.scene_grid.rmax_m << "\n";
    out << "imaging.scene.prf_hz = " << config.imaging.scene_grid.prf_hz << "\n";
    out << "imaging.scene.c_mps = " << config.imaging.scene_grid.c_mps << "\n";
    out << "imaging.scene.fc_hz = " << config.imaging.scene_grid.fc_hz << "\n";
    out << "imaging.scene.fs_hz = " << config.imaging.scene_grid.fs_hz << "\n";
    out << "imaging.scene.beam_width_deg = " << config.imaging.scene_grid.beam_width_deg << "\n";
    out << "imaging.scene.squint_angle_deg = " << config.imaging.scene_grid.squint_angle_deg << "\n";
    out << "imaging.scene.grid_spacing_m = " << config.imaging.scene_grid.grid_spacing_m << "\n\n";
    out << "# Runtime execution config\n";
    out << "runtime.mode = " << to_string(config.runtime.mode) << "\n";
    out << "runtime.max_frames = " << config.runtime.max_frames << "\n";
    out << "runtime.realtime_poll_interval_ms = " << config.runtime.realtime_poll_interval_ms << "\n\n";
    out << "runtime.continue_on_frame_error = "
        << (config.runtime.continue_on_frame_error ? "true" : "false") << "\n";
    out << "runtime.max_consecutive_frame_errors = "
        << config.runtime.max_consecutive_frame_errors << "\n\n";
    out << "# Stream config\n";
    out << "stream.source = " << to_string(config.stream.source) << "\n";
    out << "stream.replay_rate_hz = " << config.stream.replay_rate_hz << "\n";
    out << "stream.ring_columns = " << config.stream.ring_columns << "\n\n";
    out << "# Display config\n";
    out << "display.enabled = " << (config.display.enabled ? "true" : "false") << "\n";
    out << "display.save_every_n = " << config.display.save_every_n << "\n\n";
    out << "# Render config\n";
    out << "render.jet_levels = " << config.render.jet_levels << "\n";
    out << "render.apply_myorder = " << (config.render.apply_myorder ? "true" : "false") << "\n";
    out << "render.low_value_white = " << (config.render.low_value_white ? "true" : "false") << "\n";
    out << "render.output_gamma = " << config.render.output_gamma << "\n\n";
    return out.str();
}

}  // namespace fft_imaging
