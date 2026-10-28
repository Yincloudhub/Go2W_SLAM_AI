#pragma once

#include <complex>
#include <cstddef>
#include <vector>

#include <Eigen/Dense>

namespace fft_imaging {

using Complex = std::complex<double>;
using MatrixXc = Eigen::Matrix<Complex, Eigen::Dynamic, Eigen::Dynamic, Eigen::ColMajor>;
using VectorXc = Eigen::Matrix<Complex, Eigen::Dynamic, 1>;
using VectorXd = Eigen::VectorXd;

enum class Axis {
    Row = 0,
    Col = 1,
};

enum class ChannelSelectionMode {
    Single = 0,
    ExplicitList = 1,
    CombinedReserved = 2,
};

enum class PipelineMode {
    Validation = 0,
    Runtime = 1,
};

struct ChannelSelection {
    ChannelSelectionMode mode = ChannelSelectionMode::Single;
    int channel_index = 0;
    std::vector<int> channel_indices;
};

struct FrameSelection {
    std::size_t frame_index = 0;
    std::size_t frag = 0;
    std::size_t step = 0;
    std::size_t zero_range_bins = 0;
    std::size_t circshift_range_bins = 0;
    std::size_t zero_head_after_shift = 0;
    std::size_t zero_tail_after_shift = 0;
};

struct ReaderConfig {
    int num_adc_bits = 16;
    int num_lanes = 4;
    bool is_real = false;
};

struct ImagingConfig {
    std::size_t nr = 1024;
    ChannelSelection channel_selection;
};

struct VideoFramePlan {
    std::size_t frag = 2000;
    std::size_t step = 200;
    std::size_t total_frames = 0;
};

struct PipelineConfig {
    PipelineMode mode = PipelineMode::Validation;
};

struct PipelineStageTimings {
    double range_fft_ms = 0.0;
    double azimuth_fft_ms = 0.0;
};

struct PipelineArtifacts {
    PipelineMode mode = PipelineMode::Validation;
    bool has_raw_channels = false;
    bool has_selected_channel = false;
    bool has_echo_matrix = false;
    bool has_frame_echo = false;
    bool has_frame_echo_zeroed = false;
    bool has_hrrp_after_range_fft = false;
    bool has_hrrp_after_circshift = false;
    bool has_hrrp_after_crop = false;
    bool has_fft2_after_azimuth_fft = false;
    MatrixXc raw_channels;
    VectorXc selected_channel;
    MatrixXc echo_matrix;
    MatrixXc frame_echo;
    MatrixXc frame_echo_zeroed;
    MatrixXc hrrp_after_range_fft;
    MatrixXc hrrp_after_circshift;
    MatrixXc hrrp_after_crop;
    MatrixXc fft2_after_azimuth_fft;
    PipelineStageTimings timings;
};

}  // namespace fft_imaging
