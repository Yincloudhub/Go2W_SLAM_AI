#include "fft_imaging/core_pipeline.hpp"

#include <chrono>
#include <stdexcept>

#include "fft_imaging/matrix_ops.hpp"

namespace fft_imaging {

CorePipeline::CorePipeline(
    ImagingConfig imaging_config,
    PipelineConfig pipeline_config)
    : imaging_config_(imaging_config),
      pipeline_config_(pipeline_config) {}

PipelineArtifacts CorePipeline::run_fft_instant_imaging(
    const RawCaptureSession& session,
    const FrameSelection& frame_selection) const {
    const bool keep_validation_stages = pipeline_config_.mode == PipelineMode::Validation;
    const Eigen::Index frag = static_cast<Eigen::Index>(frame_selection.frag);
    const Eigen::Index step = static_cast<Eigen::Index>(frame_selection.step);
    const Eigen::Index frame_index = static_cast<Eigen::Index>(frame_selection.frame_index);
    const Eigen::Index start_col = frame_index * step;

    if (start_col < 0 || start_col + frag > session.echo_matrix().cols()) {
        throw std::out_of_range("Requested frame exceeds available echo matrix columns.");
    }

    MatrixXc frame_echo = session.echo_matrix().block(0, start_col, session.echo_matrix().rows(), frag);
    PipelineArtifacts artifacts = run_fft_instant_imaging_frame(frame_echo, frame_selection);
    if (keep_validation_stages) {
        artifacts.raw_channels = session.raw_channels();
        artifacts.selected_channel = session.selected_channel();
        artifacts.echo_matrix = session.echo_matrix();
        artifacts.has_raw_channels = true;
        artifacts.has_selected_channel = true;
        artifacts.has_echo_matrix = true;
    }
    return artifacts;
}

PipelineArtifacts CorePipeline::run_fft_instant_imaging_frame(
    const MatrixXc& frame_echo,
    const FrameSelection& frame_selection) const {
    if (imaging_config_.nr == 0) {
        throw std::invalid_argument("ImagingConfig.nr must be positive.");
    }
    if (frame_selection.frag == 0) {
        throw std::invalid_argument("FrameSelection.frag must be positive.");
    }
    if (frame_echo.rows() != static_cast<Eigen::Index>(imaging_config_.nr)) {
        throw std::invalid_argument("frame_echo row count does not match ImagingConfig.nr.");
    }
    if (frame_echo.cols() != static_cast<Eigen::Index>(frame_selection.frag)) {
        throw std::invalid_argument("frame_echo column count does not match FrameSelection.frag.");
    }

    PipelineArtifacts artifacts;
    artifacts.mode = pipeline_config_.mode;
    const bool keep_validation_stages = pipeline_config_.mode == PipelineMode::Validation;

    MatrixXc frame_echo_zeroed = frame_echo;

    const Eigen::Index zero_bins =
        std::min<Eigen::Index>(frame_echo_zeroed.rows(),
                               static_cast<Eigen::Index>(frame_selection.zero_range_bins));
    if (zero_bins > 0) {
        frame_echo_zeroed.topRows(zero_bins).setZero();
    }

    // MATLAB semantics: fftshift(fft(fftshift(Data,1),[],1),1)
    const auto range_fft_start = std::chrono::steady_clock::now();
    MatrixXc hrrp_after_range_fft =
        fftshift(fft_along_axis(fft_engine_, fftshift(frame_echo_zeroed, Axis::Row), Axis::Row), Axis::Row);
    const auto range_fft_end = std::chrono::steady_clock::now();
    artifacts.timings.range_fft_ms =
        std::chrono::duration<double, std::milli>(range_fft_end - range_fft_start).count();

    MatrixXc hrrp_after_circshift = circshift(
        hrrp_after_range_fft,
        Axis::Row,
        static_cast<Eigen::Index>(frame_selection.circshift_range_bins));

    MatrixXc hrrp_after_crop = hrrp_after_circshift;
    const Eigen::Index zero_head =
        std::min<Eigen::Index>(hrrp_after_crop.rows(),
                               static_cast<Eigen::Index>(frame_selection.zero_head_after_shift));
    const Eigen::Index zero_tail =
        std::min<Eigen::Index>(hrrp_after_crop.rows(),
                               static_cast<Eigen::Index>(frame_selection.zero_tail_after_shift));
    if (zero_head > 0) {
        hrrp_after_crop.topRows(zero_head).setZero();
    }
    if (zero_tail > 0) {
        hrrp_after_crop.bottomRows(zero_tail).setZero();
    }

    // MATLAB semantics: fftshift(fft(fftshift(Data,2),[],2),2)
    const auto azimuth_fft_start = std::chrono::steady_clock::now();
    artifacts.fft2_after_azimuth_fft =
        fftshift(fft_along_axis(fft_engine_, fftshift(hrrp_after_crop, Axis::Col), Axis::Col), Axis::Col);
    const auto azimuth_fft_end = std::chrono::steady_clock::now();
    artifacts.timings.azimuth_fft_ms =
        std::chrono::duration<double, std::milli>(azimuth_fft_end - azimuth_fft_start).count();
    artifacts.has_fft2_after_azimuth_fft = true;

    if (keep_validation_stages) {
        artifacts.frame_echo = frame_echo;
        artifacts.frame_echo_zeroed = std::move(frame_echo_zeroed);
        artifacts.hrrp_after_range_fft = std::move(hrrp_after_range_fft);
        artifacts.hrrp_after_circshift = std::move(hrrp_after_circshift);
        artifacts.hrrp_after_crop = std::move(hrrp_after_crop);
        artifacts.has_frame_echo = true;
        artifacts.has_frame_echo_zeroed = true;
        artifacts.has_hrrp_after_range_fft = true;
        artifacts.has_hrrp_after_circshift = true;
        artifacts.has_hrrp_after_crop = true;
    }

    return artifacts;
}

}  // namespace fft_imaging
