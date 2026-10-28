#include "fft_imaging/session.hpp"

#include <stdexcept>

#include "fft_imaging/matrix_ops.hpp"

namespace fft_imaging {

RawCaptureSession::RawCaptureSession(
    MatrixXc raw_channels,
    VectorXc selected_channel,
    MatrixXc echo_matrix,
    std::filesystem::path source_path)
    : raw_channels_(std::move(raw_channels)),
      selected_channel_(std::move(selected_channel)),
      echo_matrix_(std::move(echo_matrix)),
      source_path_(std::move(source_path)) {}

const MatrixXc& RawCaptureSession::raw_channels() const {
    return raw_channels_;
}

const VectorXc& RawCaptureSession::selected_channel() const {
    return selected_channel_;
}

const MatrixXc& RawCaptureSession::echo_matrix() const {
    return echo_matrix_;
}

const std::filesystem::path& RawCaptureSession::source_path() const {
    return source_path_;
}

std::size_t RawCaptureSession::total_frames(std::size_t frag, std::size_t step) const {
    if (frag == 0) {
        throw std::invalid_argument("frag must be positive.");
    }
    if (step == 0) {
        throw std::invalid_argument("step must be positive.");
    }
    if (echo_matrix_.cols() < static_cast<Eigen::Index>(frag)) {
        return 0;
    }
    return static_cast<std::size_t>(
        (echo_matrix_.cols() - static_cast<Eigen::Index>(frag)) /
        static_cast<Eigen::Index>(step));
}

RawCaptureSession make_raw_capture_session(
    const MatrixXc& raw_channels,
    const ImagingConfig& imaging_config,
    std::filesystem::path source_path) {
    if (imaging_config.nr == 0) {
        throw std::invalid_argument("ImagingConfig.nr must be positive.");
    }

    const VectorXc selected_channel =
        select_channel_row_as_vector(raw_channels, imaging_config.channel_selection);

    const Eigen::Index available_columns =
        static_cast<Eigen::Index>(selected_channel.size() / imaging_config.nr);
    const Eigen::Index usable_samples =
        available_columns * static_cast<Eigen::Index>(imaging_config.nr);
    const VectorXc truncated = selected_channel.head(usable_samples);
    const MatrixXc echo_matrix =
        reshape_column_major(truncated, static_cast<Eigen::Index>(imaging_config.nr));

    return RawCaptureSession(raw_channels, selected_channel, echo_matrix, std::move(source_path));
}

}  // namespace fft_imaging
