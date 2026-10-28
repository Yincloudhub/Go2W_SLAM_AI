#pragma once

#include <filesystem>

#include "fft_imaging/types.hpp"

namespace fft_imaging {

class RawCaptureSession {
public:
    RawCaptureSession(
        MatrixXc raw_channels,
        VectorXc selected_channel,
        MatrixXc echo_matrix,
        std::filesystem::path source_path = {});

    const MatrixXc& raw_channels() const;
    const VectorXc& selected_channel() const;
    const MatrixXc& echo_matrix() const;
    const std::filesystem::path& source_path() const;

    std::size_t total_frames(std::size_t frag, std::size_t step) const;

private:
    MatrixXc raw_channels_;
    VectorXc selected_channel_;
    MatrixXc echo_matrix_;
    std::filesystem::path source_path_;
};

RawCaptureSession make_raw_capture_session(
    const MatrixXc& raw_channels,
    const ImagingConfig& imaging_config,
    std::filesystem::path source_path = {});

}  // namespace fft_imaging
