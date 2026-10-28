#pragma once

#include <filesystem>

#include "fft_imaging/session.hpp"
#include "fft_imaging/types.hpp"

namespace fft_imaging {

class RawCaptureReader {
public:
    explicit RawCaptureReader(ReaderConfig config);

    MatrixXc read_dca1000_complex(const std::filesystem::path& file_path) const;
    RawCaptureSession load_session(
        const std::filesystem::path& file_path,
        const ImagingConfig& imaging_config) const;

private:
    ReaderConfig config_;
};

}  // namespace fft_imaging
