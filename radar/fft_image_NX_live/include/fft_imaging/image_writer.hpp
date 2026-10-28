#pragma once

#include <filesystem>

#include "fft_imaging/types.hpp"

namespace cv {
class Mat;
}

namespace fft_imaging {

struct RenderConfig {
    int jet_levels = 15000;
    bool apply_myorder = true;
    bool low_value_white = true;
    double output_gamma = 1.0;
};

Eigen::MatrixXd abs_normalize(const MatrixXc& input);
Eigen::MatrixXd myorder_map(const MatrixXc& input);
cv::Mat render_complex_image_to_bgr(
    const MatrixXc& image,
    const RenderConfig& config = {});

void write_jpg_from_complex_image(
    const MatrixXc& image,
    const std::filesystem::path& output_path,
    const RenderConfig& config = {});

}  // namespace fft_imaging
