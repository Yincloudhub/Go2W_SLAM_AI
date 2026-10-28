#include "fft_imaging/image_writer.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <vector>

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

namespace fft_imaging {

namespace {

cv::Mat build_jet_colormap(int levels, bool low_value_white) {
    cv::Mat lut(levels, 1, CV_8UC3);
    for (int i = 0; i < levels; ++i) {
        const double x = static_cast<double>(i) / static_cast<double>(std::max(1, levels - 1));
        const double r = std::clamp(1.5 - std::abs(4.0 * x - 3.0), 0.0, 1.0);
        const double g = std::clamp(1.5 - std::abs(4.0 * x - 2.0), 0.0, 1.0);
        const double b = std::clamp(1.5 - std::abs(4.0 * x - 1.0), 0.0, 1.0);
        lut.at<cv::Vec3b>(i, 0) = cv::Vec3b(
            static_cast<unsigned char>(std::round(b * 255.0)),
            static_cast<unsigned char>(std::round(g * 255.0)),
            static_cast<unsigned char>(std::round(r * 255.0)));
    }

    // MATLAB saveCJ prepends white to jet. Live video can disable this so
    // low-energy background does not look like a blank white frame.
    if (low_value_white) {
        lut.at<cv::Vec3b>(0, 0) = cv::Vec3b(255, 255, 255);
    }
    return lut;
}

Eigen::MatrixXd normalize_01(const Eigen::MatrixXd& input) {
    if (input.size() == 0) {
        return input;
    }
    const double max_value = input.maxCoeff();
    if (max_value <= 1e-12) {
        return Eigen::MatrixXd::Zero(input.rows(), input.cols());
    }
    return input / max_value;
}

}  // namespace

Eigen::MatrixXd abs_normalize(const MatrixXc& input) {
    Eigen::MatrixXd mag = input.cwiseAbs();
    return normalize_01(mag);
}

Eigen::MatrixXd myorder_map(const MatrixXc& input) {
    Eigen::MatrixXd x_abs = abs_normalize(input);

    const int bins = 256;
    std::vector<int> hist(static_cast<std::size_t>(bins), 0);
    for (Eigen::Index row = 0; row < x_abs.rows(); ++row) {
        for (Eigen::Index col = 0; col < x_abs.cols(); ++col) {
            const double x = std::clamp(x_abs(row, col), 0.0, 1.0);
            const int idx = std::min(bins - 1, static_cast<int>(std::floor(x * (bins - 1))));
            hist[static_cast<std::size_t>(idx)] += 1;
        }
    }

    int best_bin = 1;
    for (int i = 2; i < bins; ++i) {
        if (hist[static_cast<std::size_t>(i)] > hist[static_cast<std::size_t>(best_bin)]) {
            best_bin = i;
        }
    }

    const double x = static_cast<double>(best_bin) / static_cast<double>(hist.size());
    const double safe_x = std::clamp(x, 1e-6, 1.0 - 1e-6);
    const double order = std::log(0.2) / std::log(safe_x);

    return x_abs.array().pow(order).matrix();
}

void write_jpg_from_complex_image(
    const MatrixXc& image,
    const std::filesystem::path& output_path,
    const RenderConfig& config) {
    const cv::Mat oriented = render_complex_image_to_bgr(image, config);

    std::filesystem::create_directories(output_path.parent_path());
    if (!cv::imwrite(output_path.string(), oriented)) {
        throw std::runtime_error("Failed to write JPG file: " + output_path.string());
    }
}

cv::Mat render_complex_image_to_bgr(
    const MatrixXc& image,
    const RenderConfig& config) {
    const Eigen::MatrixXd mapped = config.apply_myorder ? myorder_map(image) : abs_normalize(image);
    // Keep the earlier blue-dominant look by avoiding aggressive percentile stretching.
    Eigen::MatrixXd normalized = normalize_01(mapped);

    if (std::abs(config.output_gamma - 1.0) > 1e-12) {
        normalized = normalized.array().pow(config.output_gamma).matrix();
    }

    cv::Mat gray(normalized.rows(), normalized.cols(), CV_8UC1);
    for (Eigen::Index row = 0; row < normalized.rows(); ++row) {
        for (Eigen::Index col = 0; col < normalized.cols(); ++col) {
            gray.at<unsigned char>(static_cast<int>(row), static_cast<int>(col)) =
                static_cast<unsigned char>(std::round(std::clamp(normalized(row, col), 0.0, 1.0) * 255.0));
        }
    }

    const cv::Mat lut = build_jet_colormap(config.jet_levels, config.low_value_white);
    cv::Mat rgb(gray.rows, gray.cols, CV_8UC3);
    for (int row = 0; row < gray.rows; ++row) {
        for (int col = 0; col < gray.cols; ++col) {
            const int idx = static_cast<int>(
                std::round(gray.at<unsigned char>(row, col) / 255.0 * static_cast<double>(config.jet_levels - 1)));
            rgb.at<cv::Vec3b>(row, col) = lut.at<cv::Vec3b>(idx, 0);
        }
    }

    // Final output orientation requested by the user:
    // 1. rotate 180 degrees
    // 2. apply a mirror flip
    cv::Mat rotated;
    cv::rotate(rgb, rotated, cv::ROTATE_180);

    cv::Mat oriented;
    cv::flip(rotated, oriented, 1);

    return oriented;
}

}  // namespace fft_imaging
