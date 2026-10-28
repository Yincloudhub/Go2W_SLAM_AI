#include "fft_imaging/matrix_ops.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace fft_imaging {

namespace {

Eigen::Index normalized_shift(Eigen::Index shift, Eigen::Index size) {
    if (size <= 0) {
        return 0;
    }
    shift %= size;
    if (shift < 0) {
        shift += size;
    }
    return shift;
}

}  // namespace

MatrixXc fftshift(const MatrixXc& input, Axis axis) {
    if (axis == Axis::Row) {
        return circshift(input, axis, input.rows() / 2);
    }
    return circshift(input, axis, input.cols() / 2);
}

MatrixXc circshift(const MatrixXc& input, Axis axis, Eigen::Index shift) {
    MatrixXc output(input.rows(), input.cols());

    if (axis == Axis::Row) {
        const Eigen::Index n = input.rows();
        const Eigen::Index actual = normalized_shift(shift, n);
        for (Eigen::Index row = 0; row < n; ++row) {
            const Eigen::Index src = (row - actual + n) % n;
            output.row(row) = input.row(src);
        }
        return output;
    }

    const Eigen::Index n = input.cols();
    const Eigen::Index actual = normalized_shift(shift, n);
    for (Eigen::Index col = 0; col < n; ++col) {
        const Eigen::Index src = (col - actual + n) % n;
        output.col(col) = input.col(src);
    }
    return output;
}

MatrixXc fft_along_axis(FftEngine& engine, const MatrixXc& input, Axis axis) {
    return engine.fft_along_axis(input, axis);
}

MatrixXc ifft_along_axis(FftEngine& engine, const MatrixXc& input, Axis axis) {
    return engine.ifft_along_axis(input, axis);
}

MatrixXc fft_along_axis(const MatrixXc& input, Axis axis) {
    thread_local FftEngine engine;
    return engine.fft_along_axis(input, axis);
}

MatrixXc ifft_along_axis(const MatrixXc& input, Axis axis) {
    thread_local FftEngine engine;
    return engine.ifft_along_axis(input, axis);
}

VectorXc select_channel_row_as_vector(const MatrixXc& channels, const ChannelSelection& selection) {
    if (selection.mode != ChannelSelectionMode::Single) {
        throw std::invalid_argument("Only ChannelSelectionMode::Single is implemented in stage 1.");
    }
    if (selection.channel_index < 0 || selection.channel_index >= channels.rows()) {
        throw std::out_of_range("channel_index is out of range.");
    }
    return channels.row(selection.channel_index).transpose();
}

MatrixXc reshape_column_major(const VectorXc& input, Eigen::Index rows) {
    if (rows <= 0) {
        throw std::invalid_argument("reshape rows must be positive.");
    }
    const Eigen::Index cols = input.size() / rows;
    if (rows * cols != input.size()) {
        throw std::invalid_argument("input size is not divisible by requested row count.");
    }

    Eigen::Map<const MatrixXc> mapped(input.data(), rows, cols);
    return MatrixXc(mapped);
}

Eigen::MatrixXd magnitude(const MatrixXc& input) {
    return input.cwiseAbs();
}

Eigen::MatrixXd safe_log10_magnitude(const MatrixXc& input, double floor_value) {
    Eigen::MatrixXd mag = magnitude(input);
    mag = mag.unaryExpr([floor_value](double x) { return std::max(x, floor_value); });
    return mag.unaryExpr([](double x) { return std::log10(x); });
}

}  // namespace fft_imaging
