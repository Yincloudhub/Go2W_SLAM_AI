#pragma once

#include "fft_imaging/fft_engine.hpp"
#include "fft_imaging/types.hpp"

namespace fft_imaging {

MatrixXc fftshift(const MatrixXc& input, Axis axis);
MatrixXc circshift(const MatrixXc& input, Axis axis, Eigen::Index shift);

MatrixXc fft_along_axis(FftEngine& engine, const MatrixXc& input, Axis axis);
MatrixXc ifft_along_axis(FftEngine& engine, const MatrixXc& input, Axis axis);
MatrixXc fft_along_axis(const MatrixXc& input, Axis axis);
MatrixXc ifft_along_axis(const MatrixXc& input, Axis axis);

VectorXc select_channel_row_as_vector(const MatrixXc& channels, const ChannelSelection& selection);
MatrixXc reshape_column_major(const VectorXc& input, Eigen::Index rows);

Eigen::MatrixXd magnitude(const MatrixXc& input);
Eigen::MatrixXd safe_log10_magnitude(const MatrixXc& input, double floor_value = 1e-12);

}  // namespace fft_imaging
