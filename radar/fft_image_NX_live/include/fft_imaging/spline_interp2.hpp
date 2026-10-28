#pragma once

#include "fft_imaging/types.hpp"

namespace fft_imaging {

MatrixXc interp2_spline_zero(
    const VectorXd& x_axis,
    const VectorXd& y_axis,
    const MatrixXc& values,
    const Eigen::MatrixXd& x_query,
    const Eigen::MatrixXd& y_query);

Complex interp2_spline_zero_sample(
    const VectorXd& x_axis,
    const VectorXd& y_axis,
    const MatrixXc& values,
    double x_query,
    double y_query);

}  // namespace fft_imaging
