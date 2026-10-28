#pragma once

#include "fft_imaging/types.hpp"

namespace fft_imaging {

VectorXd compute_natural_cubic_spline_node_derivatives(
    const VectorXd& x_axis,
    const VectorXd& values);

double evaluate_cubic_hermite_segment(
    double x0,
    double x1,
    double y0,
    double y1,
    double m0,
    double m1,
    double x_query);

}  // namespace fft_imaging
