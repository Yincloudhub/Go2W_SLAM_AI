#include "fft_imaging/spline_interp2.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <stdexcept>

#include "fft_imaging/spline_interp1.hpp"

namespace fft_imaging {

namespace {

struct RealSplineGrid2D {
    VectorXd x_axis;
    VectorXd y_axis;
    Eigen::MatrixXd values;
    Eigen::MatrixXd fx;
    Eigen::MatrixXd fy;
    Eigen::MatrixXd fxy;
};

Eigen::Index find_interval(const VectorXd& axis, double query) {
    const auto begin = axis.data();
    const auto end = axis.data() + axis.size();
    auto it = std::upper_bound(begin, end, query);
    if (it == begin || it == end) {
        return -1;
    }
    const Eigen::Index idx = static_cast<Eigen::Index>(std::distance(begin, it) - 1);
    if (idx < 0 || idx >= axis.size() - 1) {
        return -1;
    }
    return idx;
}

RealSplineGrid2D build_real_spline_grid(
    const VectorXd& x_axis,
    const VectorXd& y_axis,
    const Eigen::MatrixXd& values) {
    if (values.rows() != y_axis.size() || values.cols() != x_axis.size()) {
        throw std::invalid_argument("RealSplineGrid2D shape mismatch.");
    }

    RealSplineGrid2D grid;
    grid.x_axis = x_axis;
    grid.y_axis = y_axis;
    grid.values = values;
    grid.fx.resize(values.rows(), values.cols());
    grid.fy.resize(values.rows(), values.cols());
    grid.fxy.resize(values.rows(), values.cols());

    for (Eigen::Index row = 0; row < values.rows(); ++row) {
        const VectorXd deriv = compute_natural_cubic_spline_node_derivatives(
            x_axis,
            values.row(row).transpose());
        grid.fx.row(row) = deriv.transpose();
    }

    for (Eigen::Index col = 0; col < values.cols(); ++col) {
        const VectorXd deriv = compute_natural_cubic_spline_node_derivatives(
            y_axis,
            values.col(col));
        grid.fy.col(col) = deriv;
    }

    for (Eigen::Index col = 0; col < values.cols(); ++col) {
        const VectorXd deriv = compute_natural_cubic_spline_node_derivatives(
            y_axis,
            grid.fx.col(col));
        grid.fxy.col(col) = deriv;
    }

    return grid;
}

double evaluate_bicubic_hermite_patch(
    const RealSplineGrid2D& grid,
    Eigen::Index x_idx,
    Eigen::Index y_idx,
    double x_query,
    double y_query) {
    const double x0 = grid.x_axis(x_idx);
    const double x1 = grid.x_axis(x_idx + 1);
    const double y0 = grid.y_axis(y_idx);
    const double y1 = grid.y_axis(y_idx + 1);
    const double hx = x1 - x0;
    const double hy = y1 - y0;
    if (hx <= 0.0 || hy <= 0.0) {
        throw std::runtime_error("Invalid spline cell dimensions.");
    }

    const double tx = (x_query - x0) / hx;
    const double ty = (y_query - y0) / hy;

    Eigen::Matrix4d H;
    H << 1.0, 0.0, 0.0, 0.0,
         0.0, 0.0, 1.0, 0.0,
        -3.0, 3.0, -2.0, -1.0,
         2.0, -2.0, 1.0, 1.0;

    Eigen::Matrix4d P;
    P <<
        grid.values(y_idx, x_idx),      grid.values(y_idx + 1, x_idx),      grid.fy(y_idx, x_idx) * hy,      grid.fy(y_idx + 1, x_idx) * hy,
        grid.values(y_idx, x_idx + 1),  grid.values(y_idx + 1, x_idx + 1),  grid.fy(y_idx, x_idx + 1) * hy,  grid.fy(y_idx + 1, x_idx + 1) * hy,
        grid.fx(y_idx, x_idx) * hx,     grid.fx(y_idx + 1, x_idx) * hx,     grid.fxy(y_idx, x_idx) * hx * hy,     grid.fxy(y_idx + 1, x_idx) * hx * hy,
        grid.fx(y_idx, x_idx + 1) * hx, grid.fx(y_idx + 1, x_idx + 1) * hx, grid.fxy(y_idx, x_idx + 1) * hx * hy, grid.fxy(y_idx + 1, x_idx + 1) * hx * hy;

    const Eigen::Matrix4d coeffs = H * P * H.transpose();
    Eigen::RowVector4d xt;
    xt << 1.0, tx, tx * tx, tx * tx * tx;
    Eigen::Vector4d yt;
    yt << 1.0, ty, ty * ty, ty * ty * ty;
    return (xt * coeffs * yt)(0, 0);
}

double interp2_spline_zero_sample_real(
    const RealSplineGrid2D& grid,
    double x_query,
    double y_query) {
    if (!std::isfinite(x_query) || !std::isfinite(y_query)) {
        return 0.0;
    }
    if (x_query < grid.x_axis(0) || x_query > grid.x_axis(grid.x_axis.size() - 1) ||
        y_query < grid.y_axis(0) || y_query > grid.y_axis(grid.y_axis.size() - 1)) {
        return 0.0;
    }

    const Eigen::Index x_idx = find_interval(grid.x_axis, x_query);
    const Eigen::Index y_idx = find_interval(grid.y_axis, y_query);
    if (x_idx < 0 || y_idx < 0) {
        return 0.0;
    }
    return evaluate_bicubic_hermite_patch(grid, x_idx, y_idx, x_query, y_query);
}

}  // namespace

Complex interp2_spline_zero_sample(
    const VectorXd& x_axis,
    const VectorXd& y_axis,
    const MatrixXc& values,
    double x_query,
    double y_query) {
    const RealSplineGrid2D real_grid = build_real_spline_grid(x_axis, y_axis, values.real());
    const RealSplineGrid2D imag_grid = build_real_spline_grid(x_axis, y_axis, values.imag());
    return Complex(
        interp2_spline_zero_sample_real(real_grid, x_query, y_query),
        interp2_spline_zero_sample_real(imag_grid, x_query, y_query));
}

MatrixXc interp2_spline_zero(
    const VectorXd& x_axis,
    const VectorXd& y_axis,
    const MatrixXc& values,
    const Eigen::MatrixXd& x_query,
    const Eigen::MatrixXd& y_query) {
    if (x_query.rows() != y_query.rows() || x_query.cols() != y_query.cols()) {
        throw std::invalid_argument("x_query and y_query shape mismatch.");
    }

    const RealSplineGrid2D real_grid = build_real_spline_grid(x_axis, y_axis, values.real());
    const RealSplineGrid2D imag_grid = build_real_spline_grid(x_axis, y_axis, values.imag());

    MatrixXc output(x_query.rows(), x_query.cols());
    for (Eigen::Index row = 0; row < x_query.rows(); ++row) {
        for (Eigen::Index col = 0; col < x_query.cols(); ++col) {
            output(row, col) = Complex(
                interp2_spline_zero_sample_real(real_grid, x_query(row, col), y_query(row, col)),
                interp2_spline_zero_sample_real(imag_grid, x_query(row, col), y_query(row, col)));
        }
    }
    return output;
}

}  // namespace fft_imaging
