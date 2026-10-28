#include "fft_imaging/range_angle_imager.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <stdexcept>

#include "fft_imaging/spline_interp2.hpp"

namespace fft_imaging {

namespace {

constexpr double kPi = 3.14159265358979323846;

double asind_safe(double x) {
    return std::asin(x) * 180.0 / kPi;
}

double mod_matlab(double x, double y) {
    return x - y * std::floor(x / y);
}

Complex bilinear_sample(
    const MatrixXc& input,
    const VectorXd& x_axis,
    const VectorXd& y_axis,
    double x_query,
    double y_query) {
    if (x_axis.size() < 2 || y_axis.size() < 2) {
        return Complex(0.0, 0.0);
    }
    if (!std::isfinite(x_query) || !std::isfinite(y_query)) {
        return Complex(0.0, 0.0);
    }

    if (x_query < x_axis(0) || x_query > x_axis(x_axis.size() - 1) ||
        y_query < y_axis(0) || y_query > y_axis(y_axis.size() - 1)) {
        return Complex(0.0, 0.0);
    }

    const auto x_begin = x_axis.data();
    const auto x_end = x_axis.data() + x_axis.size();
    const auto y_begin = y_axis.data();
    const auto y_end = y_axis.data() + y_axis.size();

    auto x_hi_it = std::lower_bound(x_begin, x_end, x_query);
    auto y_hi_it = std::lower_bound(y_begin, y_end, y_query);

    Eigen::Index x1 = static_cast<Eigen::Index>(std::distance(x_begin, x_hi_it));
    Eigen::Index y1 = static_cast<Eigen::Index>(std::distance(y_begin, y_hi_it));
    Eigen::Index x0 = std::max<Eigen::Index>(0, x1 - 1);
    Eigen::Index y0 = std::max<Eigen::Index>(0, y1 - 1);

    x1 = std::min<Eigen::Index>(x_axis.size() - 1, x1);
    y1 = std::min<Eigen::Index>(y_axis.size() - 1, y1);

    const double xa = x_axis(x0);
    const double xb = x_axis(x1);
    const double ya = y_axis(y0);
    const double yb = y_axis(y1);

    if (x0 == x1 && y0 == y1) {
        return input(y0, x0);
    }
    if (x0 == x1) {
        const double ty = (std::abs(yb - ya) < 1e-12) ? 0.0 : (y_query - ya) / (yb - ya);
        return (1.0 - ty) * input(y0, x0) + ty * input(y1, x0);
    }
    if (y0 == y1) {
        const double tx = (std::abs(xb - xa) < 1e-12) ? 0.0 : (x_query - xa) / (xb - xa);
        return (1.0 - tx) * input(y0, x0) + tx * input(y0, x1);
    }

    const double tx = (x_query - xa) / (xb - xa);
    const double ty = (y_query - ya) / (yb - ya);

    const Complex q00 = input(y0, x0);
    const Complex q01 = input(y0, x1);
    const Complex q10 = input(y1, x0);
    const Complex q11 = input(y1, x1);

    return (1.0 - tx) * (1.0 - ty) * q00 +
           tx * (1.0 - ty) * q01 +
           (1.0 - tx) * ty * q10 +
           tx * ty * q11;
}

std::array<double, 4> cubic_convolution_weights(double t) {
    // Catmull-Rom / Keys cubic convolution (a = -0.5).
    // This is still an approximation to MATLAB's spline kernel, but it is
    // better behaved than the previous 4-point Lagrange polynomial, which
    // overshot badly on the measured remap workload.
    const double t2 = t * t;
    const double t3 = t2 * t;

    return {
        -0.5 * t + 1.0 * t2 - 0.5 * t3,
        1.0 - 2.5 * t2 + 1.5 * t3,
        0.5 * t + 2.0 * t2 - 1.5 * t3,
        -0.5 * t2 + 0.5 * t3,
    };
}

Complex bicubic_sample(
    const MatrixXc& input,
    const VectorXd& x_axis,
    const VectorXd& y_axis,
    double x_query,
    double y_query) {
    if (x_axis.size() < 4 || y_axis.size() < 4) {
        return bilinear_sample(input, x_axis, y_axis, x_query, y_query);
    }
    if (!std::isfinite(x_query) || !std::isfinite(y_query)) {
        return Complex(0.0, 0.0);
    }

    if (x_query < x_axis(0) || x_query > x_axis(x_axis.size() - 1) ||
        y_query < y_axis(0) || y_query > y_axis(y_axis.size() - 1)) {
        return Complex(0.0, 0.0);
    }

    const auto x_begin = x_axis.data();
    const auto x_end = x_axis.data() + x_axis.size();
    const auto y_begin = y_axis.data();
    const auto y_end = y_axis.data() + y_axis.size();

    Eigen::Index x1 = static_cast<Eigen::Index>(std::distance(x_begin, std::lower_bound(x_begin, x_end, x_query)));
    Eigen::Index y1 = static_cast<Eigen::Index>(std::distance(y_begin, std::lower_bound(y_begin, y_end, y_query)));
    x1 = std::clamp<Eigen::Index>(x1, 1, x_axis.size() - 2);
    y1 = std::clamp<Eigen::Index>(y1, 1, y_axis.size() - 2);

    const Eigen::Index x_left = x_axis(x1) >= x_query ? x1 - 1 : x1;
    const Eigen::Index y_left = y_axis(y1) >= y_query ? y1 - 1 : y1;

    Eigen::Index x0 = std::clamp<Eigen::Index>(x_left - 1, 0, x_axis.size() - 4);
    Eigen::Index y0 = std::clamp<Eigen::Index>(y_left - 1, 0, y_axis.size() - 4);

    const double x_den = x_axis(x0 + 2) - x_axis(x0 + 1);
    const double y_den = y_axis(y0 + 2) - y_axis(y0 + 1);
    if (std::abs(x_den) < 1e-12 || std::abs(y_den) < 1e-12) {
        return bilinear_sample(input, x_axis, y_axis, x_query, y_query);
    }

    const double tx = std::clamp((x_query - x_axis(x0 + 1)) / x_den, 0.0, 1.0);
    const double ty = std::clamp((y_query - y_axis(y0 + 1)) / y_den, 0.0, 1.0);
    const auto wx = cubic_convolution_weights(tx);
    const auto wy = cubic_convolution_weights(ty);

    Complex sum(0.0, 0.0);
    for (int yi = 0; yi < 4; ++yi) {
        for (int xi = 0; xi < 4; ++xi) {
            sum += wy[yi] * wx[xi] * input(y0 + yi, x0 + xi);
        }
    }
    return sum;
}
}  // namespace

double derive_lambda_m(double c_mps, double fc_hz) {
    return c_mps / fc_hz;
}

double derive_chirp_rate_hz_per_s(double fs_hz, double c_mps, double rmax_m) {
    return fs_hz * c_mps / (2.0 * rmax_m);
}

RangeAngleGeometry build_range_angle_geometry(
    Eigen::Index nr,
    Eigen::Index na,
    const SceneGridConfig& config,
    double v_mps) {
    if (nr <= 0 || na <= 0) {
        throw std::invalid_argument("nr and na must be positive.");
    }
    if (std::abs(v_mps) < 1e-9) {
        throw std::invalid_argument("v_mps must be non-zero for range-angle remap.");
    }

    const double lambda_m = derive_lambda_m(config.c_mps, config.fc_hz);
    const double chirp_rate = derive_chirp_rate_hz_per_s(config.fs_hz, config.c_mps, config.rmax_m);
    const double rangw_m = config.rmax_m;
    const double azi_width_m = rangw_m;

    RangeAngleGeometry geometry;
    geometry.rr_m.resize(nr);
    geometry.fa_hz.resize(na);

    for (Eigen::Index i = 0; i < nr; ++i) {
        const double one_based = static_cast<double>(i + 1);
        geometry.rr_m(i) = -(one_based / static_cast<double>(nr)) * config.fs_hz / (-chirp_rate) * config.c_mps / 2.0;
    }
    for (Eigen::Index i = 0; i < na; ++i) {
        geometry.fa_hz(i) =
            (static_cast<double>(i) - static_cast<double>(na) / 2.0) / static_cast<double>(na) * config.prf_hz;
    }

    geometry.aq_deg.resize(na);
    for (Eigen::Index i = 0; i < na; ++i) {
        const double sin_q = geometry.fa_hz(i) * lambda_m / (2.0 * v_mps);
        const double sin_qm = mod_matlab(sin_q + 1.0, 2.0) - 1.0;
        geometry.aq_deg(i) = asind_safe(sin_qm) + 180.0 * std::floor((sin_q + 1.0) / 2.0);
    }

    const Eigen::Index nx =
        static_cast<Eigen::Index>(std::floor((2.0 * azi_width_m) / config.grid_spacing_m + 0.5)) + 1;
    const Eigen::Index ny =
        static_cast<Eigen::Index>(std::floor(rangw_m / config.grid_spacing_m + 0.5)) + 1;

    geometry.ax_m.resize(nx);
    geometry.ay_m.resize(ny);
    for (Eigen::Index i = 0; i < nx; ++i) {
        geometry.ax_m(i) = -azi_width_m + static_cast<double>(i) * config.grid_spacing_m;
    }
    for (Eigen::Index i = 0; i < ny; ++i) {
        geometry.ay_m(i) = static_cast<double>(i) * config.grid_spacing_m;
    }

    geometry.aq_grid_deg.resize(ny, nx);
    geometry.ar_grid_m.resize(ny, nx);
    geometry.mask = Eigen::ArrayXXd::Zero(ny, nx);

    for (Eigen::Index row = 0; row < ny; ++row) {
        for (Eigen::Index col = 0; col < nx; ++col) {
            const double ax = geometry.ax_m(col);
            const double ay = geometry.ay_m(row);
            const double raw_ar = std::sqrt(ax * ax + ay * ay);
            geometry.ar_grid_m(row, col) = raw_ar;

            const double aq = (raw_ar > 1e-12)
                ? asind_safe(ax / raw_ar)
                : std::numeric_limits<double>::quiet_NaN();
            geometry.aq_grid_deg(row, col) = aq;

            const bool in_mask =
                raw_ar > 1e-12 &&
                raw_ar <= rangw_m &&
                aq >= (config.squint_angle_deg - config.beam_width_deg / 2.0) &&
                aq <= (config.squint_angle_deg + config.beam_width_deg / 2.0);
            geometry.mask(row, col) = in_mask ? 1.0 : 0.0;
        }
    }

    return geometry;
}

RangeAngleRemapPlan build_range_angle_remap_plan(
    Eigen::Index nr,
    Eigen::Index na,
    const SceneGridConfig& config,
    double v_mps) {
    RangeAngleRemapPlan plan;
    plan.nr = nr;
    plan.na = na;
    plan.v_mps = v_mps;
    plan.config = config;
    plan.geometry = build_range_angle_geometry(nr, na, config, v_mps);
    return plan;
}

void resize_remap_buffers(const RangeAngleRemapPlan& plan, RemapBuffers& buffers) {
    const Eigen::Index rows = plan.geometry.ay_m.size();
    const Eigen::Index cols = plan.geometry.ax_m.size();
    if (buffers.remapped_image.rows() != rows || buffers.remapped_image.cols() != cols) {
        buffers.remapped_image.resize(rows, cols);
    }
    if (buffers.remapped_fliplr_image.rows() != rows || buffers.remapped_fliplr_image.cols() != cols) {
        buffers.remapped_fliplr_image.resize(rows, cols);
    }
}

void remap_fft_to_range_angle(
    const MatrixXc& fft_image,
    const RangeAngleRemapPlan& plan,
    InterpolationMode interpolation_mode,
    RemapBuffers& buffers) {
    if (fft_image.rows() < 2 || fft_image.cols() < 2) {
        throw std::invalid_argument("fft_image must be at least 2x2 for bilinear interpolation.");
    }
    if (fft_image.rows() != plan.nr || fft_image.cols() != plan.na) {
        throw std::invalid_argument("fft_image shape does not match remap plan.");
    }

    resize_remap_buffers(plan, buffers);
    auto& remapped_image = buffers.remapped_image;
    auto& remapped_fliplr_image = buffers.remapped_fliplr_image;
    const auto& geometry = plan.geometry;

    if (interpolation_mode == InterpolationMode::Spline) {
        remapped_image = interp2_spline_zero(
            geometry.aq_deg,
            geometry.rr_m,
            fft_image,
            geometry.aq_grid_deg,
            geometry.ar_grid_m);
        remapped_image.array() *= geometry.mask;

        for (Eigen::Index row = 0; row < remapped_image.rows(); ++row) {
            for (Eigen::Index col = 0; col < remapped_image.cols(); ++col) {
                remapped_fliplr_image(row, col) =
                    remapped_image(row, remapped_image.cols() - 1 - col);
            }
        }
        return;
    }

    for (Eigen::Index row = 0; row < remapped_image.rows(); ++row) {
        for (Eigen::Index col = 0; col < remapped_image.cols(); ++col) {
            const double aq = geometry.aq_grid_deg(row, col);
            const double ar = geometry.ar_grid_m(row, col);
            Complex sampled(0.0, 0.0);
            if (interpolation_mode == InterpolationMode::Bilinear) {
                sampled = bilinear_sample(fft_image, geometry.aq_deg, geometry.rr_m, aq, ar);
            } else if (interpolation_mode == InterpolationMode::Bicubic) {
                sampled = bicubic_sample(fft_image, geometry.aq_deg, geometry.rr_m, aq, ar);
            }

            remapped_image(row, col) = sampled * geometry.mask(row, col);
        }
    }

    for (Eigen::Index row = 0; row < remapped_image.rows(); ++row) {
        for (Eigen::Index col = 0; col < remapped_image.cols(); ++col) {
            remapped_fliplr_image(row, col) =
                remapped_image(row, remapped_image.cols() - 1 - col);
        }
    }
}

RemapResult remap_fft_to_range_angle(
    const MatrixXc& fft_image,
    const SceneGridConfig& config,
    double v_mps,
    InterpolationMode interpolation_mode) {
    const RangeAngleRemapPlan plan =
        build_range_angle_remap_plan(fft_image.rows(), fft_image.cols(), config, v_mps);
    RemapBuffers buffers;
    remap_fft_to_range_angle(fft_image, plan, interpolation_mode, buffers);

    RemapResult result;
    result.geometry = plan.geometry;
    result.remapped_image = std::move(buffers.remapped_image);
    result.remapped_fliplr_image = std::move(buffers.remapped_fliplr_image);
    return result;
}

}  // namespace fft_imaging
