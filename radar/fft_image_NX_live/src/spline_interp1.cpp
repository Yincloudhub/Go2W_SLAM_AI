#include "fft_imaging/spline_interp1.hpp"

#include <cmath>
#include <stdexcept>
#include <vector>

namespace fft_imaging {

namespace {

VectorXd solve_tridiagonal(
    const VectorXd& lower,
    const VectorXd& diag,
    const VectorXd& upper,
    const VectorXd& rhs) {
    const Eigen::Index n = diag.size();
    if (rhs.size() != n) {
        throw std::invalid_argument("Tridiagonal rhs size mismatch.");
    }
    if (n == 0) {
        return VectorXd();
    }
    if (lower.size() != std::max<Eigen::Index>(0, n - 1) ||
        upper.size() != std::max<Eigen::Index>(0, n - 1)) {
        throw std::invalid_argument("Tridiagonal band size mismatch.");
    }

    VectorXd cprime = VectorXd::Zero(std::max<Eigen::Index>(0, n - 1));
    VectorXd dprime(n);

    double denom = diag(0);
    if (std::abs(denom) < 1e-12) {
        throw std::runtime_error("Degenerate tridiagonal system.");
    }
    if (n > 1) {
        cprime(0) = upper(0) / denom;
    }
    dprime(0) = rhs(0) / denom;

    for (Eigen::Index i = 1; i < n; ++i) {
        denom = diag(i) - lower(i - 1) * cprime(i - 1);
        if (std::abs(denom) < 1e-12) {
            throw std::runtime_error("Degenerate tridiagonal system.");
        }
        if (i < n - 1) {
            cprime(i) = upper(i) / denom;
        }
        dprime(i) = (rhs(i) - lower(i - 1) * dprime(i - 1)) / denom;
    }

    VectorXd x(n);
    x(n - 1) = dprime(n - 1);
    for (Eigen::Index i = n - 2; i >= 0; --i) {
        x(i) = dprime(i) - cprime(i) * x(i + 1);
        if (i == 0) {
            break;
        }
    }
    return x;
}

}  // namespace

VectorXd compute_natural_cubic_spline_node_derivatives(
    const VectorXd& x_axis,
    const VectorXd& values) {
    if (x_axis.size() != values.size()) {
        throw std::invalid_argument("x_axis and values size mismatch.");
    }
    const Eigen::Index n = x_axis.size();
    if (n < 2) {
        throw std::invalid_argument("Need at least 2 points for spline interpolation.");
    }
    if (n == 2) {
        const double dx = x_axis(1) - x_axis(0);
        if (dx <= 0.0) {
            throw std::invalid_argument("x_axis must be strictly increasing.");
        }
        const double slope = (values(1) - values(0)) / dx;
        VectorXd deriv(2);
        deriv << slope, slope;
        return deriv;
    }

    VectorXd h(n - 1);
    VectorXd delta(n - 1);
    for (Eigen::Index i = 0; i < n - 1; ++i) {
        h(i) = x_axis(i + 1) - x_axis(i);
        if (h(i) <= 0.0) {
            throw std::invalid_argument("x_axis must be strictly increasing.");
        }
        delta(i) = (values(i + 1) - values(i)) / h(i);
    }

    const Eigen::Index m_size = n - 2;
    VectorXd second = VectorXd::Zero(n);
    if (m_size > 0) {
        VectorXd lower(std::max<Eigen::Index>(0, m_size - 1));
        VectorXd diag(m_size);
        VectorXd upper(std::max<Eigen::Index>(0, m_size - 1));
        VectorXd rhs(m_size);

        for (Eigen::Index i = 0; i < m_size; ++i) {
            const Eigen::Index node = i + 1;
            diag(i) = 2.0 * (h(node - 1) + h(node));
            rhs(i) = 6.0 * (delta(node) - delta(node - 1));
            if (i > 0) {
                lower(i - 1) = h(node - 1);
            }
            if (i < m_size - 1) {
                upper(i) = h(node);
            }
        }

        const VectorXd inner_second = solve_tridiagonal(lower, diag, upper, rhs);
        for (Eigen::Index i = 0; i < m_size; ++i) {
            second(i + 1) = inner_second(i);
        }
    }

    VectorXd derivatives(n);
    derivatives(0) = delta(0) - h(0) * (2.0 * second(0) + second(1)) / 6.0;
    for (Eigen::Index i = 1; i < n - 1; ++i) {
        derivatives(i) = delta(i - 1) + h(i - 1) * (second(i - 1) + 2.0 * second(i)) / 6.0;
    }
    derivatives(n - 1) = delta(n - 2) + h(n - 2) * (second(n - 2) + 2.0 * second(n - 1)) / 6.0;
    return derivatives;
}

double evaluate_cubic_hermite_segment(
    double x0,
    double x1,
    double y0,
    double y1,
    double m0,
    double m1,
    double x_query) {
    const double h = x1 - x0;
    if (h <= 0.0) {
        throw std::invalid_argument("evaluate_cubic_hermite_segment requires x1 > x0.");
    }
    const double t = (x_query - x0) / h;
    const double t2 = t * t;
    const double t3 = t2 * t;

    const double h00 = 2.0 * t3 - 3.0 * t2 + 1.0;
    const double h10 = t3 - 2.0 * t2 + t;
    const double h01 = -2.0 * t3 + 3.0 * t2;
    const double h11 = t3 - t2;
    return h00 * y0 + h10 * h * m0 + h01 * y1 + h11 * h * m1;
}

}  // namespace fft_imaging
