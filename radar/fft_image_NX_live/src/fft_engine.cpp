#include "fft_imaging/fft_engine.hpp"

#include <stdexcept>
#include <utility>
#include <vector>

#include <fftw3.h>

namespace fft_imaging {

struct FftEngine::PlanBundle {
    explicit PlanBundle(int length)
        : n(length),
          in(static_cast<std::size_t>(length)),
          out(static_cast<std::size_t>(length)) {
        forward_plan = fftw_plan_dft_1d(n, in.data(), out.data(), FFTW_FORWARD, FFTW_ESTIMATE);
        backward_plan = fftw_plan_dft_1d(n, in.data(), out.data(), FFTW_BACKWARD, FFTW_ESTIMATE);
        if (forward_plan == nullptr || backward_plan == nullptr) {
            if (forward_plan != nullptr) {
                fftw_destroy_plan(forward_plan);
            }
            if (backward_plan != nullptr) {
                fftw_destroy_plan(backward_plan);
            }
            throw std::runtime_error("Failed to create FFTW plans.");
        }
    }

    ~PlanBundle() {
        if (forward_plan != nullptr) {
            fftw_destroy_plan(forward_plan);
        }
        if (backward_plan != nullptr) {
            fftw_destroy_plan(backward_plan);
        }
    }

    int n = 0;
    std::vector<fftw_complex> in;
    std::vector<fftw_complex> out;
    fftw_plan forward_plan = nullptr;
    fftw_plan backward_plan = nullptr;
};

FftEngine::FftEngine() = default;
FftEngine::~FftEngine() = default;

FftEngine::FftEngine(FftEngine&&) noexcept = default;
FftEngine& FftEngine::operator=(FftEngine&&) noexcept = default;

MatrixXc FftEngine::fft_along_axis(const MatrixXc& input, Axis axis) {
    return apply_fft(input, axis, false);
}

MatrixXc FftEngine::ifft_along_axis(const MatrixXc& input, Axis axis) {
    return apply_fft(input, axis, true);
}

std::size_t FftEngine::cached_plan_count() const {
    return plans_.size();
}

FftEngine::PlanBundle& FftEngine::get_plan_bundle(int n) {
    auto it = plans_.find(n);
    if (it == plans_.end()) {
        auto inserted = plans_.emplace(n, std::make_unique<PlanBundle>(n));
        it = inserted.first;
    }
    return *(it->second);
}

MatrixXc FftEngine::apply_fft(const MatrixXc& input, Axis axis, bool inverse) {
    MatrixXc output(input.rows(), input.cols());

    if (axis == Axis::Row) {
        const int n = static_cast<int>(input.rows());
        auto& bundle = get_plan_bundle(n);
        for (Eigen::Index col = 0; col < input.cols(); ++col) {
            for (Eigen::Index row = 0; row < input.rows(); ++row) {
                bundle.in[static_cast<std::size_t>(row)][0] = input(row, col).real();
                bundle.in[static_cast<std::size_t>(row)][1] = input(row, col).imag();
            }

            fftw_execute(inverse ? bundle.backward_plan : bundle.forward_plan);

            for (Eigen::Index row = 0; row < input.rows(); ++row) {
                Complex value(
                    bundle.out[static_cast<std::size_t>(row)][0],
                    bundle.out[static_cast<std::size_t>(row)][1]);
                if (inverse) {
                    value /= static_cast<double>(n);
                }
                output(row, col) = value;
            }
        }
        return output;
    }

    const int n = static_cast<int>(input.cols());
    auto& bundle = get_plan_bundle(n);
    for (Eigen::Index row = 0; row < input.rows(); ++row) {
        for (Eigen::Index col = 0; col < input.cols(); ++col) {
            bundle.in[static_cast<std::size_t>(col)][0] = input(row, col).real();
            bundle.in[static_cast<std::size_t>(col)][1] = input(row, col).imag();
        }

        fftw_execute(inverse ? bundle.backward_plan : bundle.forward_plan);

        for (Eigen::Index col = 0; col < input.cols(); ++col) {
            Complex value(
                bundle.out[static_cast<std::size_t>(col)][0],
                bundle.out[static_cast<std::size_t>(col)][1]);
            if (inverse) {
                value /= static_cast<double>(n);
            }
            output(row, col) = value;
        }
    }
    return output;
}

}  // namespace fft_imaging
