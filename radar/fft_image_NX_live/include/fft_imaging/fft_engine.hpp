#pragma once

#include <cstddef>
#include <memory>
#include <unordered_map>

#include "fft_imaging/types.hpp"

struct fftw_plan_s;

namespace fft_imaging {

class FftEngine {
public:
    FftEngine();
    ~FftEngine();

    FftEngine(const FftEngine&) = delete;
    FftEngine& operator=(const FftEngine&) = delete;

    FftEngine(FftEngine&&) noexcept;
    FftEngine& operator=(FftEngine&&) noexcept;

    MatrixXc fft_along_axis(const MatrixXc& input, Axis axis);
    MatrixXc ifft_along_axis(const MatrixXc& input, Axis axis);

    std::size_t cached_plan_count() const;

private:
    struct PlanBundle;

    MatrixXc apply_fft(const MatrixXc& input, Axis axis, bool inverse);
    PlanBundle& get_plan_bundle(int n);

    std::unordered_map<int, std::unique_ptr<PlanBundle>> plans_;
};

}  // namespace fft_imaging
