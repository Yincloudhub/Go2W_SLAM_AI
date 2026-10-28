#pragma once

#include <memory>

#include "fft_imaging/range_angle_imager.hpp"

namespace fft_imaging {

enum class BackendKind {
    CpuSerial = 0,
    CpuParallel = 1,
    Cuda = 2,
};

class ImagingBackend {
public:
    virtual ~ImagingBackend() = default;

    virtual BackendKind kind() const = 0;
    virtual const char* name() const = 0;

    virtual void remap_to_range_angle(
        const MatrixXc& fft_image,
        const RangeAngleRemapPlan& plan,
        InterpolationMode interpolation_mode,
        RemapBuffers& buffers) const = 0;
};

class CpuImagingBackend final : public ImagingBackend {
public:
    explicit CpuImagingBackend(BackendKind kind = BackendKind::CpuSerial);

    BackendKind kind() const override;
    const char* name() const override;

    void remap_to_range_angle(
        const MatrixXc& fft_image,
        const RangeAngleRemapPlan& plan,
        InterpolationMode interpolation_mode,
        RemapBuffers& buffers) const override;

private:
    BackendKind kind_;
};

std::unique_ptr<ImagingBackend> make_imaging_backend(BackendKind kind);

}  // namespace fft_imaging
