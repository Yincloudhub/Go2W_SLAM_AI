#include "fft_imaging/backend.hpp"

#include <stdexcept>

namespace fft_imaging {

CpuImagingBackend::CpuImagingBackend(BackendKind kind) : kind_(kind) {}

BackendKind CpuImagingBackend::kind() const {
    return kind_;
}

const char* CpuImagingBackend::name() const {
    switch (kind_) {
        case BackendKind::CpuSerial:
            return "cpu_serial";
        case BackendKind::CpuParallel:
            return "cpu_parallel_reserved";
        case BackendKind::Cuda:
            return "cuda_reserved";
    }
    return "cpu_unknown";
}

void CpuImagingBackend::remap_to_range_angle(
    const MatrixXc& fft_image,
    const RangeAngleRemapPlan& plan,
    InterpolationMode interpolation_mode,
    RemapBuffers& buffers) const {
    remap_fft_to_range_angle(fft_image, plan, interpolation_mode, buffers);
}

std::unique_ptr<ImagingBackend> make_imaging_backend(BackendKind kind) {
    switch (kind) {
        case BackendKind::CpuSerial:
        case BackendKind::CpuParallel:
            return std::make_unique<CpuImagingBackend>(kind);
        case BackendKind::Cuda:
            throw std::invalid_argument("CUDA backend is not implemented yet.");
    }
    throw std::invalid_argument("Unknown backend kind.");
}

}  // namespace fft_imaging
