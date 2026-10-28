#pragma once

#include "fft_imaging/fft_engine.hpp"
#include "fft_imaging/session.hpp"

namespace fft_imaging {

class CorePipeline {
public:
    explicit CorePipeline(
        ImagingConfig imaging_config,
        PipelineConfig pipeline_config = {});

    PipelineArtifacts run_fft_instant_imaging(
        const RawCaptureSession& session,
        const FrameSelection& frame_selection) const;

    PipelineArtifacts run_fft_instant_imaging_frame(
        const MatrixXc& frame_echo,
        const FrameSelection& frame_selection) const;

private:
    ImagingConfig imaging_config_;
    PipelineConfig pipeline_config_;
    mutable FftEngine fft_engine_;
};

}  // namespace fft_imaging
