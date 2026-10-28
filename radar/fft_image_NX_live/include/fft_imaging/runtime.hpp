#pragma once

#include <cstddef>
#include <memory>
#include <string>

#include "fft_imaging/backend.hpp"
#include "fft_imaging/config.hpp"
#include "fft_imaging/core_pipeline.hpp"
#include "fft_imaging/logging.hpp"
#include "fft_imaging/profiler.hpp"

namespace fft_imaging {

class FrameVelocityProvider {
public:
    virtual ~FrameVelocityProvider() = default;

    virtual std::size_t available_frames() const = 0;
    virtual double velocity_for_frame(std::size_t frame_index) const = 0;
};

class FrameOutputSink {
public:
    virtual ~FrameOutputSink() = default;

    virtual void write_frame(
        std::size_t frame_index,
        const MatrixXc& remapped_fliplr_image) = 0;
};

class NullFrameOutputSink final : public FrameOutputSink {
public:
    void write_frame(
        std::size_t frame_index,
        const MatrixXc& remapped_fliplr_image) override;
};

struct RuntimeSummary {
    RuntimeMode mode = RuntimeMode::Offline;
    std::size_t frames_processed = 0;
    std::size_t frames_planned = 0;
    std::size_t frames_failed = 0;
    const char* backend_name = "unknown";
};

enum class RuntimeStage {
    Read = 0,
    RangeFft = 1,
    AzimuthFft = 2,
    Remap = 3,
    Write = 4,
};

enum class RuntimeEventKind {
    RunStarted = 0,
    FrameStarted = 1,
    FrameCompleted = 2,
    FrameFailed = 3,
    RunCompleted = 4,
};

struct RuntimeEvent {
    RuntimeEventKind kind = RuntimeEventKind::RunStarted;
    RuntimeStage stage = RuntimeStage::Read;
    std::size_t frame_index = 0;
    std::string message;
};

class RuntimeStatusSink {
public:
    virtual ~RuntimeStatusSink() = default;
    virtual void on_event(const RuntimeEvent& event) = 0;
};

class NullRuntimeStatusSink final : public RuntimeStatusSink {
public:
    void on_event(const RuntimeEvent& event) override;
};

class ImagingRuntime {
public:
    explicit ImagingRuntime(
        AppConfig app_config,
        std::unique_ptr<ImagingBackend> backend = {});

    RuntimeSummary run(
        const RawCaptureSession& session,
        const FrameVelocityProvider& velocity_provider,
        FrameOutputSink& output_sink,
        Profiler* profiler = nullptr,
        Logger* logger = nullptr,
        RuntimeStatusSink* status_sink = nullptr) const;

private:
    RuntimeSummary run_offline(
        const RawCaptureSession& session,
        const FrameVelocityProvider& velocity_provider,
        FrameOutputSink& output_sink,
        Profiler& profiler,
        Logger& logger,
        RuntimeStatusSink& status_sink) const;

    RuntimeSummary run_realtime(
        const RawCaptureSession& session,
        const FrameVelocityProvider& velocity_provider,
        FrameOutputSink& output_sink,
        Profiler& profiler,
        Logger& logger,
        RuntimeStatusSink& status_sink) const;

    RuntimeSummary run_finite_frame_loop(
        RuntimeMode mode,
        const RawCaptureSession& session,
        const FrameVelocityProvider& velocity_provider,
        FrameOutputSink& output_sink,
        Profiler& profiler,
        Logger& logger,
        RuntimeStatusSink& status_sink) const;

    AppConfig app_config_;
    std::unique_ptr<ImagingBackend> backend_;
};

}  // namespace fft_imaging
