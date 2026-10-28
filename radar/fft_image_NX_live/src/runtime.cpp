#include "fft_imaging/runtime.hpp"

#include <algorithm>
#include <cmath>
#include <sstream>
#include <stdexcept>

namespace fft_imaging {

void NullFrameOutputSink::write_frame(
    std::size_t,
    const MatrixXc&) {}

void NullRuntimeStatusSink::on_event(const RuntimeEvent&) {}

ImagingRuntime::ImagingRuntime(
    AppConfig app_config,
    std::unique_ptr<ImagingBackend> backend)
    : app_config_(std::move(app_config)),
      backend_(backend ? std::move(backend) : make_imaging_backend(app_config_.imaging.backend)) {}

RuntimeSummary ImagingRuntime::run(
    const RawCaptureSession& session,
    const FrameVelocityProvider& velocity_provider,
    FrameOutputSink& output_sink,
    Profiler* profiler,
    Logger* logger,
    RuntimeStatusSink* status_sink) const {
    Profiler local_profiler;
    NullLogger local_logger;
    NullRuntimeStatusSink local_status_sink;
    if (profiler == nullptr) {
        local_profiler.reset();
        profiler = &local_profiler;
    }
    if (logger == nullptr) {
        logger = &local_logger;
    }
    if (status_sink == nullptr) {
        status_sink = &local_status_sink;
    }

    switch (app_config_.runtime.mode) {
        case RuntimeMode::Offline:
            return run_offline(session, velocity_provider, output_sink, *profiler, *logger, *status_sink);
        case RuntimeMode::Realtime:
            return run_realtime(session, velocity_provider, output_sink, *profiler, *logger, *status_sink);
    }
    throw std::invalid_argument("Unknown runtime mode.");
}

RuntimeSummary ImagingRuntime::run_offline(
    const RawCaptureSession& session,
    const FrameVelocityProvider& velocity_provider,
    FrameOutputSink& output_sink,
    Profiler& profiler,
    Logger& logger,
    RuntimeStatusSink& status_sink) const {
    return run_finite_frame_loop(
        RuntimeMode::Offline,
        session,
        velocity_provider,
        output_sink,
        profiler,
        logger,
        status_sink);
}

RuntimeSummary ImagingRuntime::run_realtime(
    const RawCaptureSession& session,
    const FrameVelocityProvider& velocity_provider,
    FrameOutputSink& output_sink,
    Profiler& profiler,
    Logger& logger,
    RuntimeStatusSink& status_sink) const {
    // Current deployable realtime path still consumes a finite session, but uses a
    // distinct entry point so future stream readers / CUDA queues do not reshape the API.
    return run_finite_frame_loop(
        RuntimeMode::Realtime,
        session,
        velocity_provider,
        output_sink,
        profiler,
        logger,
        status_sink);
}

RuntimeSummary ImagingRuntime::run_finite_frame_loop(
    RuntimeMode mode,
    const RawCaptureSession& session,
    const FrameVelocityProvider& velocity_provider,
    FrameOutputSink& output_sink,
    Profiler& profiler,
    Logger& logger,
    RuntimeStatusSink& status_sink) const {
    ImagingConfig imaging_config;
    imaging_config.nr = app_config_.fft_pipeline.nr;
    imaging_config.channel_selection = app_config_.imaging.channel_selection;

    PipelineConfig pipeline_config;
    pipeline_config.mode = PipelineMode::Runtime;
    CorePipeline pipeline(imaging_config, pipeline_config);

    const std::size_t session_frames =
        session.total_frames(
            app_config_.fft_pipeline.frame_selection.frag,
            app_config_.fft_pipeline.frame_selection.step);
    std::size_t planned_frames = std::min(session_frames, velocity_provider.available_frames());
    if (app_config_.runtime.max_frames > 0) {
        planned_frames = std::min(planned_frames, app_config_.runtime.max_frames);
    }
    if (planned_frames == 0) {
        throw std::runtime_error("No frames available for runtime execution.");
    }

    {
        std::ostringstream message;
        message << "starting run: mode="
                << (mode == RuntimeMode::Offline ? "offline" : "realtime")
                << ", frames_planned=" << planned_frames
                << ", backend=" << backend_->name();
        logger.log({LogLevel::Info, "runtime", message.str()});
        status_sink.on_event({RuntimeEventKind::RunStarted, RuntimeStage::Read, 0, message.str()});
    }

    FrameSelection frame_selection = app_config_.fft_pipeline.frame_selection;
    RangeAngleRemapPlan remap_plan;
    RemapBuffers remap_buffers;
    bool has_remap_plan = false;
    std::size_t frames_failed = 0;
    std::size_t consecutive_failures = 0;

    for (std::size_t frame = 0; frame < planned_frames; ++frame) {
        try {
            {
                std::ostringstream message;
                message << "frame " << frame << " started";
                logger.log({LogLevel::Info, "runtime", message.str()});
                status_sink.on_event({RuntimeEventKind::FrameStarted, RuntimeStage::Read, frame, message.str()});
            }

            frame_selection.frame_index = frame;
            const auto artifacts = pipeline.run_fft_instant_imaging(session, frame_selection);
            profiler.add_sample(StageKind::RangeFft, artifacts.timings.range_fft_ms);
            profiler.add_sample(StageKind::AzimuthFft, artifacts.timings.azimuth_fft_ms);

            const double v_mps = velocity_provider.velocity_for_frame(frame);
            if (!has_remap_plan || std::abs(remap_plan.v_mps - v_mps) > 1e-12) {
                remap_plan = build_range_angle_remap_plan(
                    artifacts.fft2_after_azimuth_fft.rows(),
                    artifacts.fft2_after_azimuth_fft.cols(),
                    app_config_.imaging.scene_grid,
                    v_mps);
                resize_remap_buffers(remap_plan, remap_buffers);
                has_remap_plan = true;
            }

            {
                auto stage = profiler.scoped(StageKind::Remap);
                backend_->remap_to_range_angle(
                    artifacts.fft2_after_azimuth_fft,
                    remap_plan,
                    app_config_.imaging.interpolation_mode,
                    remap_buffers);
            }

            {
                auto stage = profiler.scoped(StageKind::Write);
                output_sink.write_frame(frame, remap_buffers.remapped_fliplr_image);
            }

            consecutive_failures = 0;
            {
                std::ostringstream message;
                message << "frame " << frame << " completed";
                logger.log({LogLevel::Info, "runtime", message.str()});
                status_sink.on_event({RuntimeEventKind::FrameCompleted, RuntimeStage::Write, frame, message.str()});
            }
        } catch (const std::exception& ex) {
            ++frames_failed;
            ++consecutive_failures;
            {
                std::ostringstream message;
                message << "frame " << frame << " failed: " << ex.what();
                logger.log({LogLevel::Error, "runtime", message.str()});
                status_sink.on_event({RuntimeEventKind::FrameFailed, RuntimeStage::Write, frame, message.str()});
            }
            if (!app_config_.runtime.continue_on_frame_error ||
                consecutive_failures >= app_config_.runtime.max_consecutive_frame_errors) {
                throw;
            }
        }
    }

    RuntimeSummary summary;
    summary.mode = mode;
    summary.frames_processed = planned_frames - frames_failed;
    summary.frames_planned = planned_frames;
    summary.frames_failed = frames_failed;
    summary.backend_name = backend_->name();
    {
        std::ostringstream message;
        message << "run completed: processed=" << summary.frames_processed
                << ", failed=" << summary.frames_failed
                << ", planned=" << summary.frames_planned;
        logger.log({LogLevel::Info, "runtime", message.str()});
        status_sink.on_event({RuntimeEventKind::RunCompleted, RuntimeStage::Write, 0, message.str()});
    }
    return summary;
}

}  // namespace fft_imaging
