#include "fft_imaging/profiler.hpp"

#include <iomanip>
#include <sstream>

namespace fft_imaging {

namespace {

std::vector<StageTiming> make_default_timings() {
    return {
        {StageKind::Read, 0, 0.0},
        {StageKind::RangeFft, 0, 0.0},
        {StageKind::AzimuthFft, 0, 0.0},
        {StageKind::Remap, 0, 0.0},
        {StageKind::Write, 0, 0.0},
    };
}

}  // namespace

Profiler::ScopedStage::ScopedStage(Profiler& profiler, StageKind stage)
    : profiler_(&profiler),
      stage_(stage),
      start_(std::chrono::steady_clock::now()) {}

Profiler::ScopedStage::~ScopedStage() {
    if (profiler_ == nullptr) {
        return;
    }
    const auto end = std::chrono::steady_clock::now();
    const double elapsed_ms =
        std::chrono::duration<double, std::milli>(end - start_).count();
    profiler_->add_sample(stage_, elapsed_ms);
}

Profiler::ScopedStage Profiler::scoped(StageKind stage) {
    return ScopedStage(*this, stage);
}

void Profiler::add_sample(StageKind stage, double elapsed_ms) {
    if (timings_.empty()) {
        timings_ = make_default_timings();
    }
    for (auto& timing : timings_) {
        if (timing.stage == stage) {
            timing.calls += 1;
            timing.total_ms += elapsed_ms;
            return;
        }
    }
    timings_.push_back({stage, 1, elapsed_ms});
}

const std::vector<StageTiming>& Profiler::timings() const {
    return timings_;
}

double Profiler::total_ms() const {
    double total = 0.0;
    for (const auto& timing : timings_) {
        total += timing.total_ms;
    }
    return total;
}

void Profiler::reset() {
    timings_ = make_default_timings();
}

const char* stage_kind_name(StageKind stage) {
    switch (stage) {
        case StageKind::Read:
            return "read";
        case StageKind::RangeFft:
            return "range_fft";
        case StageKind::AzimuthFft:
            return "azimuth_fft";
        case StageKind::Remap:
            return "remap";
        case StageKind::Write:
            return "write";
    }
    return "unknown";
}

std::string format_profiler_report(const Profiler& profiler) {
    std::ostringstream out;
    out << std::fixed << std::setprecision(3);
    out << "stage timings (ms):\n";
    for (const auto& timing : profiler.timings()) {
        const double average_ms = timing.calls == 0
            ? 0.0
            : timing.total_ms / static_cast<double>(timing.calls);
        out << "  " << stage_kind_name(timing.stage)
            << ": total=" << timing.total_ms
            << ", calls=" << timing.calls
            << ", avg=" << average_ms << "\n";
    }
    out << "  total=" << profiler.total_ms();
    return out.str();
}

}  // namespace fft_imaging
