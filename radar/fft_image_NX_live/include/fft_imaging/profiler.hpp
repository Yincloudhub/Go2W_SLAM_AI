#pragma once

#include <chrono>
#include <cstddef>
#include <string>
#include <vector>

namespace fft_imaging {

enum class StageKind {
    Read = 0,
    RangeFft = 1,
    AzimuthFft = 2,
    Remap = 3,
    Write = 4,
};

struct StageTiming {
    StageKind stage = StageKind::Read;
    std::size_t calls = 0;
    double total_ms = 0.0;
};

class Profiler {
public:
    class ScopedStage {
    public:
        ScopedStage(Profiler& profiler, StageKind stage);
        ~ScopedStage();

        ScopedStage(const ScopedStage&) = delete;
        ScopedStage& operator=(const ScopedStage&) = delete;

    private:
        Profiler* profiler_;
        StageKind stage_;
        std::chrono::steady_clock::time_point start_;
    };

    ScopedStage scoped(StageKind stage);
    void add_sample(StageKind stage, double elapsed_ms);
    const std::vector<StageTiming>& timings() const;
    double total_ms() const;
    void reset();

private:
    std::vector<StageTiming> timings_;
};

const char* stage_kind_name(StageKind stage);
std::string format_profiler_report(const Profiler& profiler);

}  // namespace fft_imaging
