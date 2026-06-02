#pragma once

#include <cstdint>
#include <string>

#include <nlohmann/json.hpp>

namespace go2w {

// Loads one bounded latest-value summary produced by an optional external edge
// node. A summary is diagnostic/semantic by default. SafetyGate integration is
// a separate explicit step after calibration and field validation.
nlohmann::json loadEdgePerceptionSummary(
    const std::string& path,
    std::int64_t max_age_ms = 3000,
    std::int64_t current_time_ms = 0);

}  // namespace go2w
