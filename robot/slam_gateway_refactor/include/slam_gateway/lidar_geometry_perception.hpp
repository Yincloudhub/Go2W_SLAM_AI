#pragma once

#include <cstdint>
#include <string>

#include "slam_gateway/models.hpp"

namespace slam_gateway {

class LidarGeometryPerception {
public:
    LidarGeometryPerception() = default;

    void setManualClearance(double front, double left, double right, double rear);
    LocalObstacleSummary getLocalObstacleSummary() const;
    LocalObstacleSummary getExternalSummaryOrFallback(const std::string& path, int64_t max_age_ms) const;

private:
    LocalObstacleSummary summary_;
};

}  // namespace slam_gateway
