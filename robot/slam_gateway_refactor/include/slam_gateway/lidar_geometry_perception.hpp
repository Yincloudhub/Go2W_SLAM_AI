#pragma once

#include <cstdint>
#include <mutex>
#include <string>

#include "slam_gateway/models.hpp"

namespace slam_gateway {

class LidarGeometryPerception {
public:
    LidarGeometryPerception() = default;

    void setManualClearance(double front, double left, double right, double rear);
    LocalObstacleSummary getLocalObstacleSummary() const;
    LocalObstacleSummary getExternalSummaryOrFallback(const std::string& path, int64_t max_age_ms) const;
    LocalObstacleSummary getFusedSummaryOrFallback(
        const std::string& lidar_path,
        int64_t lidar_max_age_ms,
        const std::string& stereo_path,
        int64_t stereo_max_age_ms) const;

private:
    LocalObstacleSummary summary_;
    mutable std::mutex fusion_state_mutex_;
    mutable int64_t last_stereo_block_timestamp_ms_{0};
    mutable int secondary_front_block_confirmation_count_{0};
};

}  // namespace slam_gateway
