#pragma once

#include "slam_gateway/models.hpp"

namespace slam_gateway {

class LidarGeometryPerception {
public:
    LidarGeometryPerception() = default;

    void setManualClearance(double front, double left, double right, double rear);
    LocalObstacleSummary getLocalObstacleSummary() const;

private:
    LocalObstacleSummary summary_;
};

}  // namespace slam_gateway
