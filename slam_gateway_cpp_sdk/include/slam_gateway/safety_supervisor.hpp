#pragma once

#include "slam_gateway/models.hpp"

namespace slam_gateway {

class SafetySupervisor {
public:
    SafetyDecision evaluate(const SlamHealth& health,
                            const LocalizationState& localization,
                            const LocalObstacleSummary& obstacle) const;
};

}  // namespace slam_gateway
