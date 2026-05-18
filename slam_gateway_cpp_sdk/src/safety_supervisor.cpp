#include "slam_gateway/safety_supervisor.hpp"

namespace slam_gateway {

SafetyDecision SafetySupervisor::evaluate(const SlamHealth& health,
                                           const LocalizationState& localization,
                                           const LocalObstacleSummary& obstacle) const
{
    SafetyDecision d;

    if (health.status == "failed" || !health.slam_alive) {
        d.allow_navigation = false;
        d.should_pause = true;
        d.recommended_mode = "stop";
        d.reason = "slam_health_failed";
        return d;
    }

    if (localization.status == "lost" || localization.status == "not_started") {
        d.allow_navigation = false;
        d.should_pause = true;
        d.recommended_mode = "stop";
        d.reason = "localization_not_valid";
        return d;
    }

    if (obstacle.front_clearance_m >= 0.0 && obstacle.front_clearance_m < 0.8) {
        d.allow_navigation = false;
        d.should_pause = true;
        d.recommended_mode = "pause";
        d.reason = "front_obstacle_too_close";
        return d;
    }

    if (health.status == "degraded" || localization.status == "degraded" || obstacle.recommended_action == "go_slow") {
        d.allow_navigation = true;
        d.should_pause = false;
        d.recommended_mode = "conservative";
        d.reason = "degraded_or_near_obstacle";
        return d;
    }

    d.allow_navigation = true;
    d.should_pause = false;
    d.recommended_mode = "normal";
    d.reason = "ok";
    return d;
}

}  // namespace slam_gateway
