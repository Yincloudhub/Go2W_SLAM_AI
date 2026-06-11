#include "slam_gateway/safety_supervisor.hpp"

namespace slam_gateway {

SafetyDecision SafetySupervisor::evaluate(const SlamHealth& health,
                                           const LocalizationState& localization,
                                           const LocalObstacleSummary& obstacle) const
{
    constexpr double kMinimumObstacleConfidence = 0.15;

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

    const bool trusted_obstacle_source =
        obstacle.source == "lidar_pointcloud" ||
        obstacle.source == "lidar_pointcloud+stereo_depth";
    if (!trusted_obstacle_source) {
        d.allow_navigation = false;
        d.should_pause = true;
        d.recommended_mode = "hold";
        d.reason = "local_obstacle_source_not_trusted";
        return d;
    }
    if (trusted_obstacle_source && (obstacle.stale || obstacle.age_ms < 0)) {
        d.allow_navigation = false;
        d.should_pause = true;
        d.recommended_mode = "hold";
        d.reason = "local_obstacle_not_fresh";
        return d;
    }
    const bool fresh_obstacle =
        trusted_obstacle_source && !obstacle.stale && obstacle.age_ms >= 0;
    if (fresh_obstacle) {
        if (obstacle.recommended_action == "stop" || obstacle.recommended_action == "emergency_stop") {
            d.allow_navigation = false;
            d.should_pause = true;
            d.recommended_mode = "stop";
            d.reason = "local_obstacle_recommends_stop";
            return d;
        }
        if (obstacle.recommended_action == "pause") {
            d.allow_navigation = false;
            d.should_pause = true;
            d.recommended_mode = "pause";
            d.reason = "local_obstacle_recommends_pause";
            return d;
        }

        if (!(obstacle.front_confidence >= kMinimumObstacleConfidence)) {
            d.allow_navigation = false;
            d.should_pause = true;
            d.recommended_mode = "hold";
            d.reason = "front_obstacle_confidence_too_low";
            return d;
        }
        if (!(obstacle.left_confidence >= kMinimumObstacleConfidence)) {
            d.allow_navigation = false;
            d.should_pause = true;
            d.recommended_mode = "hold";
            d.reason = "left_obstacle_confidence_too_low";
            return d;
        }
        if (!(obstacle.right_confidence >= kMinimumObstacleConfidence)) {
            d.allow_navigation = false;
            d.should_pause = true;
            d.recommended_mode = "hold";
            d.reason = "right_obstacle_confidence_too_low";
            return d;
        }

        if (obstacle.front_clearance_m >= 0.0 && obstacle.front_clearance_m < 0.8) {
            d.allow_navigation = false;
            d.should_pause = true;
            d.recommended_mode = "pause";
            d.reason = "front_obstacle_too_close";
            return d;
        }

        if ((obstacle.left_clearance_m >= 0.0 && obstacle.left_clearance_m < 0.8) ||
            (obstacle.right_clearance_m >= 0.0 && obstacle.right_clearance_m < 0.8)) {
            d.allow_navigation = false;
            d.should_pause = true;
            d.recommended_mode = "pause";
            d.reason = "side_obstacle_too_close";
            return d;
        }
    }

    if (health.status == "degraded" || localization.status == "degraded" ||
        (fresh_obstacle && obstacle.recommended_action == "go_slow")) {
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
