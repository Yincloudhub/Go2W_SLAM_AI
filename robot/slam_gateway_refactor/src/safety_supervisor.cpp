#include "slam_gateway/safety_supervisor.hpp"
#include "slam_gateway/obstacle_policy.hpp"

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

    if (localization.status != "localized") {
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
    const bool supervised_stale_grace =
        obstacle.supervised_release_active &&
        obstacle.stale &&
        obstacle.age_ms >= 0 &&
        obstacle.age_ms <= obstacle_policy::kSupervisedObstacleMaxAgeMs;
    if (trusted_obstacle_source &&
        (obstacle.age_ms < 0 || (obstacle.stale && !supervised_stale_grace))) {
        d.allow_navigation = false;
        d.should_pause = true;
        d.recommended_mode = "hold";
        d.reason = "local_obstacle_not_fresh";
        return d;
    }
    if (obstacle.supervised_release_active &&
        (obstacle.supervised_release_id.empty() ||
         obstacle.supervised_max_speed_mps <= 0.0 ||
         obstacle.supervised_max_speed_mps > 0.1)) {
        d.allow_navigation = false;
        d.should_pause = true;
        d.recommended_mode = "hold";
        d.reason = "supervised_release_invalid";
        return d;
    }
    const bool usable_obstacle =
        trusted_obstacle_source &&
        obstacle.age_ms >= 0 &&
        (!obstacle.stale || supervised_stale_grace);
    if (usable_obstacle) {
        if (obstacle.recommended_action == "stop" || obstacle.recommended_action == "emergency_stop") {
            d.allow_navigation = false;
            d.should_pause = true;
            d.recommended_mode = "stop";
            d.reason = "local_obstacle_recommends_stop";
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

        if (obstacle.supervised_release_active) {
            if (health.status == "degraded") {
                d.allow_navigation = false;
                d.should_pause = true;
                d.recommended_mode = "stop";
                d.reason = "slam_health_degraded";
                return d;
            }
            const bool local_obstacle_advisory =
                obstacle.recommended_action == "pause" ||
                (obstacle.front_clearance_m >= 0.0 &&
                 obstacle.front_clearance_m < obstacle_policy::kFrontPauseM) ||
                (obstacle.left_clearance_m >= 0.0 &&
                 obstacle.left_clearance_m < obstacle_policy::kSideSlowM) ||
                (obstacle.right_clearance_m >= 0.0 &&
                 obstacle.right_clearance_m < obstacle_policy::kSideSlowM) ||
                (obstacle.rear_clearance_m >= 0.0 &&
                 obstacle.rear_clearance_m < obstacle_policy::kRearSlowM);
            d.allow_navigation = true;
            d.should_pause = false;
            d.recommended_mode = "conservative";
            d.motion_direction = "unitree_pose_navigation_mode_0";
            d.reason = supervised_stale_grace
                ? "supervised_unitree_avoidance_available_with_sensor_delay_advisory"
                : (local_obstacle_advisory
                    ? "supervised_unitree_avoidance_available_with_local_obstacle_advisory"
                    : "supervised_unitree_avoidance_available");
            d.speed_limit_mps = obstacle.supervised_max_speed_mps;
            return d;
        }

        if (obstacle.front_clearance_m >= 0.0 &&
            obstacle.front_clearance_m < obstacle_policy::kFrontPauseM) {
            d.allow_navigation = false;
            d.should_pause = true;
            d.recommended_mode = "pause";
            d.reason = "front_obstacle_too_close";
            return d;
        }

        if (obstacle.recommended_action == "pause") {
            d.allow_navigation = false;
            d.should_pause = true;
            d.recommended_mode = "pause";
            d.reason = "local_obstacle_recommends_pause";
            return d;
        }

        if ((obstacle.left_clearance_m >= 0.0 &&
             obstacle.left_clearance_m < obstacle_policy::kSidePauseM) ||
            (obstacle.right_clearance_m >= 0.0 &&
             obstacle.right_clearance_m < obstacle_policy::kSidePauseM)) {
            d.allow_navigation = false;
            d.should_pause = true;
            d.recommended_mode = "pause";
            d.reason = "side_obstacle_too_close";
            return d;
        }
        if (obstacle.rear_clearance_m >= 0.0 &&
            obstacle.rear_clearance_m < obstacle_policy::kRearPauseM) {
            d.allow_navigation = false;
            d.should_pause = true;
            d.recommended_mode = "pause";
            d.reason = "rear_obstacle_too_close";
            return d;
        }
    }

    if (health.status == "degraded") {
        d.allow_navigation = false;
        d.should_pause = true;
        d.recommended_mode = "stop";
        d.reason = "slam_health_degraded";
        return d;
    }

    const bool corridor_conservative =
        usable_obstacle &&
        ((obstacle.front_clearance_m >= 0.0 &&
          obstacle.front_clearance_m < obstacle_policy::kFrontSlowM) ||
         (obstacle.left_clearance_m >= 0.0 &&
          obstacle.left_clearance_m < obstacle_policy::kSideSlowM) ||
         (obstacle.right_clearance_m >= 0.0 &&
          obstacle.right_clearance_m < obstacle_policy::kSideSlowM) ||
         (obstacle.rear_clearance_m >= 0.0 &&
          obstacle.rear_clearance_m < obstacle_policy::kRearSlowM));
    if (usable_obstacle &&
        (obstacle.recommended_action == "go_slow" || corridor_conservative)) {
        d.allow_navigation = true;
        d.should_pause = false;
        d.recommended_mode = "conservative";
        d.reason = "corridor_clearance_requires_low_speed";
        d.speed_limit_mps =
            obstacle.supervised_release_active
                ? obstacle.supervised_max_speed_mps
                : obstacle_policy::kConservativeSpeedMps;
        return d;
    }
    d.allow_navigation = true;
    d.should_pause = false;
    d.recommended_mode = "normal";
    d.reason = "ok";
    return d;
}

}  // namespace slam_gateway
