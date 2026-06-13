#include "slam_gateway/safety_supervisor.hpp"

#include <iostream>
#include <stdexcept>

namespace {

void require(bool condition, const char* message)
{
    if (!condition) throw std::runtime_error(message);
}

slam_gateway::SlamHealth healthySlam()
{
    slam_gateway::SlamHealth health;
    health.status = "ok";
    health.slam_alive = true;
    health.localization_alive = true;
    return health;
}

slam_gateway::LocalizationState localized()
{
    slam_gateway::LocalizationState state;
    state.status = "localized";
    state.pose_age_ms = 100;
    state.confidence = 0.9;
    return state;
}

slam_gateway::LocalObstacleSummary lidarObstacle()
{
    slam_gateway::LocalObstacleSummary obstacle;
    obstacle.source = "lidar_pointcloud";
    obstacle.stale = false;
    obstacle.age_ms = 100;
    obstacle.front_clearance_m = 2.0;
    obstacle.left_clearance_m = 2.0;
    obstacle.right_clearance_m = 2.0;
    obstacle.front_confidence = 1.0;
    obstacle.left_confidence = 1.0;
    obstacle.right_confidence = 1.0;
    obstacle.recommended_action = "normal";
    return obstacle;
}

}  // namespace

int main()
{
    slam_gateway::SafetySupervisor supervisor;
    const auto health = healthySlam();
    const auto localization = localized();

    auto clear = lidarObstacle();
    require(supervisor.evaluate(health, localization, clear).allow_navigation, "fresh clear lidar should allow");

    auto missing_front_confidence = clear;
    missing_front_confidence.front_confidence = 0.0;
    const auto missing_front_confidence_decision =
        supervisor.evaluate(health, localization, missing_front_confidence);
    require(!missing_front_confidence_decision.allow_navigation,
            "missing front confidence must block despite clear clearance");
    require(missing_front_confidence_decision.reason == "front_obstacle_confidence_too_low",
            "missing front confidence reason mismatch");

    auto missing_left_confidence = clear;
    missing_left_confidence.left_confidence = 0.0;
    const auto missing_left_confidence_decision =
        supervisor.evaluate(health, localization, missing_left_confidence);
    require(!missing_left_confidence_decision.allow_navigation,
            "missing left confidence must block despite clear clearance");
    require(missing_left_confidence_decision.reason == "left_obstacle_confidence_too_low",
            "missing left confidence reason mismatch");

    auto missing_right_confidence = clear;
    missing_right_confidence.right_confidence = 0.0;
    const auto missing_right_confidence_decision =
        supervisor.evaluate(health, localization, missing_right_confidence);
    require(!missing_right_confidence_decision.allow_navigation,
            "missing right confidence must block despite clear clearance");
    require(missing_right_confidence_decision.reason == "right_obstacle_confidence_too_low",
            "missing right confidence reason mismatch");

    auto stale = clear;
    stale.stale = true;
    const auto stale_decision = supervisor.evaluate(health, localization, stale);
    require(!stale_decision.allow_navigation, "stale trusted lidar should fail closed");
    require(stale_decision.reason == "local_obstacle_not_fresh", "stale lidar reason mismatch");

    auto corridor_side = clear;
    corridor_side.right_clearance_m = 0.5;
    corridor_side.recommended_action = "go_slow";
    const auto corridor_decision = supervisor.evaluate(health, localization, corridor_side);
    require(corridor_decision.allow_navigation, "corridor side clearance should remain navigable");
    require(corridor_decision.recommended_mode == "conservative",
            "corridor side clearance should force conservative mode");
    require(corridor_decision.speed_limit_mps == 0.2,
            "conservative corridor mode must expose a real speed limit");
    const auto corridor_json = corridor_decision.toJson();
    require(corridor_json.value("policy_version", std::string{}) == "corridor_clearance_v1",
            "safety JSON must identify the active clearance policy");
    require(corridor_json.value("motion_direction", std::string{}) == "planner_controlled",
            "safety JSON must keep direction under planner control");
    require(corridor_json.value("speed_limit_mps", -1.0) == 0.2,
            "safety JSON must expose the conservative speed limit");

    auto extreme_side = clear;
    extreme_side.right_clearance_m = 0.1;
    extreme_side.recommended_action = "pause";
    require(!supervisor.evaluate(health, localization, extreme_side).allow_navigation,
            "extremely close side obstacle should block");

    auto stub = clear;
    stub.source = "manual_stub";
    stub.stale = true;
    const auto stub_decision = supervisor.evaluate(health, localization, stub);
    require(!stub_decision.allow_navigation, "manual stub must not authorize real navigation");
    require(stub_decision.reason == "local_obstacle_source_not_trusted", "manual stub reason mismatch");

    auto stereo_only = clear;
    stereo_only.source = "stereo_depth";
    const auto stereo_decision = supervisor.evaluate(health, localization, stereo_only);
    require(!stereo_decision.allow_navigation, "forward stereo alone must not authorize side-safe navigation");
    require(stereo_decision.reason == "local_obstacle_source_not_trusted", "stereo-only reason mismatch");

    auto degraded_localization = localization;
    degraded_localization.status = "degraded";
    degraded_localization.pose_age_ms = 700;
    const auto degraded_localization_decision =
        supervisor.evaluate(health, degraded_localization, clear);
    require(!degraded_localization_decision.allow_navigation,
            "degraded localization must fail closed");
    require(degraded_localization_decision.should_pause,
            "degraded localization must request pause");

    auto degraded_health = health;
    degraded_health.status = "degraded";
    const auto degraded_health_decision =
        supervisor.evaluate(degraded_health, localization, clear);
    require(!degraded_health_decision.allow_navigation,
            "degraded SLAM health must fail closed");
    require(degraded_health_decision.reason == "slam_health_degraded",
            "degraded SLAM health reason mismatch");

    std::cout << "slam_gateway_safety_supervisor_smoke_test=passed\n";
    return 0;
}
