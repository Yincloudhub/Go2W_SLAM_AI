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

    auto stale = clear;
    stale.stale = true;
    const auto stale_decision = supervisor.evaluate(health, localization, stale);
    require(!stale_decision.allow_navigation, "stale trusted lidar should fail closed");
    require(stale_decision.reason == "local_obstacle_not_fresh", "stale lidar reason mismatch");

    auto side = clear;
    side.right_clearance_m = 0.5;
    side.recommended_action = "pause";
    require(!supervisor.evaluate(health, localization, side).allow_navigation, "close side obstacle should block");

    auto stub = clear;
    stub.source = "manual_stub";
    stub.stale = true;
    require(supervisor.evaluate(health, localization, stub).allow_navigation, "manual stub should remain advisory");

    std::cout << "slam_gateway_safety_supervisor_smoke_test=passed\n";
    return 0;
}
