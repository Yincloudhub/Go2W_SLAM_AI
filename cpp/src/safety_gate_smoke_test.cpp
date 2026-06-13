#include "go2w/safety_gate.hpp"

#include <iostream>
#include <stdexcept>

namespace {

void require(bool condition, const std::string& message)
{
    if (!condition) throw std::runtime_error(message);
}

nlohmann::json world(double front, double left, double right)
{
    return {
        {"world_state", {
            {"current_pose", {{"pose", {{"x", 0.0}, {"y", 0.0}, {"yaw", 0.0}}}, {"pose_age_ms", 10}}},
            {"localization", {{"status", "localized"}, {"pose_age_ms", 10}}},
            {"slam_health", {{"status", "ok"}, {"slam_alive", true}, {"localization_alive", true}}},
            {"safety", {{"allow_navigation", true}, {"reason", "ok"}}},
            {"local_obstacle", {
                {"source", "lidar_pointcloud"},
                {"stale", false},
                {"age_ms", 100},
                {"confidence", 0.8},
                {"front_confidence", 0.8},
                {"left_confidence", 0.8},
                {"right_confidence", 0.8},
                {"front_clearance_m", front},
                {"left_clearance_m", left},
                {"right_clearance_m", right},
                {"recommended_action", "normal"},
            }},
        }},
    };
}

}  // namespace

int main()
{
    go2w::SafetyGate gate;
    require(gate.evaluateWorldState(world(2.0, 2.0, 2.0)).allowed, "fresh XT16 geometry should pass");
    require(
        gate.evaluateWorldState(world(2.0, 2.0, 0.6)).allowed,
        "compatibility gate must not duplicate Gateway clearance thresholds");

    auto stale = world(2.0, 2.0, 2.0);
    stale["world_state"]["local_obstacle"]["stale"] = true;
    require(!gate.evaluateWorldState(stale).allowed, "stale trusted sensor summary should fail closed");

    auto adapter_fresh = world(2.0, 2.0, 0.6);
    adapter_fresh["world_state"]["local_obstacle"]["age_ms"] = 2500;
    require(
        gate.evaluateWorldState(adapter_fresh).allowed,
        "Gateway-owned freshness decision should remain authoritative");

    auto stub = world(6.0, 6.0, 6.0);
    stub["world_state"]["local_obstacle"]["source"] = "manual_stub";
    require(!gate.evaluateWorldState(stub).allowed, "manual clearance stub must fail closed");

    auto stereo_only = world(6.0, 6.0, 6.0);
    stereo_only["world_state"]["local_obstacle"]["source"] = "stereo_depth";
    require(!gate.evaluateWorldState(stereo_only).allowed, "forward stereo alone must not authorize navigation");

    auto missing_safety = world(2.0, 2.0, 2.0);
    missing_safety["world_state"].erase("safety");
    require(!gate.evaluateWorldState(missing_safety).allowed, "missing safety decision should block");

    auto stale_pose = world(2.0, 2.0, 2.0);
    stale_pose["world_state"]["localization"]["pose_age_ms"] = 2500;
    require(!gate.evaluateWorldState(stale_pose).allowed, "stale localization pose should block");

    auto degraded_localization = world(2.0, 2.0, 2.0);
    degraded_localization["world_state"]["localization"]["status"] = "degraded";
    degraded_localization["world_state"]["localization"]["pose_age_ms"] = 1500;
    require(!gate.evaluateWorldState(degraded_localization).allowed, "degraded localization must fail closed");

    std::cout << "go2w_safety_gate_smoke_test=passed\n";
    return 0;
}
