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
            {"current_pose", {{"pose_age_ms", 10}}},
            {"localization", {{"status", "localized"}, {"pose_age_ms", 10}}},
            {"slam_health", {{"status", "ok"}, {"slam_alive", true}, {"localization_alive", true}}},
            {"safety", {{"allow_navigation", true}, {"reason", "ok"}}},
            {"local_obstacle", {
                {"source", "stereo_depth"},
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
    require(gate.evaluateWorldState(world(2.0, 2.0, 2.0)).allowed, "fresh stereo depth should pass");
    require(!gate.evaluateWorldState(world(2.0, 2.0, 0.6)).allowed, "right-side obstacle should block");

    auto stale = world(2.0, 2.0, 2.0);
    stale["world_state"]["local_obstacle"]["stale"] = true;
    require(!gate.evaluateWorldState(stale).allowed, "stale sensor summary should block");

    auto stub = world(6.0, 6.0, 6.0);
    stub["world_state"]["local_obstacle"]["source"] = "manual_stub";
    require(!gate.evaluateWorldState(stub).allowed, "manual clearance stub should block");

    std::cout << "go2w_safety_gate_smoke_test=passed\n";
    return 0;
}
