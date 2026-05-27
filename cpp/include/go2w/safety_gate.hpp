#pragma once

#include <string>

#include <nlohmann/json.hpp>

namespace go2w {

struct SafetyDecision {
    bool allowed = false;
    std::string reason;
    std::string recommended_mode = "stop";
};

struct SafetyLimits {
    double emergency_clearance_m = 0.8;
    double pause_clearance_m = 1.5;
    double low_battery_percent = 20.0;
    double weak_bandwidth_kbps = 200.0;
    double weak_latency_ms = 500.0;
    double weak_packet_loss_ratio = 0.2;
    double arrival_distance_m = 0.7;
};

class SafetyGate {
public:
    explicit SafetyGate(SafetyLimits limits = {});

    SafetyDecision evaluateWorldState(const nlohmann::json& world_state_result) const;
    SafetyDecision evaluateBeforeNavigation(const nlohmann::json& world_state_result, const std::string& target_node) const;
    SafetyDecision evaluateBeforeNavigation(
        const nlohmann::json& world_state_result,
        const std::string& target_node,
        const nlohmann::json& target_pose) const;

private:
    SafetyLimits limits_;
};

bool worldAllowsNavigation(const nlohmann::json& world_state_result, std::string* reason);

}  // namespace go2w
