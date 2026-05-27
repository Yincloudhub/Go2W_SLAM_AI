#include "go2w/safety_gate.hpp"

#include <cmath>
#include <utility>

namespace go2w {
namespace {

const nlohmann::json* objectAt(const nlohmann::json& root, const std::initializer_list<const char*> keys)
{
    const nlohmann::json* current = &root;
    for (const char* key : keys) {
        if (!current->is_object() || !current->contains(key)) return nullptr;
        current = &current->at(key);
    }
    return current;
}

double numberAt(const nlohmann::json& root, const std::initializer_list<const char*> keys, double fallback)
{
    const auto* value = objectAt(root, keys);
    if (!value || !value->is_number()) return fallback;
    return value->get<double>();
}

bool boolAt(const nlohmann::json& root, const std::initializer_list<const char*> keys, bool fallback)
{
    const auto* value = objectAt(root, keys);
    if (!value || !value->is_boolean()) return fallback;
    return value->get<bool>();
}

std::string stringAt(const nlohmann::json& root, const std::initializer_list<const char*> keys, const std::string& fallback = "")
{
    const auto* value = objectAt(root, keys);
    if (!value || !value->is_string()) return fallback;
    return value->get<std::string>();
}

double poseDistance(const nlohmann::json& world, const nlohmann::json& target_pose)
{
    const auto* pose = objectAt(world, {"current_pose", "pose"});
    if (!pose || !pose->is_object() || !target_pose.is_object()) return -1.0;
    const double x = numberAt(*pose, {"x"}, NAN);
    const double y = numberAt(*pose, {"y"}, NAN);
    const double tx = numberAt(target_pose, {"x"}, NAN);
    const double ty = numberAt(target_pose, {"y"}, NAN);
    if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(tx) || !std::isfinite(ty)) return -1.0;
    return std::hypot(x - tx, y - ty);
}

SafetyDecision blocked(std::string reason, std::string mode = "stop")
{
    SafetyDecision decision;
    decision.allowed = false;
    decision.reason = std::move(reason);
    decision.recommended_mode = std::move(mode);
    return decision;
}

bool hasBlockingRisk(const nlohmann::json& risk)
{
    const std::string severity = risk.value("severity", "");
    return severity == "critical" || severity == "high";
}

}  // namespace

SafetyGate::SafetyGate(SafetyLimits limits)
    : limits_(limits)
{
}

SafetyDecision SafetyGate::evaluateWorldState(const nlohmann::json& world_state_result) const
{
    const auto* world = objectAt(world_state_result, {"world_state"});
    if (!world || !world->is_object()) return blocked("missing world_state");

    std::string conservative_reason;
    std::string conservative_mode = "slow";

    const auto* safety = objectAt(*world, {"safety"});
    if (safety && safety->is_object() && safety->value("allow_navigation", false) != true) {
        return blocked(
            "safety disallows navigation: " + safety->value("reason", std::string("unknown")),
            safety->value("recommended_mode", std::string("stop")));
    }

    const auto* health = objectAt(*world, {"slam_health"});
    if (health && health->is_object()) {
        const std::string status = health->value("status", "");
        if (!status.empty() && status != "ok") {
            if (status == "degraded") {
                conservative_reason = "slam health degraded; conservative navigation only";
                conservative_mode = "slow";
            } else {
                return blocked("slam health is " + status);
            }
        }
        if (health->contains("slam_alive") && !health->value("slam_alive", false)) return blocked("slam is not alive");
        if (health->contains("localization_alive") && !health->value("localization_alive", false)) return blocked("localization is not alive");
    }

    const auto* loc = objectAt(*world, {"localization"});
    if (loc && loc->is_object()) {
        const std::string status = loc->value("status", "");
        if (!status.empty() && status != "localized_or_tracking" && status != "tracking" && status != "localized") {
            if (status == "degraded") {
                if (conservative_reason.empty()) conservative_reason = "localization degraded; conservative navigation only";
                conservative_mode = "slow";
            } else {
                return blocked("localization is " + status);
            }
        }
        const double pose_age = loc->value("pose_age_ms", 0.0);
        if (pose_age < 0.0) return blocked("localization pose is not valid");
    }

    const double current_pose_age = numberAt(*world, {"current_pose", "pose_age_ms"}, 0.0);
    if (current_pose_age < 0.0) return blocked("current pose is not valid");

    const auto* navigation = objectAt(*world, {"navigation"});
    if (navigation && navigation->is_object()) {
        const std::string nav_state = navigation->value("state", "");
        if (nav_state == "failed" || nav_state == "blocked") return blocked("navigation backend is " + nav_state);
    }

    const double battery = numberAt(*world, {"robot", "battery_percent"}, -1.0);
    if (battery >= 0.0 && battery < limits_.low_battery_percent) {
        return blocked("battery below low_battery_percent", "confirm_or_charge");
    }

    const double front_clearance = numberAt(*world, {"local_obstacle", "front_clearance_m"}, -1.0);
    const std::string obstacle_action = stringAt(*world, {"local_obstacle", "recommended_action"});
    if (front_clearance >= 0.0 && front_clearance < limits_.emergency_clearance_m) {
        return blocked("front obstacle inside emergency distance", "emergency_stop");
    }
    if ((front_clearance >= 0.0 && front_clearance < limits_.pause_clearance_m) || obstacle_action == "pause") {
        return blocked("front obstacle inside pause distance", "pause");
    }

    const auto* risk_events = objectAt(*world, {"risk_events"});
    if (risk_events && risk_events->is_array()) {
        for (const auto& risk : *risk_events) {
            if (!risk.is_object() || !hasBlockingRisk(risk)) continue;
            const double distance = risk.value("distance_m", -1.0);
            if (distance < 0.0 || distance <= limits_.pause_clearance_m) {
                return blocked("nearby high risk: " + risk.value("description", risk.value("event_type", std::string("unknown"))), "hold");
            }
        }
    }

    const auto* objects = objectAt(*world, {"objects"});
    if (objects && objects->is_array()) {
        for (const auto& object : *objects) {
            if (!object.is_object()) continue;
            if (boolAt(object, {"traversable"}, true)) continue;
            const double distance = object.value("distance_m", -1.0);
            if (distance >= 0.0 && distance <= limits_.pause_clearance_m) {
                return blocked("path blocked by " + object.value("category", std::string("object")), "replan");
            }
        }
    }

    SafetyDecision decision;
    decision.allowed = true;
    const double bandwidth = numberAt(*world, {"link_quality", "bandwidth_kbps"}, -1.0);
    const double latency = numberAt(*world, {"link_quality", "latency_ms"}, -1.0);
    const double packet_loss = numberAt(*world, {"link_quality", "packet_loss_ratio"}, 0.0);
    if ((bandwidth >= 0.0 && bandwidth < limits_.weak_bandwidth_kbps) ||
        (latency >= 0.0 && latency > limits_.weak_latency_ms) ||
        packet_loss >= limits_.weak_packet_loss_ratio) {
        decision.reason = conservative_reason.empty()
            ? "weak network detected; semantic-only feedback recommended"
            : conservative_reason + "; weak network detected; semantic-only feedback recommended";
        decision.recommended_mode = "semantic_only";
        return decision;
    }

    decision.reason = conservative_reason.empty() ? "world state is safe enough for navigation" : conservative_reason;
    decision.recommended_mode = conservative_reason.empty() ? "normal" : conservative_mode;
    return decision;
}

SafetyDecision SafetyGate::evaluateBeforeNavigation(const nlohmann::json& world_state_result, const std::string& target_node) const
{
    SafetyDecision decision = evaluateWorldState(world_state_result);
    if (!decision.allowed) return decision;
    if (target_node.empty()) {
        decision.allowed = false;
        decision.reason = "missing target node";
        decision.recommended_mode = "hold";
    }
    return decision;
}

SafetyDecision SafetyGate::evaluateBeforeNavigation(
    const nlohmann::json& world_state_result,
    const std::string& target_node,
    const nlohmann::json& target_pose) const
{
    SafetyDecision decision = evaluateBeforeNavigation(world_state_result, target_node);
    if (!decision.allowed) return decision;

    const auto* world = objectAt(world_state_result, {"world_state"});
    if (!world || !world->is_object()) return decision;
    const double distance = poseDistance(*world, target_pose);
    if (distance >= 0.0 && distance <= limits_.arrival_distance_m) {
        decision.allowed = false;
        decision.reason = "already within arrival_distance_m of target";
        decision.recommended_mode = "hold";
    }
    return decision;
}

bool worldAllowsNavigation(const nlohmann::json& world_state_result, std::string* reason)
{
    const SafetyDecision decision = SafetyGate().evaluateWorldState(world_state_result);
    if (reason) *reason = decision.reason;
    return decision.allowed;
}

}  // namespace go2w
