#include "slam_gateway/llm_command_processor.hpp"

#include <cmath>
#include <iostream>
#include <string>

namespace slam_gateway {
namespace {

bool hasFiniteNumber(const nlohmann::json& j, const char* key)
{
    return j.contains(key) && j.at(key).is_number() && std::isfinite(j.at(key).get<double>());
}

bool hasOptionalFiniteNumber(const nlohmann::json& j, const char* key)
{
    return !j.contains(key) || (j.at(key).is_number() && std::isfinite(j.at(key).get<double>()));
}

std::string validatePoseJson(const nlohmann::json& j, bool require_positive_speed)
{
    if (!hasFiniteNumber(j, "x")) return "target_pose.x must be a finite number";
    if (!hasFiniteNumber(j, "y")) return "target_pose.y must be a finite number";
    if (!hasOptionalFiniteNumber(j, "z")) return "target_pose.z must be a finite number";
    if (!hasOptionalFiniteNumber(j, "q_x")) return "target_pose.q_x must be a finite number";
    if (!hasOptionalFiniteNumber(j, "q_y")) return "target_pose.q_y must be a finite number";
    if (!hasOptionalFiniteNumber(j, "q_z")) return "target_pose.q_z must be a finite number";
    if (!hasOptionalFiniteNumber(j, "q_w")) return "target_pose.q_w must be a finite number";
    if (!hasOptionalFiniteNumber(j, "speed")) return "target_pose.speed must be a finite number";
    if (j.contains("speed")) {
        const double speed = j.at("speed").get<double>();
        if ((require_positive_speed && speed <= 0.0) || speed < 0.0 || speed > 0.8) {
            return "target_pose.speed out of safe range";
        }
    }
    if (j.contains("mode")) {
        if (!j.at("mode").is_number_integer()) return "target_pose.mode must be an integer";
        const int mode = j.at("mode").get<int>();
        if (mode != 0 && mode != 1) return "target_pose.mode must be 0 or 1";
    }
    return "";
}

bool hasOperatorAck(const nlohmann::json& cmd)
{
    const auto ack = cmd.find("operator_ack");
    if (ack != cmd.end() && ack->is_boolean() && ack->get<bool>()) return true;
    const auto confirm = cmd.find("confirm");
    return confirm != cmd.end() && confirm->is_boolean() && confirm->get<bool>();
}

bool hasRuntimeWatchdog(const nlohmann::json& cmd)
{
    if (cmd.value("runtime_watchdog", false) != true) return false;
    const std::string context = cmd.value("execution_context", "");
    return context == "queue_executor_v1" || context == "python_closed_loop_v1";
}

bool isFreshEnoughForWaypoint(const LocalizationState& loc)
{
    const bool status_ok =
        loc.status == "localized" || loc.status == "degraded" || loc.status == "localized_or_tracking" || loc.status == "tracking";
    return status_ok && loc.pose_age_ms >= 0 && loc.pose_age_ms <= 2000;
}

}  // namespace

LlmCommandProcessor::LlmCommandProcessor(SlamGateway& gateway)
    : gateway_(gateway)
{
}

nlohmann::json LlmCommandProcessor::reject(const std::string& reason) const
{
    return {
        {"accepted", false},
        {"reason", reason}
    };
}

nlohmann::json LlmCommandProcessor::ok(const std::string& action, const ServiceResult& result) const
{
    return {
        {"accepted", result.ok},
        {"action", action},
        {"status_code", result.status_code},
        {"data", result.data},
        {"world_state", gateway_.buildWorldStateJson()}
    };
}

PoseData LlmCommandProcessor::parsePose(const nlohmann::json& j)
{
    PoseData p;
    p.name = j.value("name", "llm_goal");
    p.x = j.value("x", 0.0f);
    p.y = j.value("y", 0.0f);
    p.z = j.value("z", 0.0f);
    p.q_x = j.value("q_x", 0.0f);
    p.q_y = j.value("q_y", 0.0f);
    p.q_z = j.value("q_z", 0.0f);
    p.q_w = j.value("q_w", 1.0f);
    p.mode = j.value("mode", 0);   // Obstacle mode: 0=avoid, 1=stop.
    p.speed = j.value("speed", 0.5f); // LLM path defaults to conservative speed.
    return p;
}

nlohmann::json LlmCommandProcessor::process(const nlohmann::json& cmd)
{
    // Security boundary: LLM must not call raw API IDs directly.
    if (cmd.contains("api_id") || cmd.contains("ROBOT_API_ID") || cmd.contains("raw_api")) {
        return reject("raw_api_id_is_forbidden; use validated action names only");
    }

    const std::string action = cmd.value("action", "");
    if (action.empty()) return reject("missing_action");

    if (action == "get_world_state") {
        return {{"accepted", true}, {"action", action}, {"world_state", gateway_.buildWorldStateJson()}};
    }

    if (action == "start_mapping") {
        if (!hasOperatorAck(cmd)) return reject("operator_ack_required_for_start_mapping");
        const std::string slam_type = cmd.value("slam_type", "indoor");
        return ok(action, gateway_.startMapping(slam_type));
    }

    if (action == "end_mapping") {
        if (!hasOperatorAck(cmd)) return reject("operator_ack_required_for_end_mapping");
        const std::string map_path = cmd.value("map_path", "/home/unitree/test.pcd");
        return ok(action, gateway_.endMapping(map_path));
    }

    if (action == "add_current_pose_waypoint") {
        if (!hasOperatorAck(cmd)) return reject("operator_ack_required_for_add_current_pose_waypoint");
        const auto loc = gateway_.getLocalizationState();
        if (!isFreshEnoughForWaypoint(loc)) {
            return {
                {"accepted", false},
                {"reason", "localization_not_fresh_enough_for_waypoint"},
                {"required_pose_age_ms_lte", 2000},
                {"localization", loc.toJson()},
                {"world_state", gateway_.buildWorldStateJson()}
            };
        }
        const std::string name = cmd.value("name", "");
        gateway_.addCurrentPoseAsWaypoint(name);
        return {
            {"accepted", true},
            {"action", action},
            {"waypoint_count", gateway_.waypointCount()},
            {"world_state", gateway_.buildWorldStateJson()}
        };
    }

    if (action == "relocate") {
        const std::string map_path = cmd.value("map_path", "/home/unitree/test.pcd");
        if (!cmd.contains("initial_pose") || !cmd["initial_pose"].is_object()) {
            return reject("missing_initial_pose");
        }
        const std::string pose_error = validatePoseJson(cmd["initial_pose"], false);
        if (!pose_error.empty()) {
            return reject(pose_error);
        }
        PoseData init_pose = parsePose(cmd["initial_pose"]);
        return ok(action, gateway_.startRelocation(map_path, init_pose));
    }

    if (action == "navigate_to_pose") {
        if (!hasRuntimeWatchdog(cmd)) {
            return reject("runtime_watchdog_required_for_navigation");
        }
        if (!cmd.contains("target_pose") || !cmd["target_pose"].is_object()) {
            return reject("missing_target_pose");
        }
        const std::string pose_error = validatePoseJson(cmd["target_pose"], true);
        if (!pose_error.empty()) {
            return reject(pose_error);
        }
        auto safety = gateway_.getSafetyDecision();
        if (!safety.allow_navigation) {
            return {{"accepted", false}, {"reason", "safety_blocked"}, {"safety", safety.toJson()}, {"world_state", gateway_.buildWorldStateJson()}};
        }
        PoseData goal = parsePose(cmd["target_pose"]);
        return ok(action, gateway_.submitNavigationGoal(goal));
    }

    if (action == "pause_navigation") {
        return ok(action, gateway_.pauseNavigation());
    }

    if (action == "resume_navigation") {
        if (!hasRuntimeWatchdog(cmd)) {
            return reject("runtime_watchdog_required_for_resume");
        }
        auto safety = gateway_.getSafetyDecision();
        if (!safety.allow_navigation) {
            return {{"accepted", false}, {"reason", "safety_blocked"}, {"safety", safety.toJson()}, {"world_state", gateway_.buildWorldStateJson()}};
        }
        return ok(action, gateway_.resumeNavigation());
    }

    if (action == "stop_slam") {
        if (!hasOperatorAck(cmd)) return reject("operator_ack_required_for_stop_slam");
        gateway_.taskThreadStop();
        return ok(action, gateway_.stopNode());
    }

    return reject("unknown_action:" + action);
}

}  // namespace slam_gateway
