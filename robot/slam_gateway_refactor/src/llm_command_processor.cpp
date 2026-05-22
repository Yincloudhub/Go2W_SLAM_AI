#include "slam_gateway/llm_command_processor.hpp"

#include <iostream>

namespace slam_gateway {

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
        const std::string slam_type = cmd.value("slam_type", "indoor");
        return ok(action, gateway_.startMapping(slam_type));
    }

    if (action == "end_mapping") {
        const std::string map_path = cmd.value("map_path", "/home/unitree/test.pcd");
        return ok(action, gateway_.endMapping(map_path));
    }

    if (action == "relocate") {
        const std::string map_path = cmd.value("map_path", "/home/unitree/test.pcd");
        PoseData init_pose;
        if (cmd.contains("initial_pose") && cmd["initial_pose"].is_object()) {
            init_pose = parsePose(cmd["initial_pose"]);
        }
        return ok(action, gateway_.startRelocation(map_path, init_pose));
    }

    if (action == "navigate_to_pose") {
        if (!cmd.contains("target_pose") || !cmd["target_pose"].is_object()) {
            return reject("missing_target_pose");
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
        return ok(action, gateway_.resumeNavigation());
    }

    if (action == "stop_slam") {
        gateway_.taskThreadStop();
        return ok(action, gateway_.stopNode());
    }

    return reject("unknown_action:" + action);
}

}  // namespace slam_gateway
