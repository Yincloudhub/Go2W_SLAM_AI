#include "go2w/plan_executor.hpp"

#include <cmath>
#include <set>
#include <stdexcept>

namespace go2w {
namespace {

const std::set<std::string> kModes = {
    "mapped_navigation",
    "mapless_scout",
    "safe_hold",
    "human_confirm",
};

const std::set<std::string> kTools = {
    "set_communication_policy",
    "create_navigation_subgoal",
    "wait_until",
    "capture_keyframe",
    "relative_motion_preview",
    "start_mapless_scout",
    "request_human_confirm",
    "hold_position",
};

const std::set<std::string> kCommunicationModes = {
    "normal",
    "semantic_only",
    "keyframe_low_rate",
    "hold_remote",
};

double jsonDouble(const nlohmann::json& object, const std::string& key, double fallback)
{
    if (!object.contains(key) || object.at(key).is_null()) return fallback;
    return object.at(key).get<double>();
}

int jsonInt(const nlohmann::json& object, const std::string& key, int fallback)
{
    if (!object.contains(key) || object.at(key).is_null()) return fallback;
    return object.at(key).get<int>();
}

double yawFromQuaternion(double qx, double qy, double qz, double qw)
{
    const double siny_cosp = 2.0 * (qw * qz + qx * qy);
    const double cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz);
    return std::atan2(siny_cosp, cosy_cosp);
}

}  // namespace

void validateLocalLlmPlanShape(const nlohmann::json& plan)
{
    if (!plan.is_object()) throw std::runtime_error("plan must be object");
    const std::set<std::string> required = {
        "plan_id",
        "mode",
        "confidence",
        "reason",
        "steps",
        "communication_policy",
        "requires_human_ack",
    };
    for (const auto& key : required) {
        if (!plan.contains(key)) throw std::runtime_error("missing key: " + key);
    }
    const std::string mode = plan.at("mode").get<std::string>();
    if (!kModes.count(mode)) throw std::runtime_error("invalid mode: " + mode);
    if (!plan.at("steps").is_array() || plan.at("steps").empty() || plan.at("steps").size() > 6) {
        throw std::runtime_error("steps length must be 1..6");
    }
    for (const auto& step : plan.at("steps")) {
        if (!step.is_object()) throw std::runtime_error("step must be object");
        if (!step.contains("step_id") || !step.contains("tool") || !step.contains("arguments")) {
            throw std::runtime_error("step must contain step_id, tool, arguments");
        }
        const std::string tool = step.at("tool").get<std::string>();
        if (!kTools.count(tool)) throw std::runtime_error("invalid tool: " + tool);
        if (!step.at("arguments").is_object()) throw std::runtime_error("step arguments must be object");
    }

    const auto& comm = plan.at("communication_policy");
    if (!comm.is_object()) throw std::runtime_error("communication_policy must be object");
    if (!comm.contains("mode") || !comm.contains("send") || !comm.contains("drop")) {
        throw std::runtime_error("communication_policy missing mode/send/drop");
    }
    const std::string comm_mode = comm.at("mode").get<std::string>();
    if (!kCommunicationModes.count(comm_mode)) throw std::runtime_error("invalid communication mode: " + comm_mode);
    if (!comm.at("send").is_array() || !comm.at("drop").is_array()) {
        throw std::runtime_error("communication send/drop must be arrays");
    }
}

std::vector<std::string> collectTools(const nlohmann::json& plan)
{
    std::vector<std::string> tools;
    if (!plan.contains("steps") || !plan.at("steps").is_array()) return tools;
    for (const auto& step : plan.at("steps")) {
        if (step.contains("tool") && step.at("tool").is_string()) {
            tools.push_back(step.at("tool").get<std::string>());
        }
    }
    return tools;
}

PlanExecutor::PlanExecutor(nlohmann::json map_registry)
    : registry_(std::move(map_registry))
{
}

ExecutorResult PlanExecutor::reject(const std::string& reason) const
{
    ExecutorResult result;
    result.accepted = false;
    result.reason = reason;
    return result;
}

const nlohmann::json* PlanExecutor::findMap(const std::string& map_id) const
{
    if (!registry_.contains("maps") || !registry_.at("maps").is_array()) return nullptr;
    for (const auto& map : registry_.at("maps")) {
        if (map.value("map_id", "") == map_id) return &map;
    }
    return nullptr;
}

const nlohmann::json* PlanExecutor::findNode(const nlohmann::json& map, const std::string& node_id) const
{
    if (!map.contains("topology_nodes") || !map.at("topology_nodes").is_array()) return nullptr;
    for (const auto& node : map.at("topology_nodes")) {
        if (node.value("node_id", "") == node_id || node.value("name", "") == node_id) return &node;
        if (node.contains("aliases") && node.at("aliases").is_array()) {
            for (const auto& alias : node.at("aliases")) {
                if (alias.is_string() && alias.get<std::string>() == node_id) return &node;
            }
        }
    }
    return nullptr;
}

nlohmann::json PlanExecutor::poseToUnitreeJson(const nlohmann::json& node, double speed_override, int mode_override) const
{
    const auto& pose = node.at("pose");
    const double qx = jsonDouble(pose, "q_x", 0.0);
    const double qy = jsonDouble(pose, "q_y", 0.0);
    const double qz = jsonDouble(pose, "q_z", 0.0);
    const double qw = jsonDouble(pose, "q_w", 1.0);
    const double yaw = pose.contains("yaw") ? jsonDouble(pose, "yaw", 0.0) : yawFromQuaternion(qx, qy, qz, qw);
    return {
        {"name", node.value("node_id", "llm_goal")},
        {"x", jsonDouble(pose, "x", 0.0)},
        {"y", jsonDouble(pose, "y", 0.0)},
        {"z", jsonDouble(pose, "z", 0.0)},
        {"q_x", qx},
        {"q_y", qy},
        {"q_z", qz},
        {"q_w", qw},
        {"yaw", yaw},
        {"speed", speed_override >= 0.0 ? speed_override : jsonDouble(pose, "speed", 0.45)},
        {"mode", mode_override >= 0 ? mode_override : jsonInt(pose, "mode", 0)},
    };
}

ExecutorResult PlanExecutor::dryRun(const nlohmann::json& plan) const
{
    try {
        validateLocalLlmPlanShape(plan);
    } catch (const std::exception& exc) {
        return reject(std::string("invalid_plan_shape:") + exc.what());
    }

    if (plan.contains("api_id") || plan.contains("ROBOT_API_ID") || plan.contains("raw_api")) {
        return reject("raw_api_id_is_forbidden");
    }

    const std::string mode = plan.value("mode", "");
    if (mode != "mapped_navigation") {
        ExecutorResult result;
        result.accepted = true;
        result.reason = "non_navigation_plan_accepted_for_state_machine";
        result.dry_run_sequence = nlohmann::json::array({{{"step", "state_machine_handles_mode"}, {"mode", mode}}});
        return result;
    }

    const nlohmann::json* nav_step = nullptr;
    bool has_wait = false;
    for (const auto& step : plan.at("steps")) {
        const std::string tool = step.value("tool", "");
        if (tool == "create_navigation_subgoal") nav_step = &step;
        if (tool == "wait_until") has_wait = true;
    }
    if (!nav_step) return reject("mapped_navigation_missing_create_navigation_subgoal");
    if (!has_wait) return reject("mapped_navigation_missing_wait_until");

    const auto& args = nav_step->at("arguments");
    if (args.contains("target_node_id")) return reject("use_target_node_not_target_node_id");
    if (!args.contains("map_id") || !args.contains("target_node")) return reject("navigation_args_require_map_id_and_target_node");

    const std::string map_id = args.at("map_id").get<std::string>();
    const std::string target_node = args.at("target_node").get<std::string>();
    const auto* map = findMap(map_id);
    if (!map) return reject("unknown_map_id:" + map_id);
    const auto* node = findNode(*map, target_node);
    if (!node) return reject("unknown_target_node:" + target_node);

    const double speed = args.contains("speed_mps") ? args.at("speed_mps").get<double>() : -1.0;
    const int mode_override = args.contains("mode") ? args.at("mode").get<int>() : -1;
    if (speed > 0.45) return reject("speed_exceeds_limit");

    nlohmann::json command = {
        {"action", "navigate_to_pose"},
        {"map_id", map_id},
        {"target_node", node->value("node_id", target_node)},
        {"target_pose", poseToUnitreeJson(*node, speed, mode_override)},
    };

    ExecutorResult result;
    result.accepted = true;
    result.reason = "dry_run_navigation_command_ready";
    result.slam_command = command;
    result.dry_run_sequence = nlohmann::json::array({
        {{"step", "resume_navigation"}, {"action", "1202_resume"}},
        {{"step", "navigate_to_pose"}, {"command", command}},
        {{"step", "wait_until"}, {"source", "/slam_info"}, {"condition", "ctrl_info arrived or FINISHED"}},
        {{"step", "pause_navigation"}, {"action", "1201_pause"}},
    });
    return result;
}

}  // namespace go2w
