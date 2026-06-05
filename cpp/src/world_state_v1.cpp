#include "go2w/world_state_v1.hpp"

#include <chrono>
#include <iomanip>
#include <initializer_list>
#include <sstream>
#include <string>
#include <vector>

namespace go2w {
namespace {

long long nowMs()
{
    return std::chrono::duration_cast<std::chrono::milliseconds>(
               std::chrono::system_clock::now().time_since_epoch())
        .count();
}

const nlohmann::json* objectAt(const nlohmann::json& root, const std::initializer_list<const char*> keys)
{
    const nlohmann::json* current = &root;
    for (const char* key : keys) {
        if (!current->is_object() || !current->contains(key)) return nullptr;
        current = &current->at(key);
    }
    return current;
}

std::string stringAt(const nlohmann::json& root, const std::initializer_list<const char*> keys, const std::string& fallback = "")
{
    const auto* value = objectAt(root, keys);
    if (!value || !value->is_string()) return fallback;
    return value->get<std::string>();
}

bool boolAt(const nlohmann::json& root, const std::initializer_list<const char*> keys, bool fallback)
{
    const auto* value = objectAt(root, keys);
    if (!value || !value->is_boolean()) return fallback;
    return value->get<bool>();
}

bool numberAt(const nlohmann::json& root, const std::initializer_list<const char*> keys, double* out)
{
    const auto* value = objectAt(root, keys);
    if (!value || !value->is_number()) return false;
    *out = value->get<double>();
    return true;
}

long long integerAt(const nlohmann::json& root, const std::initializer_list<const char*> keys, long long fallback)
{
    const auto* value = objectAt(root, keys);
    if (!value || !value->is_number_integer()) return fallback;
    return value->get<long long>();
}

bool containsString(const std::vector<std::string>& values, const std::string& item)
{
    for (const auto& value : values) {
        if (value == item) return true;
    }
    return false;
}

nlohmann::json compactPose(const nlohmann::json* pose)
{
    if (!pose || !pose->is_object()) return nullptr;
    nlohmann::json out = nlohmann::json::object();
    for (const char* key : {"x", "y", "z", "yaw"}) {
        const auto it = pose->find(key);
        if (it != pose->end() && it->is_number()) out[key] = *it;
    }
    return out.empty() ? nlohmann::json(nullptr) : out;
}

std::string obstacleStatus(const nlohmann::json& front_clearance)
{
    if (!front_clearance.is_number()) return "unknown";
    const double value = front_clearance.get<double>();
    if (value < 0.8) return "blocked";
    if (value < 1.5) return "slow";
    return "clear";
}

nlohmann::json availableTools(bool localized, bool motion_allowed, bool map_loaded, bool capture_configured)
{
    nlohmann::json tools = {"safe_hold", "ask_human_confirm", "semantic_report", "cancel_task"};
    if (!localized) tools.push_back("request_relocalization");
    if (map_loaded) {
        tools.push_back(capture_configured ? "capture_keyframe" : "record_keyframe_event");
        tools.push_back("speak");
    }
    if (localized && motion_allowed) {
        tools.push_back("navigate");
        tools.push_back("patrol_route");
        tools.push_back("inspect_area");
        tools.push_back("return_to_base");
    }
    return tools;
}

nlohmann::json refreshPolicy()
{
    return {
        {"sensor_ingest_hz", {{"min", 10.0}, {"max", 20.0}, {"consumer", "internal_safety_only"}}},
        {"world_state_hz", {{"min", 2.0}, {"max", 5.0}, {"consumer", "safety_and_ui"}}},
        {"ui_display_hz", {{"min", 1.0}, {"max", 2.0}, {"consumer", "operator_panel"}}},
        {"llm_feedback", {{"mode", "event_driven"}, {"min_interval_s", 6.0}, {"progress_interval_s", 8.0}}},
    };
}

nlohmann::json normalizePerceptionSummary(const nlohmann::json& value)
{
    return {
        {"source", value.value("source", std::string("unknown"))},
        {"confidence", value.contains("confidence") && value.at("confidence").is_number() ? value.at("confidence") : nlohmann::json(nullptr)},
        {"stale", value.value("stale", false)},
        {"latency_ms", value.contains("latency_ms") && value.at("latency_ms").is_number() ? value.at("latency_ms") : nlohmann::json(nullptr)},
        {"timestamp_ms", value.value("timestamp_ms", nowMs())},
        {"summary", value.value("summary", nlohmann::json::object())},
    };
}

nlohmann::json latestFeedback(const nlohmann::json& queue_execution, const std::string& key)
{
    const auto* events = objectAt(queue_execution, {"events"});
    if (!events || !events->is_array()) return nullptr;
    for (auto it = events->rbegin(); it != events->rend(); ++it) {
        if (!it->is_object() || !it->contains(key) || !it->at(key).is_array() || it->at(key).empty()) continue;
        const auto& latest = it->at(key).back();
        if (latest.is_object()) return latest;
    }
    return nullptr;
}

std::vector<std::string> queueTargets(const nlohmann::json& task_queue)
{
    std::vector<std::string> targets;
    const auto* values = objectAt(task_queue, {"targets"});
    if (!values || !values->is_array()) return targets;
    for (const auto& item : *values) {
        if (item.is_string() && !item.get<std::string>().empty()) targets.push_back(item.get<std::string>());
    }
    return targets;
}

std::string jsonString(const nlohmann::json& value, const std::string& fallback = "")
{
    if (value.is_string()) return value.get<std::string>();
    if (value.is_boolean()) return value.get<bool>() ? "true" : "false";
    if (value.is_number()) return value.dump();
    return fallback;
}

std::string screenString(const nlohmann::json& screen, const char* key, const std::string& fallback = "")
{
    if (!screen.is_object() || !screen.contains(key)) return fallback;
    return jsonString(screen.at(key), fallback);
}

}  // namespace

nlohmann::json buildWorldStateV1(const nlohmann::json& runtime_or_gateway, const nlohmann::json& options)
{
    if (runtime_or_gateway.is_object() && runtime_or_gateway.value("schema_version", 0) == 1 &&
        runtime_or_gateway.contains("localized") && runtime_or_gateway.contains("available_tools")) {
        return runtime_or_gateway;
    }

    const auto* world_ptr = objectAt(runtime_or_gateway, {"world_state"});
    const nlohmann::json& world = world_ptr && world_ptr->is_object() ? *world_ptr : runtime_or_gateway;

    const std::string loc_status = stringAt(world, {"localization", "status"}, stringAt(runtime_or_gateway, {"localization_status"}, ""));
    const std::string slam_status = stringAt(world, {"slam_health", "status"}, stringAt(runtime_or_gateway, {"health_status"}, ""));
    const std::vector<std::string> localized_statuses = {"localized", "localized_or_tracking", "tracking"};
    const std::vector<std::string> good_slam_statuses = {"ok", "degraded", ""};

    nlohmann::json pose = compactPose(objectAt(world, {"current_pose", "pose"}));
    if (pose.is_null()) pose = compactPose(objectAt(runtime_or_gateway, {"relocation_odom"}));
    const bool localized = containsString(localized_statuses, loc_status) && containsString(good_slam_statuses, slam_status) && pose.is_object();

    std::string map_id = options.value("map_id", std::string(""));
    if (map_id.empty()) map_id = stringAt(runtime_or_gateway, {"expected_map_id"}, "");
    if (map_id.empty()) map_id = stringAt(world, {"current_pose", "map_id"}, "unknown");
    const bool map_loaded = !map_id.empty() && map_id != "unknown";
    const bool capture_configured =
        options.value("capture_configured", false) ||
        boolAt(runtime_or_gateway, {"capture_command_configured"}, false) ||
        boolAt(runtime_or_gateway, {"camera_capture_configured"}, false);

    bool safety_allow = boolAt(world, {"safety", "allow_navigation"}, localized);
    if (options.contains("motion_allowed") && options.at("motion_allowed").is_boolean()) {
        safety_allow = options.at("motion_allowed").get<bool>();
    }

    nlohmann::json front_clearance = nullptr;
    double clearance = 0.0;
    if (numberAt(world, {"local_obstacle", "front_clearance_m"}, &clearance)) front_clearance = clearance;
    if (front_clearance.is_null() && options.contains("front_clearance_m") && options.at("front_clearance_m").is_number()) {
        front_clearance = options.at("front_clearance_m");
    }

    nlohmann::json perception_summaries = nlohmann::json::array();
    if (options.contains("perception_summaries") && options.at("perception_summaries").is_array()) {
        for (const auto& item : options.at("perception_summaries")) {
            if (item.is_object()) perception_summaries.push_back(normalizePerceptionSummary(item));
        }
    }
    if (!front_clearance.is_null()) {
        perception_summaries.push_back(normalizePerceptionSummary({
            {"source", "local_obstacle"},
            {"confidence", objectAt(world, {"local_obstacle", "confidence"}) ? *objectAt(world, {"local_obstacle", "confidence"}) : nlohmann::json(0.5)},
            {"stale", boolAt(world, {"local_obstacle", "stale"}, false)},
            {"latency_ms", objectAt(world, {"local_obstacle", "latency_ms"}) ? *objectAt(world, {"local_obstacle", "latency_ms"}) : nlohmann::json(nullptr)},
            {"timestamp_ms", integerAt(world, {"local_obstacle", "timestamp_ms"}, nowMs())},
            {"summary", {{"front_clearance_m", front_clearance}}},
        }));
    }

    const std::string task_phase = options.value("task_phase", localized ? std::string("idle") : std::string("not_localized"));
    const std::string current_node = options.value("current_node", std::string(""));
    nlohmann::json candidate_nodes = options.contains("candidate_nodes") && options.at("candidate_nodes").is_array()
        ? options.at("candidate_nodes")
        : nlohmann::json::array();

    return {
        {"schema_version", 1},
        {"timestamp_ms", integerAt(runtime_or_gateway, {"timestamp_ms"}, integerAt(world, {"timestamp_ms"}, nowMs()))},
        {"localized", localized},
        {"map_loaded", map_loaded},
        {"map_id", map_id},
        {"current_pose", pose},
        {"current_node", current_node.empty() ? nlohmann::json(nullptr) : nlohmann::json(current_node)},
        {"candidate_nodes", candidate_nodes},
        {"front_clearance_m", front_clearance},
        {"obstacle_status", obstacleStatus(front_clearance)},
        {"detected_objects", options.value("detected_objects", nlohmann::json::array())},
        {"network_level", options.value("network_level", std::string("normal"))},
        {"task_phase", task_phase},
        {"last_execution_result", options.value("last_execution_result", std::string(""))},
        {"motion_allowed", localized && safety_allow},
        {"available_tools", availableTools(localized, safety_allow, map_loaded, capture_configured)},
        {"capture_keyframe", {
            {"configured", capture_configured},
            {"mode", capture_configured ? std::string("image_capture") : std::string("semantic_event_only")},
        }},
        {"source_health", {
            {"slam_status", slam_status.empty() ? std::string("unknown") : slam_status},
            {"localization_status", loc_status.empty() ? std::string("unknown") : loc_status},
            {"safety_reason", stringAt(world, {"safety", "reason"}, "")},
        }},
        {"perception_summaries", perception_summaries},
        {"refresh_policy", refreshPolicy()},
    };
}

nlohmann::json buildOperatorDisplayState(
    const nlohmann::json& world_state,
    const nlohmann::json& task_queue,
    const nlohmann::json& queue_execution,
    const std::string& user_command)
{
    const auto llm_feedback = latestFeedback(queue_execution, "llm_feedback_results");
    const auto operator_feedback = latestFeedback(queue_execution, "operator_feedback");
    const auto targets = queueTargets(task_queue);

    std::string task_phase = world_state.value("task_phase", std::string("idle"));
    const std::string blocked_reason = queue_execution.value("blocked_reason", std::string(""));
    if (queue_execution.value("completed", false)) {
        task_phase = "completed";
    } else if (!blocked_reason.empty()) {
        task_phase = "blocked";
    }

    std::string current_target = targets.empty() ? stringAt(world_state, {"current_node"}, "") : targets.front();
    if (current_target.empty()) current_target = "";

    return {
        {"schema_version", 1},
        {"timestamp_ms", world_state.value("timestamp_ms", nowMs())},
        {"refresh_hz", 1.0},
        {"screen", {
            {"task_phase", task_phase},
            {"user_command", user_command},
            {"current_target", current_target},
            {"targets", targets},
            {"localized", world_state.value("localized", false)},
            {"map_loaded", world_state.value("map_loaded", false)},
            {"motion_allowed", world_state.value("motion_allowed", false)},
            {"obstacle_status", world_state.value("obstacle_status", std::string("unknown"))},
            {"front_clearance_m", world_state.contains("front_clearance_m") ? world_state.at("front_clearance_m") : nlohmann::json(nullptr)},
            {"network_level", world_state.value("network_level", std::string("normal"))},
            {"safety_reason", stringAt(world_state, {"source_health", "safety_reason"}, "")},
            {"blocked_reason", blocked_reason},
            {"llm_reply", llm_feedback.is_object() ? llm_feedback.value("text", std::string("")) : std::string("")},
            {"operator_reply", operator_feedback.is_object() ? operator_feedback.value("text", std::string("")) : std::string("")},
        }},
        {"event_budget", {
            {"ui_display_hz", objectAt(world_state, {"refresh_policy", "ui_display_hz"}) ? *objectAt(world_state, {"refresh_policy", "ui_display_hz"}) : nlohmann::json::object()},
            {"llm_feedback", objectAt(world_state, {"refresh_policy", "llm_feedback"}) ? *objectAt(world_state, {"refresh_policy", "llm_feedback"}) : nlohmann::json::object()},
        }},
    };
}

nlohmann::json buildRuntimeLogRecord(
    const nlohmann::json& world_state,
    const nlohmann::json& operator_display,
    const nlohmann::json& task_queue,
    const nlohmann::json& queue_execution,
    const std::string& user_command,
    const nlohmann::json& llm_result)
{
    return {
        {"schema_version", 1},
        {"timestamp_ms", world_state.value("timestamp_ms", nowMs())},
        {"user_command", user_command},
        {"world_state", world_state},
        {"operator_display", operator_display},
        {"task_queue", task_queue},
        {"queue_execution", queue_execution},
        {"llm_result", llm_result},
        {"artifact_policy", {
            {"allow_raw_video", false},
            {"allow_dense_pointcloud", false},
            {"allow_keyframe", true},
            {"allow_semantic_summary", true},
        }},
    };
}

std::string formatOperatorDisplayLine(const nlohmann::json& operator_display, bool weak_link_mode)
{
    const auto* screen_ptr = objectAt(operator_display, {"screen"});
    const nlohmann::json screen = screen_ptr && screen_ptr->is_object() ? *screen_ptr : nlohmann::json::object();

    std::ostringstream ss;
    ss << "phase=" << screenString(screen, "task_phase", "unknown")
       << " | target=" << screenString(screen, "current_target", "")
       << " | loc=" << screenString(screen, "localized", "false")
       << " | map=" << screenString(screen, "map_loaded", "false")
       << " | motion=" << screenString(screen, "motion_allowed", "false")
       << " | obstacle=" << screenString(screen, "obstacle_status", "unknown")
       << " | net=" << screenString(screen, "network_level", "normal");
    if (!weak_link_mode) {
        if (screen.contains("front_clearance_m") && screen.at("front_clearance_m").is_number()) {
            ss << " | front=" << std::fixed << std::setprecision(2) << screen.at("front_clearance_m").get<double>() << "m";
        }
        const std::string safety = screenString(screen, "safety_reason", "");
        const std::string blocked = screenString(screen, "blocked_reason", "");
        const std::string llm = screenString(screen, "llm_reply", "");
        const std::string op = screenString(screen, "operator_reply", "");
        if (!safety.empty()) ss << " | safety=" << safety;
        if (!blocked.empty()) ss << " | blocked=" << blocked;
        if (!llm.empty()) ss << " | llm=" << llm;
        if (!op.empty()) ss << " | operator=" << op;
    }
    return ss.str();
}

}  // namespace go2w
