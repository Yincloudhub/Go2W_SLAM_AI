#include "go2w/task_queue_validator.hpp"

#include <set>
#include <sstream>

namespace go2w {
namespace {

const std::set<std::string> kModes = {"sequential"};
const std::set<std::string> kStatuses = {"planned", "running", "completed", "failed", "blocked"};
const std::set<std::string> kSources = {"semantic_topology", "deterministic_cpp", "llm_fallback", "operator_panel", "scripted"};
const std::set<std::string> kActions = {
    "navigate",
    "wait_until",
    "capture_keyframe",
    "report",
    "speak",
    "ask_confirm",
    "hold_position",
    "set_communication_policy",
};
const std::set<std::string> kStepStatuses = {"pending", "running", "ok", "failed", "blocked", "dry_run", "skipped"};
const std::set<std::string> kCommModes = {"normal", "semantic_only", "keyframe_low_rate", "hold_remote"};
const std::set<std::string> kSendItems = {
    "task_state",
    "risk_events",
    "keyframe",
    "semantic_topology",
    "navigation_feedback",
    "world_state_summary",
};
const std::set<std::string> kDropItems = {"raw_video", "dense_pointcloud", "full_log", "high_rate_images"};

void addError(TaskQueueValidationResult& result, const std::string& error)
{
    result.valid = false;
    result.errors.push_back(error);
}

bool nonEmptyString(const nlohmann::json& object, const char* key)
{
    return object.contains(key) && object.at(key).is_string() && !object.at(key).get<std::string>().empty();
}

bool stringIn(const nlohmann::json& object, const char* key, const std::set<std::string>& allowed)
{
    return object.contains(key) && object.at(key).is_string() && allowed.count(object.at(key).get<std::string>()) > 0;
}

void validateStringArray(
    const nlohmann::json& object,
    const char* key,
    const std::set<std::string>& allowed,
    TaskQueueValidationResult& result)
{
    if (!object.contains(key) || !object.at(key).is_array()) {
        addError(result, std::string("communication_policy.") + key + " must be array");
        return;
    }
    for (const auto& item : object.at(key)) {
        if (!item.is_string() || allowed.count(item.get<std::string>()) == 0) {
            addError(result, std::string("communication_policy.") + key + " contains invalid item");
        }
    }
}

void validateCommunicationPolicy(const nlohmann::json& queue, TaskQueueValidationResult& result)
{
    if (!queue.contains("communication_policy") || !queue.at("communication_policy").is_object()) {
        addError(result, "communication_policy must be object");
        return;
    }
    const auto& policy = queue.at("communication_policy");
    if (!stringIn(policy, "mode", kCommModes)) addError(result, "communication_policy.mode is invalid");
    validateStringArray(policy, "send", kSendItems, result);
    validateStringArray(policy, "drop", kDropItems, result);
}

std::string stepId(const nlohmann::json& step)
{
    if (nonEmptyString(step, "task_id")) return step.at("task_id").get<std::string>();
    if (nonEmptyString(step, "step_id")) return step.at("step_id").get<std::string>();
    return "";
}

}  // namespace

std::string TaskQueueValidationResult::summary() const
{
    if (valid) return "task_queue is valid";
    std::ostringstream ss;
    for (std::size_t i = 0; i < errors.size(); ++i) {
        if (i) ss << "; ";
        ss << errors[i];
    }
    return ss.str();
}

TaskQueueValidationResult validateTaskQueue(const nlohmann::json& task_queue)
{
    TaskQueueValidationResult result;
    if (!task_queue.is_object()) {
        addError(result, "task_queue must be object");
        return result;
    }

    if (!nonEmptyString(task_queue, "queue_id")) addError(result, "queue_id must be non-empty string");
    if (!stringIn(task_queue, "mode", kModes)) addError(result, "mode must be sequential");
    if (!stringIn(task_queue, "status", kStatuses)) addError(result, "status is invalid");
    if (!stringIn(task_queue, "source", kSources)) addError(result, "source is invalid");

    if (!task_queue.contains("targets") || !task_queue.at("targets").is_array()) {
        addError(result, "targets must be array");
    } else {
        for (const auto& target : task_queue.at("targets")) {
            if (!target.is_string() || target.get<std::string>().empty()) addError(result, "targets contains invalid target");
        }
    }

    validateCommunicationPolicy(task_queue, result);

    if (!task_queue.contains("steps") || !task_queue.at("steps").is_array() || task_queue.at("steps").empty()) {
        addError(result, "steps must be non-empty array");
        return result;
    }
    if (task_queue.at("steps").size() > 24) addError(result, "steps exceeds max length 24");

    std::set<std::string> ids;
    bool has_navigation = false;
    for (const auto& step : task_queue.at("steps")) {
        if (!step.is_object()) {
            addError(result, "step must be object");
            continue;
        }
        const std::string id = stepId(step);
        if (id.empty()) {
            addError(result, "step missing task_id");
        } else if (!ids.insert(id).second) {
            addError(result, "duplicate task_id: " + id);
        }

        if (!stringIn(step, "action", kActions)) {
            addError(result, "step action is invalid");
            continue;
        }
        if (!stringIn(step, "status", kStepStatuses)) addError(result, "step status is invalid");

        const std::string action = step.at("action").get<std::string>();
        if (action == "navigate") {
            has_navigation = true;
            if (!nonEmptyString(step, "target_node")) addError(result, "navigate step requires target_node");
        } else if (action == "capture_keyframe") {
            if (!nonEmptyString(step, "target_node")) addError(result, "capture_keyframe step requires target_node");
        } else if (action == "report" || action == "speak" || action == "ask_confirm") {
            if (!nonEmptyString(step, "message")) addError(result, action + " step requires message");
        }
    }

    if (!has_navigation && task_queue.at("targets").size() > 0) {
        addError(result, "targets present but no navigate step");
    }
    return result;
}

}  // namespace go2w
