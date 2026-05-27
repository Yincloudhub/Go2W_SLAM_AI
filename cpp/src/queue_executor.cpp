#include "go2w/queue_executor.hpp"
#include "go2w/task_queue_validator.hpp"

#include <chrono>
#include <cmath>
#include <iomanip>
#include <map>
#include <ostream>
#include <sstream>
#include <thread>
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

double jsonNumber(const nlohmann::json* value, double fallback = 0.0)
{
    if (!value || !value->is_number()) return fallback;
    return value->get<double>();
}

double poseDistance(const nlohmann::json& world_state_result, const nlohmann::json& target_pose)
{
    const auto* pose = objectAt(world_state_result, {"world_state", "current_pose", "pose"});
    if (!pose || !pose->is_object()) return -1.0;
    const double x = jsonNumber(objectAt(*pose, {"x"}));
    const double y = jsonNumber(objectAt(*pose, {"y"}));
    const double tx = jsonNumber(objectAt(target_pose, {"x"}));
    const double ty = jsonNumber(objectAt(target_pose, {"y"}));
    return std::hypot(x - tx, y - ty);
}

double positiveOr(double value, double fallback)
{
    return value > 0.0 ? value : fallback;
}

int nonNegativeOr(int value, int fallback)
{
    return value >= 0 ? value : fallback;
}

std::chrono::steady_clock::duration secondsDuration(double value, double fallback)
{
    return std::chrono::duration_cast<std::chrono::steady_clock::duration>(
        std::chrono::duration<double>(positiveOr(value, fallback)));
}

std::string displayName(const std::string& target_node, const std::string& target_name)
{
    return target_name.empty() ? target_node : target_name;
}

nlohmann::json feedbackMessage(
    const std::string& phase,
    const std::string& severity,
    const std::string& text,
    const std::string& target_node,
    const std::string& target_name,
    double distance_m = -1.0)
{
    nlohmann::json message = {
        {"phase", phase},
        {"severity", severity},
        {"channel", "operator_display"},
        {"llm_surface", true},
        {"text", text},
        {"target_node", target_node},
        {"target_name", target_name},
    };
    if (distance_m >= 0.0) message["distance_to_target_m"] = distance_m;
    return message;
}

void incrementDropped(nlohmann::json* event, const std::string& key, int count = 1)
{
    if (!event || count <= 0) return;
    int current = 0;
    auto* existing = objectAt(*event, {"performance", "dropped_counts", key.c_str()});
    if (existing && existing->is_number_integer()) current = existing->get<int>();
    (*event)["performance"]["dropped_counts"][key] = current + count;
}

void pushLimited(nlohmann::json* event, const std::string& key, nlohmann::json value, int max_items, const std::string& dropped_key)
{
    if (!event) return;
    if (max_items <= 0) {
        incrementDropped(event, dropped_key);
        return;
    }
    auto& array = (*event)[key];
    if (!array.is_array()) array = nlohmann::json::array();
    while (array.size() >= static_cast<std::size_t>(max_items)) {
        array.erase(array.begin());
        incrementDropped(event, dropped_key);
    }
    array.push_back(std::move(value));
}

void pushFeedback(nlohmann::json* event, const nlohmann::json& message, int max_items)
{
    pushLimited(event, "operator_feedback", message, max_items, "operator_feedback");
}

nlohmann::json llmFeedbackRequest(
    const std::string& phase,
    const std::string& target_node,
    const std::string& target_name,
    double distance_m,
    const std::string& reason = "")
{
    nlohmann::json request = {
        {"phase", phase},
        {"target_node", target_node},
        {"target_name", target_name},
        {"reason", reason},
        {"instruction", "用一句自然中文向操作者说明当前正在前往哪里、剩余大致距离、是否到达或是否需要人工干预。"},
    };
    if (distance_m >= 0.0) request["distance_to_target_m"] = distance_m;
    return request;
}

std::string templateLlmFeedbackText(const nlohmann::json& request)
{
    const std::string phase = request.value("phase", "progress");
    const std::string name = request.value("target_name", request.value("target_node", "目标点"));
    if (phase == "queued") return "已将" + name + "加入任务队列，等待执行。";
    if (phase == "arrived") return "已到达" + name + "，导航已暂停。";
    if (phase == "blocked") return "前往" + name + "已被安全门阻断，请人工确认。";
    if (phase == "timeout") return "还未确认到达" + name + "，已停止后续队列，请人工确认。";
    if (request.contains("distance_to_target_m") && request["distance_to_target_m"].is_number()) {
        std::ostringstream msg;
        msg << "正在前往" << name << "，距离约" << std::fixed << std::setprecision(2)
            << request["distance_to_target_m"].get<double>() << "米。";
        return msg.str();
    }
    return "正在前往" + name + "。";
}

void pushLlmFeedback(nlohmann::json* event, const nlohmann::json& request, int max_items)
{
    if (!event) return;
    pushLimited(event, "llm_feedback_requests", request, max_items, "llm_feedback");
    nlohmann::json result = {
        {"phase", request.value("phase", "")},
        {"target_node", request.value("target_node", "")},
        {"target_name", request.value("target_name", "")},
        {"source", "template"},
        {"llm_surface", true},
        {"text", templateLlmFeedbackText(request)},
    };
    if (request.contains("distance_to_target_m")) result["distance_to_target_m"] = request["distance_to_target_m"];
    pushLimited(event, "llm_feedback_results", result, max_items, "llm_feedback");
}

std::map<std::string, nlohmann::json> commandsByTarget(const SemanticRoute& route)
{
    std::map<std::string, nlohmann::json> commands;
    for (const auto& command : route.slam_commands) {
        if (command.is_object()) commands[command.value("target_node", "")] = command;
    }
    return commands;
}

}  // namespace

QueueExecutor::QueueExecutor(QueueExecutorConfig config)
    : config_(std::move(config))
{
}

nlohmann::json QueueExecutor::sendGatewayCommand(const nlohmann::json& command) const
{
    GatewayClient client({config_.gateway_client, config_.network_interface, config_.gateway_timeout_s});
    return client.send(command).response;
}

nlohmann::json QueueExecutor::getWorldState() const
{
    return sendGatewayCommand({{"action", "get_world_state"}});
}

bool QueueExecutor::waitForArrival(
    const nlohmann::json& target_pose,
    const std::string& target_node,
    const std::string& target_name,
    const SafetyGate& safety_gate,
    nlohmann::json* event,
    std::ostream& log) const
{
    const auto start = std::chrono::steady_clock::now();
    const auto poll_interval = secondsDuration(config_.feedback_policy.slam_poll_interval_s, 1.0);
    const auto ui_interval = secondsDuration(config_.feedback_policy.ui_refresh_interval_s, 1.0);
    const auto feedback_interval = secondsDuration(config_.feedback_policy.operator_feedback_interval_s, 5.0);
    const auto llm_feedback_interval = secondsDuration(config_.feedback_policy.llm_feedback_interval_s, 8.0);
    auto last_ui = start - ui_interval;
    auto last_feedback = start - feedback_interval;
    auto last_llm_feedback = start - llm_feedback_interval;
    const int max_arrival_samples = nonNegativeOr(config_.feedback_policy.max_arrival_samples, 120);
    const int max_feedback_events = nonNegativeOr(config_.feedback_policy.max_feedback_events, 120);
    const int max_llm_feedback_events = nonNegativeOr(config_.feedback_policy.max_llm_feedback_events, 40);
    int entered_count = 0;
    int consecutive_errors = 0;
    int poll_overruns = 0;
    double max_loop_elapsed_s = 0.0;
    const std::string name = displayName(target_node, target_name);
    if (event) {
        (*event)["performance"] = {
            {"poll_interval_s", config_.feedback_policy.slam_poll_interval_s},
            {"ui_interval_s", config_.feedback_policy.ui_refresh_interval_s},
            {"operator_feedback_interval_s", config_.feedback_policy.operator_feedback_interval_s},
            {"llm_feedback_interval_s", config_.feedback_policy.llm_feedback_interval_s},
            {"max_arrival_samples", max_arrival_samples},
            {"max_feedback_events", max_feedback_events},
            {"max_llm_feedback_events", max_llm_feedback_events},
            {"poll_overruns", 0},
            {"max_loop_elapsed_s", 0.0},
            {"dropped_counts", {{"arrival_samples", 0}, {"operator_feedback", 0}, {"llm_feedback", 0}}},
        };
    }

    const auto finishLoop = [&](const std::chrono::steady_clock::time_point& loop_start) {
        const auto elapsed = std::chrono::steady_clock::now() - loop_start;
        const double elapsed_s = std::chrono::duration<double>(elapsed).count();
        if (elapsed_s > max_loop_elapsed_s) max_loop_elapsed_s = elapsed_s;
        if (event) {
            (*event)["performance"]["max_loop_elapsed_s"] = max_loop_elapsed_s;
        }
        if (elapsed >= poll_interval) {
            ++poll_overruns;
            if (event) (*event)["performance"]["poll_overruns"] = poll_overruns;
            return;
        }
        std::this_thread::sleep_for(poll_interval - elapsed);
    };

    while (std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count() < config_.arrival_monitor_s) {
        const auto loop_start = std::chrono::steady_clock::now();
        try {
            const auto state = getWorldState();
            consecutive_errors = 0;
            const double distance = poseDistance(state, target_pose);
            if (event) {
                pushLimited(event, "arrival_samples", {
                    {"distance_to_target_m", distance},
                    {"navigation", objectAt(state, {"world_state", "navigation"}) ? *objectAt(state, {"world_state", "navigation"}) : nlohmann::json(nullptr)},
                    {"localization", objectAt(state, {"world_state", "localization"}) ? *objectAt(state, {"world_state", "localization"}) : nlohmann::json(nullptr)},
                    {"safety", objectAt(state, {"world_state", "safety"}) ? *objectAt(state, {"world_state", "safety"}) : nlohmann::json(nullptr)},
                }, max_arrival_samples, "arrival_samples");
            }
            if (config_.feedback_policy.runtime_safety_check) {
                const SafetyDecision runtime_safety = safety_gate.evaluateWorldState(state);
                if (!runtime_safety.allowed) {
                    if (event) {
                        (*event)["blocked_reason"] = runtime_safety.reason;
                        (*event)["runtime_safety"] = {
                            {"allowed", runtime_safety.allowed},
                            {"reason", runtime_safety.reason},
                            {"recommended_mode", runtime_safety.recommended_mode},
                        };
                    }
                    pushFeedback(event, feedbackMessage("blocked", "error", "运行中安全门阻断，已停止等待：" + runtime_safety.reason, target_node, name, distance), max_feedback_events);
                    pushLlmFeedback(event, llmFeedbackRequest("blocked", target_node, name, distance, runtime_safety.reason), max_llm_feedback_events);
                    return false;
                }
            }

            const auto now = std::chrono::steady_clock::now();
            if (now - last_ui >= ui_interval) {
                log << "  到点监控: distance=" << std::fixed << std::setprecision(2) << distance << "m\n";
                last_ui = now;
            }
            if (now - last_feedback >= feedback_interval) {
                std::ostringstream msg;
                msg << "正在前往" << name;
                if (distance >= 0.0) msg << "，距离约" << std::fixed << std::setprecision(2) << distance << "米";
                pushFeedback(event, feedbackMessage("progress", "info", msg.str(), target_node, name, distance), max_feedback_events);
                last_feedback = now;
            }
            if (now - last_llm_feedback >= llm_feedback_interval) {
                pushLlmFeedback(event, llmFeedbackRequest("progress", target_node, name, distance), max_llm_feedback_events);
                last_llm_feedback = now;
            }
            if (distance >= 0.0 && distance <= config_.arrival_distance_m) {
                ++entered_count;
                if (entered_count >= 2) {
                    const auto pause_result = sendGatewayCommand({{"action", "pause_navigation"}});
                    if (event) (*event)["pause_result"] = pause_result;
                    log << "  已到达阈值，已发送暂停 accepted=" << (pause_result.value("accepted", false) ? "true" : "false") << "\n";
                    pushFeedback(event, feedbackMessage("arrived", "ok", "已到达" + name + "，导航已暂停。", target_node, name, distance), max_feedback_events);
                    pushLlmFeedback(event, llmFeedbackRequest("arrived", target_node, name, distance, "arrived and paused"), max_llm_feedback_events);
                    return true;
                }
            } else {
                entered_count = 0;
            }
        } catch (const std::exception& exc) {
            ++consecutive_errors;
            if (event) pushLimited(event, "arrival_errors", exc.what(), max_arrival_samples, "arrival_samples");
            log << "  到点监控失败: " << exc.what() << "\n";
            if (consecutive_errors >= config_.feedback_policy.max_consecutive_gateway_errors) {
                const std::string reason = "gateway feedback failed repeatedly: " + std::string(exc.what());
                if (event) (*event)["blocked_reason"] = reason;
                pushFeedback(event, feedbackMessage("blocked", "error", "连续读取 SLAM 状态失败，停止等待并请求人工确认。", target_node, name), max_feedback_events);
                pushLlmFeedback(event, llmFeedbackRequest("blocked", target_node, name, -1.0, reason), max_llm_feedback_events);
                return false;
            }
        }
        finishLoop(loop_start);
    }
    pushFeedback(event, feedbackMessage("timeout", "warning", "未在限定时间内确认到达" + name + "，停止后续队列。", target_node, name), max_feedback_events);
    pushLlmFeedback(event, llmFeedbackRequest("timeout", target_node, name, -1.0, "arrival threshold not reached before timeout"), max_llm_feedback_events);
    return false;
}

QueueExecutionResult QueueExecutor::execute(const SemanticRoute& route) const
{
    QueueExecutionResult result;
    std::ostringstream out;
    const TaskQueueValidationResult validation = validateTaskQueue(route.task_queue);
    if (!validation.valid) {
        result.exit_code = 2;
        result.execution = {
            {"queue_id", route.task_queue.value("queue_id", "cpp_queue")},
            {"executed", false},
            {"completed", false},
            {"failed_step", nullptr},
            {"blocked_reason", validation.summary()},
            {"events", nlohmann::json::array()},
        };
        result.stdout_text = "任务队列校验失败：" + validation.summary() + "\n";
        return result;
    }

    nlohmann::json execution = {
        {"queue_id", route.task_queue.value("queue_id", "cpp_queue")},
        {"executed", config_.execute_enabled},
        {"completed", false},
        {"failed_step", nullptr},
        {"blocked_reason", ""},
        {"feedback_policy", {
            {"slam_poll_interval_s", config_.feedback_policy.slam_poll_interval_s},
            {"ui_refresh_interval_s", config_.feedback_policy.ui_refresh_interval_s},
            {"operator_feedback_interval_s", config_.feedback_policy.operator_feedback_interval_s},
            {"llm_feedback_interval_s", config_.feedback_policy.llm_feedback_interval_s},
            {"max_consecutive_gateway_errors", config_.feedback_policy.max_consecutive_gateway_errors},
            {"max_arrival_samples", config_.feedback_policy.max_arrival_samples},
            {"max_feedback_events", config_.feedback_policy.max_feedback_events},
            {"max_llm_feedback_events", config_.feedback_policy.max_llm_feedback_events},
            {"runtime_safety_check", config_.feedback_policy.runtime_safety_check},
        }},
        {"events", nlohmann::json::array()},
    };

    SafetyLimits safety_limits = config_.safety_limits;
    safety_limits.arrival_distance_m = config_.arrival_distance_m;
    const SafetyGate safety_gate(safety_limits);
    const auto commands = commandsByTarget(route);
    const auto steps = route.task_queue.value("steps", nlohmann::json::array());
    const int max_feedback_events = nonNegativeOr(config_.feedback_policy.max_feedback_events, 120);
    const int max_llm_feedback_events = nonNegativeOr(config_.feedback_policy.max_llm_feedback_events, 40);

    for (const auto& step : steps) {
        const std::string step_id = step.value("task_id", step.value("step_id", ""));
        const std::string action = step.value("action", "");
        const std::string target_node = step.value("target_node", "");
        const std::string target_name = step.value("target_name", target_node);
        nlohmann::json event = {
            {"task_id", step_id},
            {"action", action},
            {"target_node", target_node},
            {"target_name", target_name},
        };

        if (action == "capture_keyframe") {
            event["status"] = "ok";
            event["result"] = {
                {"captured", false},
                {"target_node", target_node},
                {"reason", config_.execute_enabled ? "capture command not configured; recorded semantic keyframe event only" : "dry run; capture not executed"},
            };
            execution["events"].push_back(event);
            out << "  capture_keyframe：当前C++原型记录语义事件，真实相机命令下一步接入。\n";
            continue;
        }
        if (action == "report") {
            event["status"] = "ok";
            event["message"] = step.value("message", "");
            execution["events"].push_back(event);
            if (step.contains("message") && step.at("message").is_string()) out << "  report：" << step.at("message").get<std::string>() << "\n";
            continue;
        }
        if (action != "navigate") {
            event["status"] = "skipped";
            event["reason"] = "unsupported task action";
            execution["events"].push_back(event);
            continue;
        }

        const auto command_it = commands.find(target_node);
        if (command_it == commands.end()) {
            const std::string reason = "missing slam command for target " + target_node;
            event["status"] = "failed";
            event["blocked_reason"] = reason;
            execution["events"].push_back(event);
            execution["failed_step"] = step_id;
            execution["blocked_reason"] = reason;
            result.exit_code = 3;
            result.execution = execution;
            result.stdout_text = out.str();
            return result;
        }

        const auto& command = command_it->second;
        if (!config_.execute_enabled) {
            event["status"] = "dry_run";
            event["slam_command"] = command;
            pushFeedback(&event, feedbackMessage("queued", "info", "干跑：将前往" + displayName(target_node, target_name) + "，不会下发运动。", target_node, displayName(target_node, target_name)), max_feedback_events);
            pushLlmFeedback(&event, llmFeedbackRequest("queued", target_node, displayName(target_node, target_name), -1.0, "dry run"), max_llm_feedback_events);
            execution["events"].push_back(event);
            continue;
        }

        const auto state = getWorldState();
        const SafetyDecision safety = safety_gate.evaluateBeforeNavigation(state, target_node, command.at("target_pose"));
        event["preflight"] = {{"allowed", safety.allowed}, {"reason", safety.reason}, {"recommended_mode", safety.recommended_mode}};
        if (!safety.allowed) {
            event["status"] = "blocked";
            event["blocked_reason"] = safety.reason;
            pushFeedback(&event, feedbackMessage("blocked", "error", "安全检查未通过：" + safety.reason, target_node, displayName(target_node, target_name)), max_feedback_events);
            pushLlmFeedback(&event, llmFeedbackRequest("blocked", target_node, displayName(target_node, target_name), -1.0, safety.reason), max_llm_feedback_events);
            execution["events"].push_back(event);
            execution["failed_step"] = step_id;
            execution["blocked_reason"] = safety.reason;
            result.exit_code = 2;
            result.execution = execution;
            out << "安全检查：未通过，原因：" << safety.reason << "\n";
            result.stdout_text = out.str();
            return result;
        }

        out << "安全检查：通过，原因：" << safety.reason << "，模式：" << safety.recommended_mode << "\n";
        out << "下发导航：" << target_node << "\n";
        pushFeedback(&event, feedbackMessage("departing", "info", "正在前往" + displayName(target_node, target_name) + "。", target_node, displayName(target_node, target_name)), max_feedback_events);
        const auto send_result = sendGatewayCommand(command);
        event["send_result"] = send_result;
        out << "  gateway accepted=" << (send_result.value("accepted", false) ? "true" : "false") << "\n";
        if (!send_result.value("accepted", false)) {
            const std::string reason = "gateway rejected navigation";
            event["status"] = "failed";
            event["blocked_reason"] = reason;
            execution["events"].push_back(event);
            execution["failed_step"] = step_id;
            execution["blocked_reason"] = reason;
            result.exit_code = 3;
            result.execution = execution;
            result.stdout_text = out.str();
            return result;
        }

        const bool arrived = waitForArrival(command.at("target_pose"), target_node, target_name, safety_gate, &event, out);
        event["status"] = arrived ? "ok" : "failed";
        if (!arrived) {
            event["blocked_reason"] = "arrival threshold not reached before timeout";
            execution["events"].push_back(event);
            execution["failed_step"] = step_id;
            execution["blocked_reason"] = event["blocked_reason"];
            result.exit_code = 4;
            result.execution = execution;
            out << "  超时未进入到点阈值，停止后续队列。\n";
            result.stdout_text = out.str();
            return result;
        }
        execution["events"].push_back(event);
    }

    execution["completed"] = true;
    result.exit_code = 0;
    result.execution = execution;
    out << "队列执行完成。\n";
    result.stdout_text = out.str();
    return result;
}

}  // namespace go2w
