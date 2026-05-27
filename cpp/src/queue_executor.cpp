#include "go2w/queue_executor.hpp"

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

bool QueueExecutor::waitForArrival(const nlohmann::json& target_pose, nlohmann::json* event, std::ostream& log) const
{
    const auto start = std::chrono::steady_clock::now();
    int entered_count = 0;
    while (std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count() < config_.arrival_monitor_s) {
        try {
            const auto state = getWorldState();
            const double distance = poseDistance(state, target_pose);
            if (event) {
                (*event)["arrival_samples"].push_back({
                    {"distance_to_target_m", distance},
                    {"navigation", objectAt(state, {"world_state", "navigation"}) ? *objectAt(state, {"world_state", "navigation"}) : nlohmann::json(nullptr)},
                    {"localization", objectAt(state, {"world_state", "localization"}) ? *objectAt(state, {"world_state", "localization"}) : nlohmann::json(nullptr)},
                    {"safety", objectAt(state, {"world_state", "safety"}) ? *objectAt(state, {"world_state", "safety"}) : nlohmann::json(nullptr)},
                });
            }
            log << "  到点监控: distance=" << std::fixed << std::setprecision(2) << distance << "m\n";
            if (distance >= 0.0 && distance <= config_.arrival_distance_m) {
                ++entered_count;
                if (entered_count >= 2) {
                    const auto pause_result = sendGatewayCommand({{"action", "pause_navigation"}});
                    if (event) (*event)["pause_result"] = pause_result;
                    log << "  已到达阈值，已发送暂停 accepted=" << (pause_result.value("accepted", false) ? "true" : "false") << "\n";
                    return true;
                }
            } else {
                entered_count = 0;
            }
        } catch (const std::exception& exc) {
            if (event) (*event)["arrival_errors"].push_back(exc.what());
            log << "  到点监控失败: " << exc.what() << "\n";
        }
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }
    return false;
}

QueueExecutionResult QueueExecutor::execute(const SemanticRoute& route) const
{
    QueueExecutionResult result;
    std::ostringstream out;
    nlohmann::json execution = {
        {"queue_id", route.task_queue.value("queue_id", "cpp_queue")},
        {"executed", config_.execute_enabled},
        {"completed", false},
        {"failed_step", nullptr},
        {"blocked_reason", ""},
        {"events", nlohmann::json::array()},
    };

    SafetyLimits safety_limits = config_.safety_limits;
    safety_limits.arrival_distance_m = config_.arrival_distance_m;
    const SafetyGate safety_gate(safety_limits);
    const auto commands = commandsByTarget(route);
    const auto steps = route.task_queue.value("steps", nlohmann::json::array());

    for (const auto& step : steps) {
        const std::string step_id = step.value("step_id", "");
        const std::string action = step.value("action", "");
        const std::string target_node = step.value("target_node", "");
        nlohmann::json event = {
            {"step_id", step_id},
            {"action", action},
            {"target_node", target_node},
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
            execution["events"].push_back(event);
            continue;
        }

        const auto state = getWorldState();
        const SafetyDecision safety = safety_gate.evaluateBeforeNavigation(state, target_node, command.at("target_pose"));
        event["preflight"] = {{"allowed", safety.allowed}, {"reason", safety.reason}, {"recommended_mode", safety.recommended_mode}};
        if (!safety.allowed) {
            event["status"] = "blocked";
            event["blocked_reason"] = safety.reason;
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

        const bool arrived = waitForArrival(command.at("target_pose"), &event, out);
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
