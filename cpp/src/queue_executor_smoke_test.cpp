#include "go2w/queue_executor.hpp"

#include <iostream>
#include <stdexcept>
#include <string>

namespace {

void require(bool condition, const std::string& message)
{
    if (!condition) throw std::runtime_error(message);
}

go2w::SemanticRoute captureRoute()
{
    go2w::SemanticRoute route;
    route.matched = true;
    route.task_queue = {
        {"queue_id", "queue_capture_test"},
        {"mode", "sequential"},
        {"status", "planned"},
        {"source", "scripted"},
        {"targets", nlohmann::json::array()},
        {"communication_policy", {
            {"mode", "normal"},
            {"send", nlohmann::json::array({"task_state"})},
            {"drop", nlohmann::json::array()},
        }},
        {"steps", nlohmann::json::array({
            {
                {"task_id", "capture_1"},
                {"action", "capture_keyframe"},
                {"status", "pending"},
                {"target_node", "node_a"},
                {"target_name", "Node A"},
            },
        })},
    };
    route.slam_commands = nlohmann::json::array();
    return route;
}

go2w::SemanticRoute navigationRoute()
{
    go2w::SemanticRoute route;
    route.matched = true;
    route.task_queue = {
        {"queue_id", "queue_navigation_guard_test"},
        {"mode", "sequential"},
        {"status", "planned"},
        {"source", "scripted"},
        {"targets", nlohmann::json::array({"node_a"})},
        {"communication_policy", {
            {"mode", "normal"},
            {"send", nlohmann::json::array({"task_state"})},
            {"drop", nlohmann::json::array()},
        }},
        {"steps", nlohmann::json::array({
            {
                {"task_id", "navigate_1"},
                {"action", "navigate"},
                {"status", "pending"},
                {"target_node", "node_a"},
                {"target_name", "Node A"},
            },
        })},
    };
    route.slam_commands = nlohmann::json::array({
        {
            {"action", "navigate_to_pose"},
            {"target_node", "node_a"},
            {"target_pose", {{"x", 1.0}, {"y", 0.0}}},
        },
    });
    return route;
}

}  // namespace

int main()
{
    {
        go2w::QueueExecutorConfig config;
        config.execute_enabled = true;
        const auto result = go2w::QueueExecutor(config).execute(navigationRoute());
        require(result.exit_code == 6, "direct C++ navigation must remain disabled");
        require(!result.execution.value("executed", true), "blocked navigation must not be marked executed");
        require(
            result.execution.value("blocked_reason", "").find("persistent Python supervised executor") !=
                std::string::npos,
            "blocked navigation must name the supervised executor");
    }

    {
        go2w::QueueExecutorConfig config;
        config.execute_enabled = true;
        config.gateway_timeout_s = 5;
        config.capture_command =
            "printf '%s\\n' '{\"captured\":true,\"image_path\":\"/tmp/go2w_capture_test.jpg\",\"sidecar_path\":\"/tmp/go2w_capture_test.json\",\"run_id\":\"queue_capture_test\"}'";
        const auto result = go2w::QueueExecutor(config).execute(captureRoute());
        require(result.exit_code == 0, "capture command success should keep queue successful");
        require(result.execution.value("completed", false), "capture command success should complete queue");
        const auto& event = result.execution.at("events").at(0);
        require(event.value("status", "") == "ok", "capture event should be ok");
        require(event.at("result").value("captured", false), "capture result should be captured");
        require(event.at("result").value("image_path", "") == "/tmp/go2w_capture_test.jpg", "image path should be surfaced");
    }

    {
        go2w::QueueExecutorConfig config;
        config.execute_enabled = true;
        config.gateway_timeout_s = 5;
        config.capture_command = "printf '%s\\n' '{\"captured\":false,\"reason\":\"camera_missing\"}'; exit 1";
        const auto result = go2w::QueueExecutor(config).execute(captureRoute());
        require(result.exit_code == 5, "capture command failure should fail queue");
        require(!result.execution.value("completed", true), "capture command failure should not complete queue");
        require(result.execution.value("failed_step", "") == "capture_1", "failed capture step should be surfaced");
        require(result.execution.value("blocked_reason", "") == "camera_missing", "capture failure reason should be surfaced");
    }

    std::cout << "go2w_queue_executor_smoke_test=passed\n";
    return 0;
}
