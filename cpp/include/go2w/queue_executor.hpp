#pragma once

#include <ostream>
#include <string>

#include <nlohmann/json.hpp>

#include "go2w/gateway_client.hpp"
#include "go2w/safety_gate.hpp"
#include "go2w/semantic_router.hpp"

namespace go2w {

struct FeedbackPolicy {
    double slam_poll_interval_s = 1.0;
    double ui_refresh_interval_s = 1.0;
    double operator_feedback_interval_s = 5.0;
    double llm_feedback_interval_s = 8.0;
    int max_consecutive_gateway_errors = 3;
    bool runtime_safety_check = true;
};

struct QueueExecutorConfig {
    std::string gateway_client = "/home/unitree/slam_gateway_refactor/build/slam_llm_command_client";
    std::string network_interface = "eth0";
    int gateway_timeout_s = 30;
    double arrival_distance_m = 0.7;
    double arrival_monitor_s = 75.0;
    SafetyLimits safety_limits = {};
    FeedbackPolicy feedback_policy = {};
    bool execute_enabled = false;
};

struct QueueExecutionResult {
    int exit_code = 0;
    std::string stdout_text;
    nlohmann::json execution;
};

class QueueExecutor {
public:
    explicit QueueExecutor(QueueExecutorConfig config);

    QueueExecutionResult execute(const SemanticRoute& route) const;

private:
    nlohmann::json sendGatewayCommand(const nlohmann::json& command) const;
    nlohmann::json getWorldState() const;
    bool waitForArrival(
        const nlohmann::json& target_pose,
        const std::string& target_node,
        const std::string& target_name,
        const SafetyGate& safety_gate,
        nlohmann::json* event,
        std::ostream& log) const;

private:
    QueueExecutorConfig config_;
};

}  // namespace go2w
