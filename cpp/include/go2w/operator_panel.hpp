#pragma once

#include <chrono>
#include <string>
#include <vector>

#include <nlohmann/json.hpp>

#include "go2w/gateway_client.hpp"
#include "go2w/semantic_router.hpp"

namespace go2w {

struct OperatorPanelConfig {
    std::string repo_root = "..";
    std::string gateway_client = "/home/unitree/slam_gateway_refactor/build/slam_llm_command_client";
    std::string network_interface = "eth0";
    std::string python = "python3";
    std::string start_slam_script = "scripts/start_go2w_slam_stack.sh";
    std::string start_rviz2_script = "scripts/start_go2w_rviz2.sh";
    std::string llm_http_url = "";
    std::string llm_http_model = "local";
    std::string current_node = "";
    double nav_speed_mps = 0.25;
    int nav_mode = 1;
    double arrival_distance_m = 0.7;
    double arrival_monitor_s = 75.0;
    double slam_poll_interval_s = 1.0;
    double ui_refresh_interval_s = 1.0;
    double operator_feedback_interval_s = 5.0;
    double llm_feedback_interval_s = 8.0;
    int max_consecutive_gateway_errors = 3;
    int max_arrival_samples = 120;
    int max_feedback_events = 120;
    int max_llm_feedback_events = 40;
    double gateway_startup_wait_s = 1.0;
    int gateway_timeout_s = 30;
    int llm_http_timeout_s = 20;
    int llm_http_max_tokens = 256;
    bool execute_enabled = false;
    bool weak_link_mode = false;
    bool ensure_slam_on_start = false;
};

struct CommandResult {
    int exit_code = 0;
    std::string stdout_text;
    std::string stderr_text;
};

class OperatorPanel {
public:
    explicit OperatorPanel(OperatorPanelConfig config);

    int runInteractive();
    void printStatusOnce() const;
    void watchWorld(int seconds) const;
    CommandResult submitUserCommand(const std::string& text) const;
    CommandResult ensureSlam() const;
    CommandResult startMapping(bool confirmed) const;
    CommandResult endMapping(const std::string& map_path, bool confirmed) const;
    CommandResult previewTopologyWaypoint(const std::string& name) const;
    CommandResult addTopologyWaypoint(const std::string& name, bool confirmed) const;
    CommandResult startRviz2(bool confirmed) const;

private:
    nlohmann::json sendGatewayCommand(const nlohmann::json& command) const;
    nlohmann::json getWorldState() const;
    CommandResult executeSemanticRoute(const SemanticRoute& route) const;
    CommandResult fallbackLlmHttpCommand(const std::string& text) const;
    CommandResult fallbackPythonCommand(const std::string& text) const;
    std::vector<nlohmann::json> buildLlmHttpMessages(const std::string& text) const;
    nlohmann::json buildPanelWorldState(const nlohmann::json& result) const;
    std::string formatWorldState(const nlohmann::json& result) const;
    std::string formatFullWorldState(const nlohmann::json& result) const;
    std::string formatWeakWorldState(const nlohmann::json& result) const;
    std::string nearestNodeText(const nlohmann::json& result) const;
    nlohmann::json loadRegistry() const;
    void printHelp() const;
    bool handleSlashCommand(const std::string& line);
    void setWeakMode(bool enabled);
    void setExecute(bool enabled);

private:
    OperatorPanelConfig config_;
};

std::string base64Encode(const std::string& bytes);
std::vector<nlohmann::json> extractJsonObjects(const std::string& text);
CommandResult runShellCommandWithInput(const std::string& command, const std::string& input);
std::string shellQuote(const std::string& value);

}  // namespace go2w
