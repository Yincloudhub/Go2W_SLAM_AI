#pragma once

#include <string>
#include <vector>

#include <nlohmann/json.hpp>

namespace go2w {

struct ProcessResult {
    int exit_code = 0;
    bool timed_out = false;
    std::string stdout_text;
    std::string stderr_text;
};

struct GatewayClientConfig {
    std::string client_path = "/home/unitree/Go2W_SLAM_AI/robot/slam_gateway_refactor/build/slam_llm_command_client";
    std::string network_interface = "eth0";
    int timeout_s = 30;
    double startup_wait_s = 0.0;
};

struct GatewayClientResult {
    nlohmann::json response;
    ProcessResult process;
};

class GatewayClient {
public:
    explicit GatewayClient(GatewayClientConfig config);

    GatewayClientResult send(const nlohmann::json& command) const;

private:
    GatewayClientConfig config_;
};

ProcessResult runProcessWithInput(
    const std::vector<std::string>& argv,
    const std::string& input,
    int timeout_s,
    double startup_wait_s = 0.0);

}  // namespace go2w
