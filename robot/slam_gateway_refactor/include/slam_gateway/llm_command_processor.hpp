#pragma once

#include <string>
#include <json.hpp>

#include "slam_gateway/slam_gateway.hpp"

namespace slam_gateway {

class LlmCommandProcessor {
public:
    explicit LlmCommandProcessor(SlamGateway& gateway);

    // Accept one JSON command from LLM/task planner and return JSON result.
    // This is not a keyboard command path; it is the machine-readable input path.
    nlohmann::json process(const nlohmann::json& cmd);

private:
    nlohmann::json reject(const std::string& reason) const;
    nlohmann::json ok(const std::string& action, const ServiceResult& result) const;
    static PoseData parsePose(const nlohmann::json& j);

private:
    SlamGateway& gateway_;
};

}  // namespace slam_gateway
