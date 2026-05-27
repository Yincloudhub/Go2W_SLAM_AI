#pragma once

#include <string>

#include <nlohmann/json.hpp>

namespace go2w {

// Low-rate contract shared by the C++ operator panel, queue executor, and
// future LLM service. It deliberately keeps raw sensor streams out of the UI and
// LLM boundary.
nlohmann::json buildWorldStateV1(const nlohmann::json& runtime_or_gateway, const nlohmann::json& options = nlohmann::json::object());

nlohmann::json buildOperatorDisplayState(
    const nlohmann::json& world_state,
    const nlohmann::json& task_queue = nlohmann::json::object(),
    const nlohmann::json& queue_execution = nlohmann::json::object(),
    const std::string& user_command = "");

nlohmann::json buildRuntimeLogRecord(
    const nlohmann::json& world_state,
    const nlohmann::json& operator_display = nullptr,
    const nlohmann::json& task_queue = nullptr,
    const nlohmann::json& queue_execution = nullptr,
    const std::string& user_command = "",
    const nlohmann::json& llm_result = nullptr);

std::string formatOperatorDisplayLine(const nlohmann::json& operator_display, bool weak_link_mode = false);

}  // namespace go2w
