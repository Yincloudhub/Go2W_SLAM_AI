#include "go2w/edge_perception.hpp"

#include <algorithm>
#include <chrono>
#include <fstream>

namespace go2w {
namespace {

constexpr std::streamoff kMaxSummaryBytes = 256 * 1024;

std::int64_t nowMs()
{
    return std::chrono::duration_cast<std::chrono::milliseconds>(
               std::chrono::system_clock::now().time_since_epoch())
        .count();
}

nlohmann::json boundedArray(const nlohmann::json& value, std::size_t max_items)
{
    if (!value.is_array()) return nlohmann::json::array();
    nlohmann::json out = nlohmann::json::array();
    const auto count = std::min(max_items, value.size());
    for (std::size_t i = 0; i < count; ++i) out.push_back(value.at(i));
    return out;
}

nlohmann::json unavailable(const std::string& path, const std::string& status, const std::string& error = "")
{
    nlohmann::json result = {
        {"available", false},
        {"fresh", false},
        {"eligible_for_llm", false},
        {"safety_candidate", false},
        {"safety_wired", false},
        {"status", status},
        {"path", path},
    };
    if (!error.empty()) result["error"] = error;
    return result;
}

}  // namespace

nlohmann::json loadEdgePerceptionSummary(
    const std::string& path,
    std::int64_t max_age_ms,
    std::int64_t current_time_ms)
{
    std::ifstream input(path, std::ios::binary | std::ios::ate);
    if (!input) return unavailable(path, "offline_or_not_started");
    const auto summary_size = input.tellg();
    if (summary_size < 0 || summary_size > kMaxSummaryBytes) {
        return unavailable(path, "invalid_summary", "summary exceeds 256 KiB limit");
    }
    input.seekg(0);

    try {
        nlohmann::json data;
        input >> data;
        if (!data.is_object()) return unavailable(path, "invalid_summary", "root must be an object");
        if (data.value("schema_version", 0) != 1) return unavailable(path, "invalid_summary", "schema_version must be 1");

        const std::string node_id = data.value("node_id", "");
        const std::string sensor_type = data.value("sensor_type", "");
        const std::string source = data.value("source", "");
        const std::int64_t timestamp_ms = data.value("timestamp_ms", std::int64_t{0});
        if (node_id.empty() || sensor_type.empty() || source.empty() || timestamp_ms <= 0) {
            return unavailable(path, "invalid_summary", "node_id, sensor_type, source and timestamp_ms are required");
        }

        const std::int64_t age_ms = (current_time_ms > 0 ? current_time_ms : nowMs()) - timestamp_ms;
        const auto health = data.value("health", nlohmann::json::object());
        const auto policy = data.value("policy", nlohmann::json::object());
        const std::string health_status = health.value("status", "unknown");
        const std::string policy_mode = policy.value("mode", "semantic_only");
        const bool calibrated = policy.value("calibrated", false);
        const bool declared_safety_candidate = policy.value("safety_candidate", false);
        const bool fresh =
            !data.value("stale", false) &&
            age_ms >= 0 &&
            age_ms <= std::max<std::int64_t>(1, max_age_ms) &&
            health_status == "ok";

        return {
            {"available", true},
            {"fresh", fresh},
            {"eligible_for_llm", fresh},
            {"safety_candidate", fresh && calibrated && declared_safety_candidate},
            {"safety_wired", false},
            {"status", fresh ? "fresh" : "stale_or_unhealthy"},
            {"path", path},
            {"age_ms", age_ms},
            {"stale_ms", std::max<std::int64_t>(1, max_age_ms)},
            {"data", {
                {"schema_version", 1},
                {"node_id", node_id},
                {"sensor_type", sensor_type},
                {"source", source},
                {"timestamp_ms", timestamp_ms},
                {"confidence", data.value("confidence", 0.0)},
                {"latency_ms", data.value("latency_ms", 0.0)},
                {"health", health},
                {"policy", {
                    {"mode", policy_mode},
                    {"calibrated", calibrated},
                    {"safety_candidate", declared_safety_candidate},
                }},
                {"observations", boundedArray(data.value("observations", nlohmann::json::array()), 32)},
                {"events", boundedArray(data.value("events", nlohmann::json::array()), 32)},
                {"summary", data.value("summary", nlohmann::json::object())},
            }},
        };
    } catch (const std::exception& exc) {
        return unavailable(path, "invalid_summary", exc.what());
    }
}

}  // namespace go2w
