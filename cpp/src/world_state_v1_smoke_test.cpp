#include "go2w/world_state_v1.hpp"

#include <iostream>
#include <stdexcept>

namespace {

void require(bool condition, const std::string& message)
{
    if (!condition) throw std::runtime_error(message);
}

}  // namespace

int main()
{
    const nlohmann::json gateway = {
        {"timestamp_ms", 123},
        {"world_state", {
            {"timestamp_ms", 123},
            {"current_pose", {{"pose", {{"x", 1.0}, {"y", 2.0}, {"yaw", 0.1}}}, {"map_id", "demo_map"}}},
            {"localization", {{"status", "localized"}, {"confidence", 0.9}}},
            {"slam_health", {{"status", "ok"}}},
            {"safety", {{"allow_navigation", true}, {"reason", "ok"}}},
            {"local_obstacle", {{"front_clearance_m", 2.0}, {"confidence", 0.8}, {"stale", false}}},
        }},
    };

    const auto world = go2w::buildWorldStateV1(
        gateway,
        {
            {"current_node", "wp_1"},
            {"motion_allowed", true},
            {"task_phase", "planning"},
            {"perception_summaries", nlohmann::json::array({
                {
                    {"source", "nx_ti_radar"},
                    {"confidence", 0.85},
                    {"stale", false},
                    {"latency_ms", 40},
                    {"timestamp_ms", 123},
                    {"summary", {{"nearest_track_range_m", 2.4}}},
                },
            })},
        });
    require(world.value("schema_version", 0) == 1, "schema_version mismatch");
    require(world.value("localized", false), "world should be localized");
    require(world.value("motion_allowed", false), "motion should be allowed");
    require(world.value("obstacle_status", std::string("")) == "clear", "obstacle status mismatch");
    require(world.contains("available_tools") && world.at("available_tools").is_array(), "missing available tools");
    require(world.at("perception_summaries").at(0).value("source", std::string("")) == "nx_ti_radar", "edge summary mismatch");

    const nlohmann::json task_queue = {
        {"targets", {"wp_1"}},
    };
    const nlohmann::json queue_execution = {
        {"completed", false},
        {"blocked_reason", ""},
        {"events", nlohmann::json::array({
            {
                {"operator_feedback", nlohmann::json::array({{{"text", "moving"}}})},
                {"llm_feedback_results", nlohmann::json::array({{{"text", "moving to target"}}})},
            },
        })},
    };
    const auto display = go2w::buildOperatorDisplayState(world, task_queue, queue_execution, "go");
    require(display.at("screen").value("current_target", std::string("")) == "wp_1", "target mismatch");
    require(display.at("screen").value("llm_reply", std::string("")) == "moving to target", "llm reply mismatch");

    const auto record = go2w::buildRuntimeLogRecord(world, display, task_queue, queue_execution, "go");
    require(!record.at("artifact_policy").value("allow_raw_video", true), "raw video should be disabled");
    std::cout << go2w::formatOperatorDisplayLine(display) << std::endl;
    return 0;
}
