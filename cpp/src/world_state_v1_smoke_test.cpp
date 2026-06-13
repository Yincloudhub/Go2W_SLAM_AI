#include "go2w/world_state_v1.hpp"

#include <fstream>
#include <iostream>
#include <stdexcept>

namespace {

void require(bool condition, const std::string& message)
{
    if (!condition) throw std::runtime_error(message);
}

bool arrayContainsString(const nlohmann::json& values, const std::string& expected)
{
    if (!values.is_array()) return false;
    for (const auto& value : values) {
        if (value.is_string() && value.get<std::string>() == expected) return true;
    }
    return false;
}

nlohmann::json perceptionContext()
{
    return {
        {"schema_version", 1},
        {"schema", "go2w_perception_context_v1"},
        {"context_id", "pc-cpp-test"},
        {"generated_at_ms", 123},
        {"stale_ms", 1000},
        {"robot_motion", {{"sources", nlohmann::json::array()}}},
        {"local_geometry", {
            {"primary", nullptr},
            {"forward_supplements", nlohmann::json::array()},
        }},
        {"visual_objects", nlohmann::json::array()},
        {"radar_tracks", nlohmann::json::array()},
        {"risk_events", nlohmann::json::array()},
        {"sources", nlohmann::json::array({
            {
                {"schema_version", 1},
                {"schema", "go2w_sensor_envelope_v1"},
                {"source_id", "ti_nx:radar_01"},
                {"source_kind", "radar_semantics"},
                {"timestamp_ms", nullptr},
                {"received_ms", 123},
                {"sequence", nullptr},
                {"age_ms", nullptr},
                {"stale_ms", 3000},
                {"frame_id", nullptr},
                {"status", "offline"},
                {"confidence", 0.0},
                {"calibration_status", "unknown"},
                {"calibration_id", nullptr},
                {"producer", "nx_edge_bridge"},
                {"producer_instance_id", nullptr},
                {"status_reasons", nlohmann::json::array({"producer_online_unverified"})},
                {"payload", nlohmann::json::object()},
            },
        })},
        {"degraded_capabilities", nlohmann::json::array({"ti_nx:radar_01"})},
        {"policy", {
            {"motion_authority", "slam_gateway"},
            {"llm_direct_motion", false},
            {"raw_sensor_streams_allowed", false},
            {"execution_chain", nlohmann::json::array({
                "task_queue",
                "mission_decision_engine",
                "slam_gateway",
                "unitree_sdk",
            })},
        }},
    };
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
            {"execution_enabled", true},
            {"task_phase", "planning"},
            {"perception_context", perceptionContext()},
            {"perception_context_current_time_ms", 123},
        });
    require(world.value("schema_version", 0) == 1, "schema_version mismatch");
    require(world.value("localized", false), "world should be localized");
    require(world.value("motion_allowed", false), "motion should be allowed");
    require(world.value("execution_enabled", false), "execution gate should be enabled");
    require(world.value("safety_allow_navigation", false), "safety should allow navigation");
    require(!world.value("chassis_motion_active", true), "idle navigation must not be shown as moving");
    require(world.contains("local_obstacle"), "full local obstacle summary should be retained");
    require(world.value("obstacle_status", std::string("")) == "clear", "obstacle status mismatch");
    require(world.contains("available_tools") && world.at("available_tools").is_array(), "missing available tools");
    require(arrayContainsString(world.at("available_tools"), "record_keyframe_event"), "unconfigured capture should be semantic event only");
    require(!arrayContainsString(world.at("available_tools"), "capture_keyframe"), "unconfigured capture should not advertise image capture");
    const auto world_with_capture = go2w::buildWorldStateV1(gateway, {{"capture_configured", true}});
    require(arrayContainsString(world_with_capture.at("available_tools"), "capture_keyframe"), "configured capture should be advertised");
    require(world_with_capture.at("capture_keyframe").value("configured", false), "configured capture state should be visible");
    require(world.at("perception_context").value("context_id", std::string("")) == "pc-cpp-test", "context mismatch");
    require(world.at("perception_summaries").at(0).value("source_id", std::string("")) == "ti_nx:radar_01", "source projection mismatch");
    const std::string context_path = "/tmp/go2w_perception_context_v1_smoke.json";
    {
        std::ofstream context_file(context_path);
        context_file << perceptionContext().dump();
    }
    require(
        go2w::loadPerceptionContextFile(context_path, 123).value("context_id", std::string("")) == "pc-cpp-test",
        "context file loader mismatch");
    require(go2w::loadPerceptionContextFile(context_path, 5000).is_null(), "stale context file must be rejected");

    auto stale_context = perceptionContext();
    stale_context["generated_at_ms"] = 1;
    stale_context["stale_ms"] = 100;
    const auto world_without_stale_context = go2w::buildWorldStateV1(
        gateway,
        {
            {"perception_context", stale_context},
            {"perception_context_current_time_ms", 123},
        });
    require(world_without_stale_context.at("perception_context").is_null(), "stale context must be rejected");
    require(world_without_stale_context.at("perception_summaries").empty(), "stale context sources must be hidden");

    auto malformed_context = perceptionContext();
    malformed_context["sources"][0]["schema"] = "wrong";
    const auto world_without_malformed_context = go2w::buildWorldStateV1(
        gateway,
        {
            {"perception_context", malformed_context},
            {"perception_context_current_time_ms", 123},
        });
    require(world_without_malformed_context.at("perception_context").is_null(), "malformed context must be rejected");

    auto blocked_gateway = gateway;
    blocked_gateway["world_state"]["safety"]["allow_navigation"] = false;
    blocked_gateway["world_state"]["safety"]["reason"] = "local_obstacle_not_fresh";
    const auto blocked_world = go2w::buildWorldStateV1(
        blocked_gateway,
        {{"execution_enabled", true}});
    require(blocked_world.value("execution_enabled", false), "execution gate should remain visible");
    require(!blocked_world.value("safety_allow_navigation", true), "safety rejection must remain visible");
    require(!blocked_world.value("motion_allowed", true), "execution gate must not override safety rejection");

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
