#include "go2w/operator_panel.hpp"
#include "go2w/edge_perception.hpp"
#include "go2w/llm_http_client.hpp"
#include "go2w/queue_executor.hpp"
#include "go2w/world_state_v1.hpp"

#include <array>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <thread>
#include <vector>

#include <fcntl.h>
#include <sys/wait.h>
#include <unistd.h>

namespace go2w {
namespace {

std::string readAll(std::istream& input)
{
    std::ostringstream ss;
    ss << input.rdbuf();
    return ss.str();
}

bool isAbsolutePath(const std::string& path)
{
    return !path.empty() && path.front() == '/';
}

std::string fmtDouble(const nlohmann::json& value, int digits = 2)
{
    if (value.is_null()) return "未知";
    try {
        std::ostringstream ss;
        ss << std::fixed << std::setprecision(digits) << value.get<double>();
        return ss.str();
    } catch (...) {
        return "未知";
    }
}

std::string fmtMeters(const nlohmann::json& value)
{
    if (value.is_number()) return fmtDouble(value) + "m";
    return "未知";
}

const nlohmann::json* objectAt(const nlohmann::json& root, const std::initializer_list<const char*> keys)
{
    const nlohmann::json* current = &root;
    for (const char* key : keys) {
        if (!current->is_object() || !current->contains(key)) return nullptr;
        current = &current->at(key);
    }
    return current;
}

std::string jsonString(const nlohmann::json* value, const std::string& fallback = "未知")
{
    if (!value || value->is_null()) return fallback;
    if (value->is_string()) return value->get<std::string>();
    if (value->is_boolean()) return value->get<bool>() ? "true" : "false";
    if (value->is_number()) return value->dump();
    return fallback;
}

bool jsonBool(const nlohmann::json* value, bool fallback = false)
{
    if (!value || !value->is_boolean()) return fallback;
    return value->get<bool>();
}

double jsonNumber(const nlohmann::json* value, double fallback = 0.0)
{
    if (!value || !value->is_number()) return fallback;
    return value->get<double>();
}

bool isTopologyPoseFresh(const nlohmann::json& gateway_state, double max_pose_age_ms = 2000.0)
{
    const auto* world_ptr = objectAt(gateway_state, {"world_state"});
    const nlohmann::json& world = world_ptr && world_ptr->is_object() ? *world_ptr : gateway_state;
    const std::string status = jsonString(objectAt(world, {"localization", "status"}), "");
    const double pose_age_ms = jsonNumber(objectAt(world, {"localization", "pose_age_ms"}), -1.0);
    const auto* pose = objectAt(world, {"current_pose", "pose"});
    const bool status_ok =
        status == "localized" || status == "degraded" || status == "localized_or_tracking" || status == "tracking";
    return status_ok && pose_age_ms >= 0.0 && pose_age_ms <= max_pose_age_ms && pose && pose->is_object();
}

std::string nowTime()
{
    const std::time_t now = std::time(nullptr);
    std::tm tm {};
    localtime_r(&now, &tm);
    char buf[16] {};
    std::strftime(buf, sizeof(buf), "%H:%M:%S", &tm);
    return buf;
}

std::string trimAscii(const std::string& value)
{
    const auto first = value.find_first_not_of(" \t\r\n");
    if (first == std::string::npos) return "";
    const auto last = value.find_last_not_of(" \t\r\n");
    return value.substr(first, last - first + 1);
}

std::string lowerAscii(std::string value)
{
    for (char& ch : value) {
        if (ch >= 'A' && ch <= 'Z') ch = static_cast<char>(ch - 'A' + 'a');
    }
    return value;
}

bool containsAny(const std::string& text, const std::vector<std::string>& terms)
{
    for (const auto& term : terms) {
        if (!term.empty() && text.find(term) != std::string::npos) return true;
    }
    return false;
}

bool isAsciiDigit(char ch)
{
    return ch >= '0' && ch <= '9';
}

bool containsMetricDistanceAscii(const std::string& text)
{
    for (std::size_t i = 0; i < text.size(); ++i) {
        if (!isAsciiDigit(text[i])) continue;
        std::size_t j = i + 1;
        while (j < text.size() && (isAsciiDigit(text[j]) || text[j] == '.')) ++j;
        while (j < text.size() && (text[j] == ' ' || text[j] == '\t')) ++j;
        if (j < text.size() && text[j] == 'm') return true;
    }
    return false;
}

std::string requestedNotWiredCapability(const std::string& text)
{
    const std::string lower = lowerAscii(text);
    if (containsAny(lower, {"cmd_vel", "slam_operate", "raw_base_control"})) {
        return "raw_base_control";
    }
    if (containsAny(lower, {"mapless_scout", "without slam", "no slam", "odom only", "odom-only"}) ||
        containsAny(text, {"不开SLAM", "不用SLAM", "不启用SLAM", "未知区域", "探索未知"})) {
        return "mapless_scout";
    }
    const bool motion_word =
        containsAny(lower, {"forward", "ahead", "straight"}) ||
        containsAny(text, {"前进", "向前", "往前", "直走"});
    const bool distance_word =
        containsAny(lower, {" meter", " meters", " metre", " metres", "m "}) ||
        containsMetricDistanceAscii(lower) ||
        containsAny(text, {"米"});
    if (motion_word && distance_word) return "relative_motion";
    return "";
}

CommandResult unsupportedCapabilityResult(const std::string& capability)
{
    CommandResult result;
    result.exit_code = 4;
    std::ostringstream out;
    out << "capability_guard: " << capability
        << " is not wired for real execution; no command sent. "
        << "Use a registered topology target or dry-run a future capability contract.\n";
    result.stdout_text = out.str();
    return result;
}

std::string joinStrings(const std::vector<std::string>& values, const std::string& sep)
{
    std::ostringstream ss;
    for (std::size_t i = 0; i < values.size(); ++i) {
        if (i) ss << sep;
        ss << values[i];
    }
    return ss.str();
}

const nlohmann::json* findRegistryMap(const nlohmann::json& registry, const std::string& map_id)
{
    const auto* maps = objectAt(registry, {"maps"});
    if (!maps || !maps->is_array()) return nullptr;
    for (const auto& map : *maps) {
        if (map.is_object() && map.value("map_id", "") == map_id) return &map;
    }
    return nullptr;
}

const nlohmann::json* findRelocalizationAnchor(const nlohmann::json& map, const std::string& anchor_id)
{
    const auto* anchors = objectAt(map, {"relocalization_anchors"});
    if (!anchors || !anchors->is_array()) return nullptr;
    for (const auto& anchor : *anchors) {
        if (!anchor.is_object()) continue;
        if (anchor.value("anchor_id", "") == anchor_id || anchor.value("name", "") == anchor_id) return &anchor;
    }
    return nullptr;
}

double yawFromQuaternionLocal(double qx, double qy, double qz, double qw)
{
    const double siny_cosp = 2.0 * (qw * qz + qx * qy);
    const double cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz);
    return std::atan2(siny_cosp, cosy_cosp);
}

nlohmann::json anchorRelocatePose(const nlohmann::json& anchor)
{
    const std::string anchor_id = anchor.value("anchor_id", "anchor");
    const auto* pose_ptr = objectAt(anchor, {"pose"});
    nlohmann::json pose = pose_ptr && pose_ptr->is_object() ? *pose_ptr : nlohmann::json::object();
    const double qx = jsonNumber(objectAt(pose, {"q_x"}), 0.0);
    const double qy = jsonNumber(objectAt(pose, {"q_y"}), 0.0);
    const double qz = jsonNumber(objectAt(pose, {"q_z"}), 0.0);
    const double qw = jsonNumber(objectAt(pose, {"q_w"}), 1.0);
    pose["name"] = anchor_id;
    pose["q_x"] = qx;
    pose["q_y"] = qy;
    pose["q_z"] = qz;
    pose["q_w"] = qw;
    if (!pose.contains("yaw") || !pose.at("yaw").is_number()) {
        pose["yaw"] = yawFromQuaternionLocal(qx, qy, qz, qw);
    }
    pose["speed"] = 0.0;
    pose["mode"] = 0;
    return pose;
}

std::string relocationSummary(const nlohmann::json& response)
{
    if (response.value("accepted", false)) return "relocation accepted by gateway";

    const auto* data = objectAt(response, {"data"});
    nlohmann::json parsed;
    const nlohmann::json* payload = &response;
    if (data && data->is_string()) {
        try {
            parsed = nlohmann::json::parse(data->get<std::string>());
            payload = &parsed;
        } catch (...) {
        }
    } else if (data && data->is_object()) {
        payload = data;
    }

    std::ostringstream out;
    out << "relocation rejected";
    if (payload->contains("statusCode")) out << " statusCode=" << payload->at("statusCode");
    if (payload->contains("errorCode")) out << " errorCode=" << payload->at("errorCode");
    if (payload->contains("info") && payload->at("info").is_string()) out << " info=" << payload->at("info").get<std::string>();
    if (payload->contains("message") && payload->at("message").is_string()) out << " message=" << payload->at("message").get<std::string>();
    return out.str();
}

std::vector<std::string> unverifiedRouteTargets(const SemanticRoute& route)
{
    std::vector<std::string> targets;
    for (const auto& target : route.targets) {
        if (!target.needs_calibration) continue;
        std::string label = target.name + "(" + target.node_id + ")";
        label += target.requires_standing_verification ? ":needs_standing_verification" : ":needs_calibration";
        targets.push_back(label);
    }
    return targets;
}

nlohmann::json loadJsonFileOrNull(const std::string& path)
{
    std::ifstream file(path);
    if (!file) return nullptr;
    try {
        return nlohmann::json::parse(readAll(file));
    } catch (...) {
        return nullptr;
    }
}

nlohmann::json loadFreshPerceptionForLlm(const std::string& path, int64_t max_age_ms)
{
    nlohmann::json payload = loadJsonFileOrNull(path);
    if (!payload.is_object()) return {{"available", false}, {"excluded_reason", "missing_or_invalid"}};
    const int64_t timestamp_ms = payload.value("timestamp_ms", int64_t{0});
    const int64_t now_ms = static_cast<int64_t>(std::time(nullptr)) * 1000;
    const int64_t age_ms = timestamp_ms > 0 ? now_ms - timestamp_ms : -1;
    if (payload.value("stale", false) || timestamp_ms <= 0 || age_ms < 0 || age_ms > max_age_ms) {
        return {
            {"available", false},
            {"excluded_reason", "stale_or_offline"},
            {"source", payload.value("source", "")},
            {"age_ms", age_ms},
        };
    }
    payload["available"] = true;
    payload["age_ms"] = age_ms;
    return payload;
}

}  // namespace

std::string shellQuote(const std::string& value)
{
    std::string out = "'";
    for (char ch : value) {
        if (ch == '\'') {
            out += "'\"'\"'";
        } else {
            out += ch;
        }
    }
    out += "'";
    return out;
}

std::string base64Encode(const std::string& bytes)
{
    static constexpr char kTable[] = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
    std::string out;
    int val = 0;
    int valb = -6;
    for (unsigned char c : bytes) {
        val = (val << 8) + c;
        valb += 8;
        while (valb >= 0) {
            out.push_back(kTable[(val >> valb) & 0x3F]);
            valb -= 6;
        }
    }
    if (valb > -6) out.push_back(kTable[((val << 8) >> (valb + 8)) & 0x3F]);
    while (out.size() % 4) out.push_back('=');
    return out;
}

std::vector<nlohmann::json> extractJsonObjects(const std::string& text)
{
    std::vector<nlohmann::json> objects;
    for (std::size_t start = 0; start < text.size(); ++start) {
        if (text[start] != '{') continue;
        bool in_string = false;
        bool escaped = false;
        int depth = 0;
        for (std::size_t i = start; i < text.size(); ++i) {
            const char ch = text[i];
            if (in_string) {
                if (escaped) {
                    escaped = false;
                } else if (ch == '\\') {
                    escaped = true;
                } else if (ch == '"') {
                    in_string = false;
                }
                continue;
            }
            if (ch == '"') {
                in_string = true;
            } else if (ch == '{') {
                ++depth;
            } else if (ch == '}') {
                --depth;
                if (depth == 0) {
                    try {
                        objects.push_back(nlohmann::json::parse(text.substr(start, i - start + 1)));
                    } catch (...) {
                    }
                    start = i;
                    break;
                }
            }
        }
    }
    return objects;
}

CommandResult runShellCommandWithInput(const std::string& command, const std::string& input)
{
    char path[] = "/tmp/go2w_panel_input_XXXXXX";
    const int fd = mkstemp(path);
    if (fd < 0) throw std::runtime_error("failed to create temp input file");
    const std::string payload = input;
    const char* data = payload.data();
    std::size_t left = payload.size();
    while (left > 0) {
        const ssize_t written = write(fd, data, left);
        if (written <= 0) {
            close(fd);
            unlink(path);
            throw std::runtime_error("failed to write temp input file");
        }
        data += written;
        left -= static_cast<std::size_t>(written);
    }
    close(fd);

    const std::string output_path = std::string(path) + ".out";
    const std::string error_path = std::string(path) + ".err";
    const std::string wrapped = command + " < " + shellQuote(path) + " > " + shellQuote(output_path) + " 2> " + shellQuote(error_path);
    const int rc = std::system(wrapped.c_str());

    std::ifstream out_file(output_path);
    std::ifstream err_file(error_path);
    CommandResult result;
    result.exit_code = WIFEXITED(rc) ? WEXITSTATUS(rc) : rc;
    result.stdout_text = readAll(out_file);
    result.stderr_text = readAll(err_file);
    unlink(path);
    unlink(output_path.c_str());
    unlink(error_path.c_str());
    return result;
}

OperatorPanel::OperatorPanel(OperatorPanelConfig config)
    : config_(std::move(config))
{
}

nlohmann::json OperatorPanel::loadRegistry() const
{
    const std::string path = registryPath();
    std::ifstream file(path);
    if (!file) return nlohmann::json::object();
    return nlohmann::json::parse(readAll(file));
}

std::string OperatorPanel::registryPath() const
{
    if (isAbsolutePath(config_.registry_path)) return config_.registry_path;
    return config_.repo_root + "/" + config_.registry_path;
}

nlohmann::json OperatorPanel::getWorldState() const
{
    return sendGatewayCommand({{"action", "get_world_state"}});
}

nlohmann::json OperatorPanel::sendGatewayCommand(const nlohmann::json& command_json) const
{
    GatewayClient client({
        config_.gateway_client,
        config_.network_interface,
        config_.gateway_timeout_s,
        config_.gateway_startup_wait_s,
    });
    return client.send(command_json).response;
}

nlohmann::json OperatorPanel::buildPanelWorldState(const nlohmann::json& result) const
{
    return buildWorldStateV1(
        result,
        {
            {"current_node", nearestNodeText(result)},
            {"motion_allowed", config_.execute_enabled},
            {"network_level", config_.weak_link_mode ? "weak" : "normal"},
            {"task_phase", "idle"},
        });
}

std::string OperatorPanel::nearestNodeText(const nlohmann::json& result) const
{
    const auto* pose = objectAt(result, {"world_state", "current_pose", "pose"});
    if (!pose || !pose->is_object()) return "未知";
    const double x = jsonNumber(objectAt(*pose, {"x"}));
    const double y = jsonNumber(objectAt(*pose, {"y"}));
    const nlohmann::json registry = loadRegistry();
    const auto* maps = objectAt(registry, {"maps"});
    if (!maps || !maps->is_array()) return "未知";

    double best = 0.0;
    std::string label;
    bool found = false;
    for (const auto& map : *maps) {
        const auto* nodes = objectAt(map, {"topology_nodes"});
        if (!nodes || !nodes->is_array()) continue;
        for (const auto& node : *nodes) {
            const auto* node_pose = objectAt(node, {"pose"});
            if (!node_pose || !node_pose->is_object()) continue;
            const double nx = jsonNumber(objectAt(*node_pose, {"x"}));
            const double ny = jsonNumber(objectAt(*node_pose, {"y"}));
            const double distance = std::hypot(x - nx, y - ny);
            if (!found || distance < best) {
                found = true;
                best = distance;
                label = node.value("name", node.value("node_id", "unknown"));
            }
        }
    }
    if (!found) return "未知";
    nlohmann::json distance = best;
    return label + "(" + fmtMeters(distance) + ")";
}

std::string OperatorPanel::formatFullWorldState(const nlohmann::json& result) const
{
    const nlohmann::json world = buildPanelWorldState(result);
    const nlohmann::json display = buildOperatorDisplayState(world);
    const auto* pose = objectAt(world, {"current_pose"});

    std::ostringstream ss;
    ss << formatOperatorDisplayLine(display, false);
    if (pose && pose->is_object()) {
        ss << " | pose:x=" << fmtDouble(pose->value("x", nlohmann::json(nullptr)))
           << ", y=" << fmtDouble(pose->value("y", nlohmann::json(nullptr)))
           << ", yaw=" << fmtDouble(pose->value("yaw", nlohmann::json(nullptr)));
    }
    ss << " | map_id=" << jsonString(objectAt(world, {"map_id"}), "unknown")
       << " | health=" << jsonString(objectAt(world, {"source_health", "slam_status"}), "unknown")
       << "/" << jsonString(objectAt(world, {"source_health", "localization_status"}), "unknown");
    return ss.str();
}

std::string OperatorPanel::formatWeakWorldState(const nlohmann::json& result) const
{
    const nlohmann::json world = buildPanelWorldState(result);
    return formatOperatorDisplayLine(buildOperatorDisplayState(world), true);
}

std::string OperatorPanel::formatWorldState(const nlohmann::json& result) const
{
    try {
        return config_.weak_link_mode ? formatWeakWorldState(result) : formatFullWorldState(result);
    } catch (const std::exception& exc) {
        return std::string("世界状态格式化失败: ") + exc.what();
    }
}

void OperatorPanel::printStatusOnce() const
{
    try {
        std::cout << "[" << nowTime() << "] " << formatWorldState(getWorldState()) << "\n";
    } catch (const std::exception& exc) {
        std::cout << "[" << nowTime() << "] 世界状态读取失败: " << exc.what() << "\n";
    }
}

void OperatorPanel::watchWorld(int seconds) const
{
    const auto start = std::chrono::steady_clock::now();
    while (seconds <= 0 || std::chrono::duration_cast<std::chrono::seconds>(std::chrono::steady_clock::now() - start).count() < seconds) {
        printStatusOnce();
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }
}

CommandResult OperatorPanel::submitUserCommand(const std::string& text) const
{
    const std::string not_wired_capability = requestedNotWiredCapability(text);
    if (!not_wired_capability.empty()) {
        return unsupportedCapabilityResult(not_wired_capability);
    }

    const SemanticRouter router(loadRegistry(), config_.map_id);
    const SemanticRoute route = router.planText(text, config_.nav_speed_mps, config_.nav_mode);
    if (route.matched) {
        return executeSemanticRoute(route);
    }
    if (!config_.llm_http_url.empty()) {
        return fallbackLlmHttpCommand(text);
    }
    return fallbackPythonCommand(text);
}

std::vector<nlohmann::json> OperatorPanel::buildLlmHttpMessages(const std::string& text) const
{
    nlohmann::json candidates = nlohmann::json::array();
    const nlohmann::json registry = loadRegistry();
    const auto* maps = objectAt(registry, {"maps"});
    if (maps && maps->is_array()) {
        for (const auto& map : *maps) {
            if (!map.is_object() || map.value("map_id", "") != config_.map_id) continue;
            const auto* nodes = objectAt(map, {"topology_nodes"});
            if (!nodes || !nodes->is_array()) continue;
            for (const auto& node : *nodes) {
                if (!node.is_object()) continue;
                bool disabled = false;
                const auto tags = node.value("tags", nlohmann::json::array());
                if (tags.is_array()) {
                    for (const auto& tag : tags) {
                        if (!tag.is_string()) continue;
                        const std::string value = tag.get<std::string>();
                        if (value == "disabled" || value == "ui_disabled" || value == "deleted") disabled = true;
                    }
                }
                if (disabled) continue;
                candidates.push_back({
                    {"node_id", node.value("node_id", "")},
                    {"name", node.value("name", "")},
                    {"aliases", node.value("aliases", nlohmann::json::array())},
                    {"tags", node.value("tags", nlohmann::json::array())},
                });
            }
        }
    }

    const std::string system_prompt =
        "You are the GO2W robot-dog operator planner. Select only registered topology node_id values. "
        "Return only one compact JSON object with this schema: "
        "{\"reply\":\"short Chinese operator reply\",\"targets\":[\"node_id\"],\"capture_keyframe\":false}. "
        "Preserve the user's target order for multi-stop tasks. Set capture_keyframe=true only when the user asks for a photo or inspection image. "
        "Do not output coordinates, speeds, Unitree API ids, markdown, or extra text. Never claim that the robot arrived before runtime feedback says so. "
        "Use capability_contract: ready or available conditional capabilities may be planned; not_wired capabilities such as relative_motion, mapless_scout, or raw_base_control must return empty targets with one short clarification reply. "
        "Perception entries with available=false are diagnostic only and must not affect the plan. "
        "If the command is unclear or a requested place is not registered, return targets as an empty array and ask one short clarification question in reply.";
    nlohmann::json perception = {
        {"stereo_depth", loadFreshPerceptionForLlm(config_.repo_root + "/artifacts/stereo_depth_summary.json", 1000)},
        {"deepyolo_semantics", loadFreshPerceptionForLlm(config_.repo_root + "/artifacts/vision_semantic_summary.json", 3000)},
        {"edge_node", loadEdgePerceptionSummary(config_.repo_root + "/artifacts/edge_perception_summary.json", 3000)},
    };
    nlohmann::json capability_contract = {
        {"planning_style", "capability_bounded_topology_selection"},
        {"ready", nlohmann::json::array({"hold_position", "request_clarification", "registered_topology_dry_run"})},
        {"conditional", nlohmann::json::array({
            {
                {"name", "mapped_topology_navigation"},
                {"available", true},
                {"requires", nlohmann::json::array({"registered_topology_node", "SafetyGate_allow", "SLAM_Gateway_accept"})},
                {"fallback", "empty_targets_with_clarification"},
            },
            {
                {"name", "capture_keyframe"},
                {"available", true},
                {"status", "semantic_event_until_camera_command_configured"},
                {"fallback", "record_semantic_keyframe_event"},
            },
        })},
        {"not_wired", nlohmann::json::array({
            {
                {"name", "relative_motion"},
                {"examples", nlohmann::json::array({"forward_10m_photo", "odom_only_drive"})},
                {"fallback", "empty_targets_with_clarification"},
            },
            {
                {"name", "mapless_scout"},
                {"fallback", "empty_targets_with_clarification"},
            },
            {
                {"name", "raw_base_control"},
                {"fallback", "reject"},
            },
        })},
    };

    nlohmann::json user_payload = {
        {"command", text},
        {"current_node", config_.current_node},
        {"execute_enabled", config_.execute_enabled},
        {"candidates", candidates},
        {"capability_contract", capability_contract},
        {"perception", perception},
    };
    return {
        {{"role", "system"}, {"content", system_prompt}},
        {{"role", "user"}, {"content", user_payload.dump()}},
    };
}

CommandResult OperatorPanel::fallbackLlmHttpCommand(const std::string& text) const
{
    CommandResult result;
    std::ostringstream out;
    const LlmHttpClient client({
        config_.llm_http_url,
        config_.llm_http_model,
        config_.llm_http_timeout_s,
        config_.llm_http_max_tokens,
        0.0,
    });
    const LlmHttpResult llm = client.chat(buildLlmHttpMessages(text));
    if (!llm.ok) {
        result.exit_code = 5;
        result.stderr_text = "C++ LLM HTTP failed: " + llm.error + "\n";
        return result;
    }

    out << "C++ LLM HTTP reply: " << llm.content << "\n";
    const auto objects = extractJsonObjects(llm.content);
    if (objects.empty()) {
        result.exit_code = 4;
        out << "C++ LLM HTTP did not return executable JSON; no command sent.\n";
        result.stdout_text = out.str();
        return result;
    }

    const nlohmann::json& plan = objects.front();
    if (plan.contains("reply") && plan.at("reply").is_string()) {
        out << "LLM operator reply: " << plan.at("reply").get<std::string>() << "\n";
    }

    std::vector<std::string> targets;
    if (plan.contains("targets") && plan.at("targets").is_array()) {
        for (const auto& item : plan.at("targets")) {
            if (item.is_string() && !item.get<std::string>().empty()) targets.push_back(item.get<std::string>());
        }
    }
    if (targets.empty()) {
        result.exit_code = 4;
        out << "C++ LLM HTTP returned no registered target; no command sent.\n";
        result.stdout_text = out.str();
        return result;
    }

    std::string route_text = joinStrings(targets, " ");
    if (plan.value("capture_keyframe", false)) route_text += " capture";
    const SemanticRouter router(loadRegistry(), config_.map_id);
    const SemanticRoute route = router.planText(route_text, config_.nav_speed_mps, config_.nav_mode);
    if (!route.matched) {
        result.exit_code = 4;
        out << "C++ LLM HTTP targets did not match registry after validation; no command sent.\n";
        result.stdout_text = out.str();
        return result;
    }

    CommandResult execution = executeSemanticRoute(route);
    result.exit_code = execution.exit_code;
    result.stdout_text = out.str() + execution.stdout_text;
    result.stderr_text = execution.stderr_text;
    return result;
}

CommandResult OperatorPanel::fallbackPythonCommand(const std::string& text) const
{
    const std::string encoded = base64Encode(text);
    std::ostringstream cmd;
    cmd << "cd " << shellQuote(config_.repo_root)
        << " && PYTHONPATH=src " << shellQuote(config_.python)
        << " scripts/go2w_agent_entry.py --go-b64 " << shellQuote(encoded)
        << " --registry " << shellQuote(registryPath())
        << " --map-id " << shellQuote(config_.map_id)
        << " --human --nav-speed-mps " << config_.nav_speed_mps
        << " --nav-mode " << config_.nav_mode
        << " --arrival-distance-m " << config_.arrival_distance_m
        << " --arrival-monitor-s " << config_.arrival_monitor_s
        << " --gateway-startup-wait-s " << config_.gateway_startup_wait_s;
    if (config_.execute_enabled) cmd << " --execute";
    if (!config_.execute_enabled) cmd << " --dry-run";
    if (!config_.current_node.empty()) cmd << " --current-node " << shellQuote(config_.current_node);
    return runShellCommandWithInput(cmd.str(), "");
}

CommandResult OperatorPanel::ensureSlam() const
{
    std::ostringstream cmd;
    cmd << "cd " << shellQuote(config_.repo_root)
        << " && bash " << shellQuote(config_.start_slam_script);
    return runShellCommandWithInput(cmd.str(), "");
}

CommandResult OperatorPanel::startMapping(bool confirmed) const
{
    CommandResult result;
    if (!confirmed) {
        result.exit_code = 2;
        result.stdout_text = "start_mapping requires explicit confirmation: /mapping start confirm\n";
        return result;
    }

    const auto response = sendGatewayCommand({
        {"action", "start_mapping"},
        {"slam_type", "indoor"},
        {"operator_ack", true},
    });
    result.exit_code = response.value("accepted", false) ? 0 : 3;
    result.stdout_text = response.dump(2) + "\n";
    return result;
}

CommandResult OperatorPanel::endMapping(const std::string& map_path, bool confirmed) const
{
    CommandResult result;
    if (!confirmed) {
        result.exit_code = 2;
        result.stdout_text = "end_mapping requires explicit confirmation: /mapping end confirm [map_path]\n";
        return result;
    }

    const std::string target_path = map_path.empty() ? "/home/unitree/test.pcd" : map_path;
    const auto response = sendGatewayCommand({
        {"action", "end_mapping"},
        {"map_path", target_path},
        {"operator_ack", true},
    });
    result.exit_code = response.value("accepted", false) ? 0 : 3;
    result.stdout_text = response.dump(2) + "\n";
    return result;
}

CommandResult OperatorPanel::relocateAnchor(const std::string& anchor_id, bool confirmed) const
{
    CommandResult result;
    const std::string target_anchor = trimAscii(anchor_id);
    if (target_anchor.empty()) {
        result.exit_code = 2;
        result.stdout_text = "relocate requires an anchor id: /relocate ANCHOR_ID confirm\n";
        return result;
    }
    if (!confirmed) {
        result.exit_code = 2;
        result.stdout_text = "relocate requires explicit confirmation: /relocate ANCHOR_ID confirm\n";
        return result;
    }

    const nlohmann::json registry = loadRegistry();
    const auto* map = findRegistryMap(registry, config_.map_id);
    if (!map) {
        result.exit_code = 2;
        result.stderr_text = "map not found in registry: " + config_.map_id + "\n";
        return result;
    }
    const auto* anchor = findRelocalizationAnchor(*map, target_anchor);
    if (!anchor) {
        result.exit_code = 2;
        result.stderr_text = "anchor not found in registry: " + target_anchor + "\n";
        return result;
    }
    const auto* pose = objectAt(*anchor, {"pose"});
    if (!pose || !pose->is_object() || !pose->contains("x") || !pose->contains("y")) {
        result.exit_code = 2;
        result.stderr_text = "anchor pose is incomplete: " + target_anchor + "\n";
        return result;
    }

    const auto command = nlohmann::json{
        {"action", "relocate"},
        {"map_id", config_.map_id},
        {"map_path", map->value("pcd_path", "/home/unitree/test.pcd")},
        {"anchor_id", anchor->value("anchor_id", target_anchor)},
        {"initial_pose", anchorRelocatePose(*anchor)},
    };
    const auto response = sendGatewayCommand(command);
    result.exit_code = response.value("accepted", false) ? 0 : 3;
    result.stdout_text = relocationSummary(response) + "\n" + response.dump(2) + "\n";
    return result;
}

CommandResult OperatorPanel::previewTopologyWaypoint(const std::string& name) const
{
    CommandResult result;
    const std::string waypoint_name = trimAscii(name);
    if (waypoint_name.empty()) {
        result.exit_code = 2;
        result.stdout_text = "topology preview requires a waypoint name: /topology preview NAME\n";
        return result;
    }

    const auto gateway_state = getWorldState();
    const auto* gateway_world_ptr = objectAt(gateway_state, {"world_state"});
    const nlohmann::json& gateway_world =
        gateway_world_ptr && gateway_world_ptr->is_object() ? *gateway_world_ptr : gateway_state;
    const auto world = buildPanelWorldState(gateway_state);
    const auto display = buildOperatorDisplayState(world);
    const auto* pose = objectAt(world, {"current_pose"});

    std::ostringstream out;
    out << "topology_preview_name=" << waypoint_name << "\n";
    out << formatOperatorDisplayLine(display, false) << "\n";
    out << "localization_status=" << jsonString(objectAt(gateway_world, {"localization", "status"}), "unknown")
        << " pose_age_ms=" << fmtDouble(jsonNumber(objectAt(gateway_world, {"localization", "pose_age_ms"}), -1.0), 0)
        << " slam_status=" << jsonString(objectAt(gateway_world, {"slam_health", "status"}), "unknown") << "\n";
    if (pose && pose->is_object()) {
        out << "candidate_pose=" << pose->dump(2) << "\n";
    } else {
        out << "candidate_pose=null\n";
    }

    if (!isTopologyPoseFresh(gateway_state)) {
        result.exit_code = 2;
        out << "blocked: localization is not fresh enough for topology write; require pose_age_ms<=2000.\n";
    }
    result.stdout_text = out.str();
    return result;
}

CommandResult OperatorPanel::addTopologyWaypoint(const std::string& name, bool confirmed) const
{
    CommandResult result;
    const std::string waypoint_name = trimAscii(name);
    if (waypoint_name.empty()) {
        result.exit_code = 2;
        result.stdout_text = "topology add requires a waypoint name: /topology add NAME confirm\n";
        return result;
    }
    if (!confirmed) {
        result.exit_code = 2;
        result.stdout_text = "topology add requires explicit confirmation: /topology add NAME confirm\n";
        return result;
    }

    const auto response = sendGatewayCommand({
        {"action", "add_current_pose_waypoint"},
        {"name", waypoint_name},
        {"operator_ack", true},
    });
    result.exit_code = response.value("accepted", false) ? 0 : 3;
    result.stdout_text = response.dump(2) + "\n";
    return result;
}

CommandResult OperatorPanel::startRviz2(bool confirmed) const
{
    CommandResult result;
    if (!confirmed) {
        result.exit_code = 2;
        result.stdout_text = "rviz2 start requires explicit confirmation: /rviz2 start confirm\n";
        return result;
    }

    std::ostringstream cmd;
    cmd << "cd " << shellQuote(config_.repo_root)
        << " && bash " << shellQuote(config_.start_rviz2_script);
    return runShellCommandWithInput(cmd.str(), "");
}

CommandResult OperatorPanel::executeSemanticRoute(const SemanticRoute& route) const
{
    CommandResult result;
    std::ostringstream out;
    out << "C++语义路由：" << route.reason << "\n";
    out << "目标队列：";
    for (std::size_t i = 0; i < route.targets.size(); ++i) {
        if (i) out << " -> ";
        out << route.targets[i].name << "(" << route.targets[i].node_id << ")";
    }
    out << "\n";

    const auto unverified_targets = unverifiedRouteTargets(route);
    if (!unverified_targets.empty()) {
        out << "verification_guard: target requires standing verification/calibration before motion: "
            << joinStrings(unverified_targets, ", ") << "\n";
        if (config_.execute_enabled) {
            result.exit_code = 3;
            result.stdout_text = out.str();
            return result;
        }
    }

    if (!config_.execute_enabled) {
        out << "执行状态：干跑/未下发运动。输入 /execute on 后才允许真实执行。\n";
        out << "任务队列JSON：" << route.task_queue.dump(2) << "\n";
    }

    QueueExecutorConfig executor_config;
    executor_config.gateway_client = config_.gateway_client;
    executor_config.network_interface = config_.network_interface;
    executor_config.gateway_timeout_s = config_.gateway_timeout_s;
    executor_config.gateway_startup_wait_s = config_.gateway_startup_wait_s;
    executor_config.arrival_distance_m = config_.arrival_distance_m;
    executor_config.arrival_monitor_s = config_.arrival_monitor_s;
    executor_config.safety_limits.arrival_distance_m = config_.arrival_distance_m;
    executor_config.feedback_policy.slam_poll_interval_s = config_.slam_poll_interval_s;
    executor_config.feedback_policy.ui_refresh_interval_s = config_.ui_refresh_interval_s;
    executor_config.feedback_policy.operator_feedback_interval_s = config_.operator_feedback_interval_s;
    executor_config.feedback_policy.llm_feedback_interval_s = config_.llm_feedback_interval_s;
    executor_config.feedback_policy.max_consecutive_gateway_errors = config_.max_consecutive_gateway_errors;
    executor_config.feedback_policy.max_arrival_samples = config_.max_arrival_samples;
    executor_config.feedback_policy.max_feedback_events = config_.max_feedback_events;
    executor_config.feedback_policy.max_llm_feedback_events = config_.max_llm_feedback_events;
    executor_config.execute_enabled = config_.execute_enabled;
    const QueueExecutor executor(executor_config);
    const QueueExecutionResult execution = executor.execute(route);
    out << execution.stdout_text;
    out << "队列执行JSON：" << execution.execution.dump(2) << "\n";
    result.stdout_text = out.str();
    result.exit_code = execution.exit_code;
    return result;
}

void OperatorPanel::printHelp() const
{
    std::cout << "Extra safe commands:\n"
              << "  /relocate ANCHOR_ID confirm   Relocalize against a registry anchor; no chassis motion\n";
    std::cout << "命令:\n"
              << "  /status              刷新一次世界状态\n"
              << "  /start-slam          启动/确认雷达 driver 和 SLAM\n"
              << "  /mapping start confirm        显式确认后开启建图\n"
              << "  /mapping end confirm [pcd]    显式确认后结束建图并保存地图\n"
              << "  /topology preview NAME        只读预览当前位置拓扑点候选\n"
              << "  /topology add NAME confirm    显式确认后记录当前位置拓扑点\n"
              << "  /rviz2 start confirm          显式确认后打开 RViz2 可视化\n"
              << "  /watch [秒]          连续显示世界状态；0 表示一直显示\n"
              << "  /weak on|off         切换弱网摘要显示\n"
              << "  /execute on|off      是否允许真实下发运动；默认 off\n"
              << "  /current NODE_ID     设置当前位置锚点，用于自动重定位\n"
              << "  /help                显示帮助\n"
              << "  /quit                退出\n"
              << "  其他文本             作为中文 LLM 指令发送\n";
}

void OperatorPanel::setWeakMode(bool enabled)
{
    config_.weak_link_mode = enabled;
    std::cout << "弱网摘要显示: " << (enabled ? "on" : "off") << "\n";
}

void OperatorPanel::setExecute(bool enabled)
{
    config_.execute_enabled = enabled;
    std::cout << "真实执行: " << (enabled ? "on" : "off") << "\n";
}

bool OperatorPanel::handleSlashCommand(const std::string& line)
{
    auto markOk = [this]() {
        last_command_exit_code_ = 0;
        return true;
    };
    auto printResult = [this](const std::string& label, const CommandResult& result) {
        if (!result.stdout_text.empty()) std::cout << result.stdout_text;
        if (!result.stderr_text.empty()) std::cerr << result.stderr_text;
        std::cout << label << "_exit_code=" << result.exit_code << "\n";
        last_command_exit_code_ = result.exit_code;
    };

    if (line.empty() || line == "/status") {
        printStatusOnce();
        return markOk();
    }
    if (line == "/help") {
        printHelp();
        return markOk();
    }
    if (line == "/start-slam") {
        const auto result = ensureSlam();
        printResult("start_slam", result);
        printStatusOnce();
        return true;
    }
    if (line.rfind("/watch", 0) == 0) {
        std::istringstream ss(line);
        std::string token;
        int seconds = 10;
        ss >> token >> seconds;
        watchWorld(seconds);
        return markOk();
    }
    if (line == "/weak on") {
        setWeakMode(true);
        return markOk();
    }
    if (line == "/weak off") {
        setWeakMode(false);
        return markOk();
    }
    if (line == "/execute on") {
        setExecute(true);
        return markOk();
    }
    if (line == "/execute off") {
        setExecute(false);
        return markOk();
    }
    if (line.rfind("/current ", 0) == 0) {
        config_.current_node = trimAscii(line.substr(9));
        std::cout << "当前位置锚点: " << config_.current_node << "\n";
        return markOk();
    }

    std::vector<std::string> tokens;
    std::istringstream ss(line);
    for (std::string token; ss >> token;) tokens.push_back(token);
    if (tokens.empty()) return markOk();

    try {
        if (tokens[0] == "/relocate") {
            if (tokens.size() >= 3) {
                printResult("relocate", relocateAnchor(tokens[1], tokens[2] == "confirm"));
                return true;
            }
            std::cout << "usage: /relocate ANCHOR_ID confirm\n";
            last_command_exit_code_ = 2;
            return true;
        }

        if (tokens[0] == "/mapping") {
            if (tokens.size() >= 3 && tokens[1] == "start") {
                printResult("mapping_start", startMapping(tokens[2] == "confirm"));
                return true;
            }
            if (tokens.size() >= 3 && tokens[1] == "end") {
                const std::string map_path = tokens.size() >= 4 ? tokens[3] : "";
                printResult("mapping_end", endMapping(map_path, tokens[2] == "confirm"));
                return true;
            }
            std::cout << "usage: /mapping start confirm | /mapping end confirm [map_path]\n";
            last_command_exit_code_ = 2;
            return true;
        }

        if (tokens[0] == "/topology") {
            if (tokens.size() >= 3 && tokens[1] == "preview") {
                printResult("topology_preview", previewTopologyWaypoint(tokens[2]));
                return true;
            }
            if (tokens.size() >= 4 && tokens[1] == "add") {
                printResult("topology_add", addTopologyWaypoint(tokens[2], tokens[3] == "confirm"));
                return true;
            }
            std::cout << "usage: /topology preview NAME | /topology add NAME confirm\n";
            last_command_exit_code_ = 2;
            return true;
        }

        if (tokens[0] == "/rviz2") {
            if (tokens.size() >= 3 && tokens[1] == "start") {
                printResult("rviz2_start", startRviz2(tokens[2] == "confirm"));
                return true;
            }
            std::cout << "usage: /rviz2 start confirm\n";
            last_command_exit_code_ = 2;
            return true;
        }
    } catch (const std::exception& exc) {
        std::cerr << "command failed: " << exc.what() << "\n";
        last_command_exit_code_ = 1;
        return true;
    }

    return false;
}

int OperatorPanel::runInteractive()
{
    std::cout << "GO2W operator panel. 输入 /help 查看命令。\n";
    if (config_.ensure_slam_on_start) {
        std::cout << "正在启动/确认 SLAM 与雷达 driver...\n";
        const auto result = ensureSlam();
        if (!result.stdout_text.empty()) std::cout << result.stdout_text;
        if (!result.stderr_text.empty()) std::cerr << result.stderr_text;
        std::cout << "start_slam_exit_code=" << result.exit_code << "\n";
        last_command_exit_code_ = result.exit_code;
    }
    printStatusOnce();
    std::string line;
    while (true) {
        std::cout << (config_.execute_enabled ? "go2w[EXEC]> " : "go2w[dry]> ") << std::flush;
        if (!std::getline(std::cin, line)) break;
        line = trimAscii(line);
        if (line == "/quit" || line == "/exit") {
            break;
        }
        if (line.empty() || line.rfind("/", 0) == 0) {
            if (!handleSlashCommand(line)) {
                std::cout << "unknown panel command. 输入 /help 查看命令。\n";
                last_command_exit_code_ = 2;
            }
            continue;
        }

        const auto result = submitUserCommand(line);
        if (!result.stdout_text.empty()) std::cout << result.stdout_text;
        if (!result.stderr_text.empty()) std::cerr << result.stderr_text;
        std::cout << "exit_code=" << result.exit_code << "\n";
        last_command_exit_code_ = result.exit_code;
    }
    return last_command_exit_code_;
}

}  // namespace go2w
