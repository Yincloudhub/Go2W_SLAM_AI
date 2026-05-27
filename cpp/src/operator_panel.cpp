#include "go2w/operator_panel.hpp"
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

std::string joinStrings(const std::vector<std::string>& values, const std::string& sep)
{
    std::ostringstream ss;
    for (std::size_t i = 0; i < values.size(); ++i) {
        if (i) ss << sep;
        ss << values[i];
    }
    return ss.str();
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
    const std::string path = config_.repo_root + "/configs/maps/go2w_real_site_map_registry.json";
    std::ifstream file(path);
    if (!file) return nlohmann::json::object();
    return nlohmann::json::parse(readAll(file));
}

nlohmann::json OperatorPanel::getWorldState() const
{
    return sendGatewayCommand({{"action", "get_world_state"}});
}

nlohmann::json OperatorPanel::sendGatewayCommand(const nlohmann::json& command_json) const
{
    GatewayClient client({config_.gateway_client, config_.network_interface, config_.gateway_timeout_s});
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
    const SemanticRouter router(loadRegistry(), "go2w_real_site");
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
            if (!map.is_object() || map.value("map_id", "") != "go2w_real_site") continue;
            const auto* nodes = objectAt(map, {"topology_nodes"});
            if (!nodes || !nodes->is_array()) continue;
            for (const auto& node : *nodes) {
                if (!node.is_object()) continue;
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
        "Do not output coordinates, speeds, Unitree API ids, markdown, or extra text. "
        "If the command is unclear, return targets as an empty array.";
    const nlohmann::json user_payload = {
        {"command", text},
        {"current_node", config_.current_node},
        {"execute_enabled", config_.execute_enabled},
        {"candidates", candidates},
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
    const SemanticRouter router(loadRegistry(), "go2w_real_site");
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

    if (!config_.execute_enabled) {
        out << "执行状态：干跑/未下发运动。输入 /execute on 后才允许真实执行。\n";
        out << "任务队列JSON：" << route.task_queue.dump(2) << "\n";
    }

    QueueExecutorConfig executor_config;
    executor_config.gateway_client = config_.gateway_client;
    executor_config.network_interface = config_.network_interface;
    executor_config.gateway_timeout_s = config_.gateway_timeout_s;
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
    std::cout << "命令:\n"
              << "  /status              刷新一次世界状态\n"
              << "  /start-slam          启动/确认雷达 driver 和 SLAM\n"
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

int OperatorPanel::runInteractive()
{
    std::cout << "GO2W operator panel. 输入 /help 查看命令。\n";
    if (config_.ensure_slam_on_start) {
        std::cout << "正在启动/确认 SLAM 与雷达 driver...\n";
        const auto result = ensureSlam();
        if (!result.stdout_text.empty()) std::cout << result.stdout_text;
        if (!result.stderr_text.empty()) std::cerr << result.stderr_text;
        std::cout << "start_slam_exit_code=" << result.exit_code << "\n";
    }
    printStatusOnce();
    std::string line;
    while (true) {
        std::cout << (config_.execute_enabled ? "go2w[EXEC]> " : "go2w[dry]> ") << std::flush;
        if (!std::getline(std::cin, line)) break;
        line = trimAscii(line);
        if (line.empty() || line == "/status") {
            printStatusOnce();
        } else if (line == "/help") {
            printHelp();
        } else if (line == "/start-slam") {
            const auto result = ensureSlam();
            if (!result.stdout_text.empty()) std::cout << result.stdout_text;
            if (!result.stderr_text.empty()) std::cerr << result.stderr_text;
            std::cout << "start_slam_exit_code=" << result.exit_code << "\n";
            printStatusOnce();
        } else if (line == "/quit" || line == "/exit") {
            break;
        } else if (line.rfind("/watch", 0) == 0) {
            std::istringstream ss(line);
            std::string token;
            int seconds = 10;
            ss >> token >> seconds;
            watchWorld(seconds);
        } else if (line == "/weak on") {
            setWeakMode(true);
        } else if (line == "/weak off") {
            setWeakMode(false);
        } else if (line == "/execute on") {
            setExecute(true);
        } else if (line == "/execute off") {
            setExecute(false);
        } else if (line.rfind("/current ", 0) == 0) {
            config_.current_node = line.substr(9);
            std::cout << "当前位置锚点: " << config_.current_node << "\n";
        } else {
            const auto result = submitUserCommand(line);
            if (!result.stdout_text.empty()) std::cout << result.stdout_text;
            if (!result.stderr_text.empty()) std::cerr << result.stderr_text;
            std::cout << "exit_code=" << result.exit_code << "\n";
        }
    }
    return 0;
}

}  // namespace go2w
