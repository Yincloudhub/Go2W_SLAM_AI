#include "go2w/operator_panel.hpp"

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
    const nlohmann::json request = {{"action", "get_world_state"}};
    const std::string command = shellQuote(config_.gateway_client) + " " + shellQuote(config_.network_interface);
    const CommandResult result = runShellCommandWithInput(command, request.dump() + "\n");
    const auto objects = extractJsonObjects(result.stdout_text);
    for (const auto& object : objects) {
        if (object.contains("world_state")) return object;
    }
    throw std::runtime_error("gateway did not return world_state: " + result.stderr_text);
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
    const nlohmann::json empty = nlohmann::json::object();
    const auto* pose_ptr = objectAt(result, {"world_state", "current_pose", "pose"});
    const auto* loc_ptr = objectAt(result, {"world_state", "localization"});
    const auto* health_ptr = objectAt(result, {"world_state", "slam_health"});
    const auto* safety_ptr = objectAt(result, {"world_state", "safety"});
    const auto* nav_ptr = objectAt(result, {"world_state", "navigation"});
    const auto* obstacle_ptr = objectAt(result, {"world_state", "local_obstacle"});
    const nlohmann::json& pose = pose_ptr && pose_ptr->is_object() ? *pose_ptr : empty;
    const nlohmann::json& loc = loc_ptr && loc_ptr->is_object() ? *loc_ptr : empty;
    const nlohmann::json& health = health_ptr && health_ptr->is_object() ? *health_ptr : empty;
    const nlohmann::json& safety = safety_ptr && safety_ptr->is_object() ? *safety_ptr : empty;
    const nlohmann::json& nav = nav_ptr && nav_ptr->is_object() ? *nav_ptr : empty;
    const nlohmann::json& obstacle = obstacle_ptr && obstacle_ptr->is_object() ? *obstacle_ptr : empty;

    const bool allow = jsonBool(objectAt(safety, {"allow_navigation"}), false);
    const std::string target_value = jsonString(objectAt(nav, {"target_node"}), "无");
    const std::string target = target_value.empty() ? "无" : target_value;
    const bool has_target = target != "无" && jsonString(objectAt(nav, {"state"}), "") != "idle";
    std::ostringstream ss;
    ss << "定位:" << jsonString(objectAt(loc, {"status"}))
       << " conf=" << fmtDouble(loc.value("confidence", nlohmann::json(nullptr))) << " | "
       << "SLAM:" << jsonString(objectAt(health, {"status"})) << " | "
       << "位置:x=" << fmtDouble(pose.value("x", nlohmann::json(nullptr))) << ", y=" << fmtDouble(pose.value("y", nlohmann::json(nullptr))) << ", yaw=" << fmtDouble(pose.value("yaw", nlohmann::json(nullptr))) << " | "
       << "最近点:" << nearestNodeText(result) << " | "
       << "导航:" << jsonString(objectAt(nav, {"state"})) << " 目标:" << target
       << " 距目标:" << (has_target ? fmtMeters(nav.value("distance_to_goal_m", nlohmann::json(nullptr))) : "无") << " | "
       << "前方净空:" << fmtMeters(obstacle.value("front_clearance_m", nlohmann::json(nullptr))) << " | "
       << "安全:" << (allow ? "允许导航" : "禁止导航") << "(" << jsonString(objectAt(safety, {"reason"})) << ")";
    return ss.str();
}

std::string OperatorPanel::formatWeakWorldState(const nlohmann::json& result) const
{
    const nlohmann::json empty = nlohmann::json::object();
    const auto* loc_ptr = objectAt(result, {"world_state", "localization"});
    const auto* health_ptr = objectAt(result, {"world_state", "slam_health"});
    const auto* safety_ptr = objectAt(result, {"world_state", "safety"});
    const auto* nav_ptr = objectAt(result, {"world_state", "navigation"});
    const nlohmann::json& loc = loc_ptr && loc_ptr->is_object() ? *loc_ptr : empty;
    const nlohmann::json& health = health_ptr && health_ptr->is_object() ? *health_ptr : empty;
    const nlohmann::json& safety = safety_ptr && safety_ptr->is_object() ? *safety_ptr : empty;
    const nlohmann::json& nav = nav_ptr && nav_ptr->is_object() ? *nav_ptr : empty;
    const bool allow = jsonBool(objectAt(safety, {"allow_navigation"}), false);
    std::ostringstream ss;
    ss << "弱网摘要 | 定位:" << jsonString(objectAt(loc, {"status"}))
       << " | SLAM:" << jsonString(objectAt(health, {"status"}))
       << " | 最近点:" << nearestNodeText(result)
       << " | 导航:" << jsonString(objectAt(nav, {"state"}))
       << " | 安全:" << (allow ? "允许" : "禁止") << "(" << jsonString(objectAt(safety, {"reason"})) << ")";
    return ss.str();
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
    if (!config_.current_node.empty()) cmd << " --current-node " << shellQuote(config_.current_node);
    return runShellCommandWithInput(cmd.str(), "");
}

void OperatorPanel::printHelp() const
{
    std::cout << "命令:\n"
              << "  /status              刷新一次世界状态\n"
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
    printStatusOnce();
    std::string line;
    while (true) {
        std::cout << (config_.execute_enabled ? "go2w[EXEC]> " : "go2w[dry]> ") << std::flush;
        if (!std::getline(std::cin, line)) break;
        if (line.empty() || line == "/status") {
            printStatusOnce();
        } else if (line == "/help") {
            printHelp();
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
