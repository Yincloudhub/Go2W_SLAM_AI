#include "go2w/semantic_router.hpp"

#include <algorithm>
#include <cmath>
#include <set>
#include <sstream>
#include <stdexcept>

namespace go2w {
namespace {

double jsonDouble(const nlohmann::json& object, const std::string& key, double fallback)
{
    if (!object.is_object() || !object.contains(key) || object.at(key).is_null()) return fallback;
    return object.at(key).get<double>();
}

int jsonInt(const nlohmann::json& object, const std::string& key, int fallback)
{
    if (!object.is_object() || !object.contains(key) || object.at(key).is_null()) return fallback;
    return object.at(key).get<int>();
}

double yawFromQuaternion(double qx, double qy, double qz, double qw)
{
    const double siny_cosp = 2.0 * (qw * qz + qx * qy);
    const double cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz);
    return std::atan2(siny_cosp, cosy_cosp);
}

std::vector<std::string> uniqueTerms(const nlohmann::json& node)
{
    std::vector<std::string> terms;
    auto add = [&](const std::string& term) {
        if (term.empty()) return;
        if (std::find(terms.begin(), terms.end(), term) == terms.end()) terms.push_back(term);
    };
    add(node.value("node_id", ""));
    add(node.value("name", ""));
    if (node.contains("aliases") && node.at("aliases").is_array()) {
        for (const auto& alias : node.at("aliases")) {
            if (alias.is_string()) add(alias.get<std::string>());
        }
    }
    return terms;
}

bool containsAny(const std::string& text, const std::vector<std::string>& needles)
{
    for (const auto& needle : needles) {
        if (text.find(needle) != std::string::npos) return true;
    }
    return false;
}

}  // namespace

SemanticRouter::SemanticRouter(nlohmann::json registry, std::string map_id)
    : registry_(std::move(registry))
    , map_id_(std::move(map_id))
{
}

const nlohmann::json* SemanticRouter::findMap() const
{
    if (!registry_.contains("maps") || !registry_.at("maps").is_array()) return nullptr;
    for (const auto& map : registry_.at("maps")) {
        if (map.value("map_id", "") == map_id_) return &map;
    }
    return nullptr;
}

const nlohmann::json* SemanticRouter::findNode(const std::string& node_id) const
{
    const auto* map = findMap();
    if (!map || !map->contains("topology_nodes") || !map->at("topology_nodes").is_array()) return nullptr;
    for (const auto& node : map->at("topology_nodes")) {
        if (node.value("node_id", "") == node_id) {
            if (nodeDisabled(node)) return nullptr;
            return &node;
        }
    }
    return nullptr;
}

std::string SemanticRouter::mapPath(const std::string& fallback) const
{
    const auto* map = findMap();
    if (map && map->contains("pcd_path") && map->at("pcd_path").is_string()) return map->at("pcd_path").get<std::string>();
    return fallback;
}

bool SemanticRouter::nodeHasTag(const nlohmann::json& node, const std::string& tag) const
{
    if (!node.contains("tags") || !node.at("tags").is_array()) return false;
    for (const auto& item : node.at("tags")) {
        if (item.is_string() && item.get<std::string>() == tag) return true;
    }
    return false;
}

bool SemanticRouter::nodeDisabled(const nlohmann::json& node) const
{
    return nodeHasTag(node, "disabled") || nodeHasTag(node, "ui_disabled") || nodeHasTag(node, "deleted");
}

std::vector<ResolvedTarget> SemanticRouter::resolveTargets(const std::string& text) const
{
    const auto* map = findMap();
    if (!map || !map->contains("topology_nodes") || !map->at("topology_nodes").is_array()) return {};

    std::vector<ResolvedTarget> matches;
    std::set<std::string> seen;
    for (const auto& node : map->at("topology_nodes")) {
        if (!node.is_object()) continue;
        if (nodeDisabled(node)) continue;
        ResolvedTarget target;
        target.node_id = node.value("node_id", "");
        target.name = node.value("name", target.node_id);
        target.photo_required = nodeHasTag(node, "photo_required");
        target.requires_standing_verification = nodeHasTag(node, "needs_standing_verification");
        target.needs_calibration = nodeHasTag(node, "needs_calibration") || target.requires_standing_verification;
        std::size_t first = std::string::npos;
        for (const auto& term : uniqueTerms(node)) {
            const auto pos = text.find(term);
            if (pos == std::string::npos) continue;
            target.matched_terms.push_back(term);
            first = std::min(first, pos);
        }
        if (target.matched_terms.empty() || target.node_id.empty()) continue;
        if (seen.count(target.node_id)) continue;
        seen.insert(target.node_id);
        target.first_index = first;
        matches.push_back(target);
    }

    std::sort(matches.begin(), matches.end(), [](const ResolvedTarget& lhs, const ResolvedTarget& rhs) {
        if (lhs.first_index != rhs.first_index) return lhs.first_index < rhs.first_index;
        return lhs.node_id < rhs.node_id;
    });
    return matches;
}

bool SemanticRouter::commandRequestsCapture(const std::string& text) const
{
    return containsAny(text, {
        "拍照",
        "拍张照",
        "拍个照",
        "拍一张照",
        "拍一张照片",
        "照片",
        "看一眼",
        "看看",
        "关键帧",
        "photo",
        "capture",
        "keyframe",
    });
}

nlohmann::json SemanticRouter::poseToUnitreeJson(const nlohmann::json& node, double speed_mps, int mode) const
{
    const auto& pose = node.at("pose");
    const double qx = jsonDouble(pose, "q_x", 0.0);
    const double qy = jsonDouble(pose, "q_y", 0.0);
    const double qz = jsonDouble(pose, "q_z", 0.0);
    const double qw = jsonDouble(pose, "q_w", 1.0);
    const double yaw = pose.contains("yaw") ? jsonDouble(pose, "yaw", 0.0) : yawFromQuaternion(qx, qy, qz, qw);
    return {
        {"name", node.value("node_id", "llm_goal")},
        {"x", jsonDouble(pose, "x", 0.0)},
        {"y", jsonDouble(pose, "y", 0.0)},
        {"z", jsonDouble(pose, "z", 0.0)},
        {"q_x", qx},
        {"q_y", qy},
        {"q_z", qz},
        {"q_w", qw},
        {"yaw", yaw},
        {"speed", speed_mps > 0.0 ? speed_mps : jsonDouble(pose, "speed", 0.3)},
        {"mode", mode >= 0 ? mode : jsonInt(pose, "mode", 0)},
    };
}

nlohmann::json SemanticRouter::buildNavigateCommand(const std::string& node_id, double speed_mps, int mode) const
{
    const auto* node = findNode(node_id);
    if (!node) throw std::runtime_error("unknown target node: " + node_id);
    return {
        {"action", "navigate_to_pose"},
        {"map_id", map_id_},
        {"target_node", node->value("node_id", node_id)},
        {"target_pose", poseToUnitreeJson(*node, speed_mps, mode)},
    };
}

SemanticRoute SemanticRouter::planText(const std::string& text, double speed_mps, int mode) const
{
    SemanticRoute route;
    route.capture_requested = commandRequestsCapture(text);
    route.targets = resolveTargets(text);
    route.matched = !route.targets.empty();
    route.multi_target = route.targets.size() > 1;
    if (!route.matched) {
        route.reason = "no topology alias matched; fallback to LLM";
        return route;
    }

    route.reason = route.multi_target ? "C++ sequential topology queue" : "C++ topology route";
    route.slam_commands = nlohmann::json::array();
    route.task_queue = {
        {"queue_id", "cpp_queue"},
        {"mode", "sequential"},
        {"status", "planned"},
        {"source", "deterministic_cpp"},
        {"targets", nlohmann::json::array()},
        {"steps", nlohmann::json::array()},
        {"communication_policy", {{"mode", "normal"}, {"send", {"task_state", "navigation_feedback", "world_state_summary"}}, {"drop", nlohmann::json::array()}, {"reason", "normal link"}}},
    };

    bool explicit_capture_used = false;
    int step_index = 0;
    for (std::size_t i = 0; i < route.targets.size(); ++i) {
        const auto& target = route.targets[i];
        route.task_queue["targets"].push_back(target.node_id);
        auto command = buildNavigateCommand(target.node_id, speed_mps, mode);
        route.slam_commands.push_back(command);
        route.task_queue["steps"].push_back({
            {"task_id", "task_" + std::to_string(++step_index)},
            {"action", "navigate"},
            {"target_node", target.node_id},
            {"target_name", target.name},
            {"status", "pending"},
            {"requires_preflight", true},
            {"needs_calibration", target.needs_calibration},
            {"requires_standing_verification", target.requires_standing_verification},
            {"semantic_reason", "matched target from user command"},
        });
        const bool should_capture = target.photo_required || (route.capture_requested && !explicit_capture_used && (i == 0 || route.targets.size() > 1));
        if (should_capture) {
            explicit_capture_used = true;
            route.task_queue["steps"].push_back({
                {"task_id", "task_" + std::to_string(++step_index)},
                {"action", "capture_keyframe"},
                {"target_node", target.node_id},
                {"target_name", target.name},
                {"status", "pending"},
                {"requires_preflight", false},
                {"semantic_reason", "photo requested or target marked photo_required"},
            });
        }
    }
    return route;
}

}  // namespace go2w
