#include "slam_gateway/navigation_target_authorizer.hpp"

#include <cctype>
#include <cmath>
#include <fstream>
#include <set>
#include <string>
#include <utility>

namespace slam_gateway {
namespace {

constexpr double kPoseTolerance = 1e-4;
constexpr double kSpeedTolerance = 1e-4;

NavigationAuthorizationResult reject(
    const std::string& reason,
    nlohmann::json details = nlohmann::json::object())
{
    return {false, reason, std::move(details), PoseData{}};
}

NavigationAuthorizationResult allow(
    const nlohmann::json& details,
    const PoseData& authorized_pose)
{
    return {true, "registry_target_authorized", details, authorized_pose};
}

RelocalizationAuthorizationResult rejectRelocation(
    const std::string& reason,
    nlohmann::json details = nlohmann::json::object())
{
    return {false, reason, std::move(details), PoseData{}};
}

RelocalizationAuthorizationResult allowRelocation(
    const nlohmann::json& details,
    const PoseData& authorized_pose)
{
    return {true, "registry_relocalization_anchor_authorized", details, authorized_pose};
}

bool closeEnough(double lhs, double rhs, double tolerance)
{
    return std::isfinite(lhs) && std::isfinite(rhs) && std::fabs(lhs - rhs) <= tolerance;
}

const nlohmann::json* findMap(const nlohmann::json& registry, const std::string& map_id)
{
    const auto maps = registry.find("maps");
    if (maps == registry.end() || !maps->is_array()) return nullptr;
    for (const auto& map : *maps) {
        if (map.is_object() && map.value("map_id", "") == map_id) return &map;
    }
    return nullptr;
}

const nlohmann::json* findNode(const nlohmann::json& map, const std::string& node_id)
{
    const auto nodes = map.find("topology_nodes");
    if (nodes == map.end() || !nodes->is_array()) return nullptr;
    for (const auto& node : *nodes) {
        if (node.is_object() && node.value("node_id", "") == node_id) return &node;
    }
    return nullptr;
}

const nlohmann::json* findRelocalizationAnchor(
    const nlohmann::json& map,
    const std::string& anchor_id)
{
    const auto anchors = map.find("relocalization_anchors");
    if (anchors == map.end() || !anchors->is_array()) return nullptr;
    for (const auto& anchor : *anchors) {
        if (anchor.is_object() && anchor.value("anchor_id", "") == anchor_id) {
            return &anchor;
        }
    }
    return nullptr;
}

bool verifiedRelocalizationStatus(const nlohmann::json& anchor)
{
    std::string status = anchor.value("status", "");
    for (auto& ch : status) {
        ch = static_cast<char>(std::tolower(static_cast<unsigned char>(ch)));
    }
    return status == "verified" || status.rfind("verified_", 0) == 0;
}

std::set<std::string> nodeTags(const nlohmann::json& node)
{
    std::set<std::string> tags;
    const auto values = node.find("tags");
    if (values == node.end() || !values->is_array()) return tags;
    for (const auto& value : *values) {
        if (value.is_string()) tags.insert(value.get<std::string>());
    }
    return tags;
}

NavigationAuthorizationResult compareNumber(
    const nlohmann::json& requested,
    const nlohmann::json& registered,
    const char* field,
    double tolerance)
{
    if (!requested.contains(field) || !requested.at(field).is_number()) {
        return reject(
            std::string("navigation_target_pose_field_required:") + field,
            {{"field", field}});
    }
    if (!registered.contains(field) || !registered.at(field).is_number()) {
        return reject(
            std::string("navigation_registry_pose_field_missing:") + field,
            {{"field", field}});
    }
    const double requested_value = requested.at(field).get<double>();
    const double registered_value = registered.at(field).get<double>();
    if (!closeEnough(requested_value, registered_value, tolerance)) {
        return reject(
            std::string("navigation_target_pose_mismatch:") + field,
            {
                {"field", field},
                {"requested", requested_value},
                {"registered", registered_value},
                {"tolerance", tolerance}
            });
    }
    return {true, "", nlohmann::json::object(), PoseData{}};
}

}  // namespace

NavigationTargetAuthorizer::NavigationTargetAuthorizer(
    std::string registry_path,
    std::string registry_map_id)
    : registry_path_(std::move(registry_path)),
      registry_map_id_(std::move(registry_map_id))
{
    std::ifstream input(registry_path_);
    if (!input) {
        load_error_ = "navigation_target_registry_open_failed";
        return;
    }
    try {
        input >> registry_snapshot_;
        if (!registry_snapshot_.is_object()) {
            load_error_ = "navigation_target_registry_root_invalid";
            registry_snapshot_ = nlohmann::json::object();
        }
    } catch (const std::exception& error) {
        load_error_ = std::string("navigation_target_registry_parse_failed:") + error.what();
        registry_snapshot_ = nlohmann::json::object();
    }
}

NavigationAuthorizationResult NavigationTargetAuthorizer::authorize(
    const std::string& command_map_id,
    const std::string& requested_map_path,
    const std::string& target_node,
    const nlohmann::json& target_pose,
    const CurrentPose& current_pose) const
{
    if (registry_path_.empty() || registry_map_id_.empty()) {
        return reject("navigation_target_registry_not_configured");
    }
    if (command_map_id != registry_map_id_) {
        return reject(
            "navigation_registry_map_id_mismatch",
            {{"requested_map_id", command_map_id}, {"registry_map_id", registry_map_id_}});
    }

    if (!load_error_.empty()) {
        return reject(
            load_error_,
            {{"registry_path", registry_path_}});
    }

    const auto* map = findMap(registry_snapshot_, registry_map_id_);
    if (map == nullptr) {
        return reject(
            "navigation_registry_map_not_found",
            {{"registry_map_id", registry_map_id_}});
    }
    const std::string registry_map_path = map->value("pcd_path", "");
    if (registry_map_path.empty()) {
        return reject("navigation_registry_map_path_missing");
    }
    if (requested_map_path != registry_map_path) {
        return reject(
            "navigation_registry_map_path_mismatch",
            {{"requested_map_path", requested_map_path}, {"registry_map_path", registry_map_path}});
    }
    if (current_pose.map_path != registry_map_path) {
        return reject(
            "navigation_current_map_path_mismatch",
            {{"current_map_path", current_pose.map_path}, {"registry_map_path", registry_map_path}});
    }

    const auto* node = findNode(*map, target_node);
    if (node == nullptr) {
        return reject("navigation_target_not_found", {{"target_node", target_node}});
    }

    const auto tags = nodeTags(*node);
    if (tags.count("live_verified") == 0) {
        return reject(
            "navigation_target_live_verified_required",
            {{"target_node", target_node}});
    }
    static const std::set<std::string> blocking_tags = {
        "disabled",
        "ui_disabled",
        "deleted",
        "needs_calibration",
        "needs_standing_verification",
        "requires_standing_verification"
    };
    for (const auto& tag : blocking_tags) {
        if (tags.count(tag) != 0) {
            return reject(
                "navigation_target_blocked_tag:" + tag,
                {{"target_node", target_node}, {"blocking_tag", tag}});
        }
    }

    const auto registered_pose_it = node->find("pose");
    if (registered_pose_it == node->end() || !registered_pose_it->is_object()) {
        return reject(
            "navigation_registry_target_pose_missing",
            {{"target_node", target_node}});
    }
    if (!target_pose.is_object()) {
        return reject("navigation_target_pose_must_be_object");
    }
    if (target_pose.value("name", "") != target_node) {
        return reject("navigation_target_pose_name_mismatch");
    }

    const auto& registered_pose = *registered_pose_it;
    for (const char* field : {"x", "y", "z", "q_x", "q_y", "q_z", "q_w"}) {
        const auto result = compareNumber(target_pose, registered_pose, field, kPoseTolerance);
        if (!result.authorized) return result;
    }
    if (!target_pose.contains("mode") || !target_pose.at("mode").is_number_integer()) {
        return reject("navigation_target_pose_field_required:mode", {{"field", "mode"}});
    }
    if (!registered_pose.contains("mode") || !registered_pose.at("mode").is_number_integer()) {
        return reject("navigation_registry_pose_field_missing:mode", {{"field", "mode"}});
    }
    const int requested_mode = target_pose.at("mode").get<int>();
    const int registered_mode = registered_pose.at("mode").get<int>();
    if (requested_mode != registered_mode) {
        return reject(
            "navigation_target_pose_mismatch:mode",
            {{"requested", requested_mode}, {"registered", registered_mode}});
    }

    PoseData authorized_pose = PoseData::fromJson(registered_pose);
    authorized_pose.name = target_node;
    if (target_pose.contains("speed") && target_pose.at("speed").is_number()) {
        const double requested_speed = target_pose.at("speed").get<double>();
        if (!std::isfinite(requested_speed) || requested_speed <= 0.0) {
            return reject("navigation_target_speed_invalid");
        }
        if (requested_speed > static_cast<double>(authorized_pose.speed) + kSpeedTolerance) {
            return reject(
                "navigation_target_speed_exceeds_registry_limit",
                {
                    {"requested_speed", requested_speed},
                    {"registry_speed_limit", authorized_pose.speed}
                });
        }
        authorized_pose.speed = static_cast<float>(requested_speed);
    }

    return allow(
        {
            {"registry_path", registry_path_},
            {"registry_map_id", registry_map_id_},
            {"map_path", registry_map_path},
            {"target_node", target_node},
            {"authorized_pose", authorized_pose.toJson()}
        },
        authorized_pose);
}

RelocalizationAuthorizationResult NavigationTargetAuthorizer::authorizeRelocation(
    const std::string& command_map_id,
    const std::string& requested_map_path,
    const std::string& anchor_id,
    const nlohmann::json& initial_pose) const
{
    if (registry_path_.empty() || registry_map_id_.empty()) {
        return rejectRelocation("relocalization_registry_not_configured");
    }
    if (command_map_id != registry_map_id_) {
        return rejectRelocation(
            "relocalization_registry_map_id_mismatch",
            {{"requested_map_id", command_map_id}, {"registry_map_id", registry_map_id_}});
    }
    if (!load_error_.empty()) {
        return rejectRelocation(load_error_, {{"registry_path", registry_path_}});
    }

    const auto* map = findMap(registry_snapshot_, registry_map_id_);
    if (map == nullptr) {
        return rejectRelocation(
            "relocalization_registry_map_not_found",
            {{"registry_map_id", registry_map_id_}});
    }
    const std::string registry_map_path = map->value("pcd_path", "");
    if (registry_map_path.empty()) {
        return rejectRelocation("relocalization_registry_map_path_missing");
    }
    if (requested_map_path != registry_map_path) {
        return rejectRelocation(
            "relocalization_registry_map_path_mismatch",
            {{"requested_map_path", requested_map_path}, {"registry_map_path", registry_map_path}});
    }
    if (anchor_id.empty()) {
        return rejectRelocation("relocalization_anchor_id_required");
    }

    const auto* anchor = findRelocalizationAnchor(*map, anchor_id);
    if (anchor == nullptr) {
        return rejectRelocation(
            "relocalization_active_anchor_not_found",
            {{"anchor_id", anchor_id}});
    }
    if (!verifiedRelocalizationStatus(*anchor)) {
        return rejectRelocation(
            "relocalization_verified_anchor_required",
            {{"anchor_id", anchor_id}, {"status", anchor->value("status", "")}});
    }

    const auto registered_pose_it = anchor->find("pose");
    if (registered_pose_it == anchor->end() || !registered_pose_it->is_object()) {
        return rejectRelocation(
            "relocalization_registry_anchor_pose_missing",
            {{"anchor_id", anchor_id}});
    }
    if (!initial_pose.is_object()) {
        return rejectRelocation("relocalization_initial_pose_must_be_object");
    }
    if (initial_pose.value("name", "") != anchor_id) {
        return rejectRelocation("relocalization_initial_pose_name_mismatch");
    }

    const auto& registered_pose = *registered_pose_it;
    for (const char* field : {"x", "y", "z", "q_x", "q_y", "q_z", "q_w"}) {
        const auto comparison = compareNumber(
            initial_pose,
            registered_pose,
            field,
            kPoseTolerance);
        if (!comparison.authorized) {
            return rejectRelocation(
                "relocalization_anchor_pose_mismatch:" + std::string(field),
                comparison.details);
        }
    }

    PoseData authorized_pose = PoseData::fromJson(registered_pose);
    authorized_pose.name = anchor_id;
    authorized_pose.speed = 0.0f;
    return allowRelocation(
        {
            {"registry_path", registry_path_},
            {"registry_map_id", registry_map_id_},
            {"map_path", registry_map_path},
            {"anchor_id", anchor_id},
            {"authorized_pose", authorized_pose.toJson()}
        },
        authorized_pose);
}

}  // namespace slam_gateway
