#pragma once

#include <string>

#include <json.hpp>

#include "slam_gateway/models.hpp"

namespace slam_gateway {

struct NavigationAuthorizationResult {
    bool authorized{false};
    std::string reason{"navigation_target_not_authorized"};
    nlohmann::json details{nlohmann::json::object()};
    PoseData authorized_pose;
};

class NavigationTargetAuthorizer {
public:
    NavigationTargetAuthorizer(std::string registry_path, std::string registry_map_id);

    NavigationAuthorizationResult authorize(
        const std::string& command_map_id,
        const std::string& requested_map_path,
        const std::string& target_node,
        const nlohmann::json& target_pose,
        const CurrentPose& current_pose) const;

    const std::string& registryPath() const { return registry_path_; }
    const std::string& registryMapId() const { return registry_map_id_; }
    bool ready() const { return load_error_.empty(); }
    const std::string& loadError() const { return load_error_; }

private:
    std::string registry_path_;
    std::string registry_map_id_;
    nlohmann::json registry_snapshot_{nlohmann::json::object()};
    std::string load_error_;
};

}  // namespace slam_gateway
