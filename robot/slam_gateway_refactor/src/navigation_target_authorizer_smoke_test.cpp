#include "slam_gateway/navigation_target_authorizer.hpp"

#include <cmath>
#include <cstdio>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <string>

namespace {

void require(bool condition, const char* message)
{
    if (!condition) throw std::runtime_error(message);
}

nlohmann::json verifiedPose()
{
    return {
        {"name", "initial_point"},
        {"x", -0.39},
        {"y", 0.18},
        {"z", 0.0},
        {"q_x", 0.0},
        {"q_y", 0.0},
        {"q_z", -0.05497227502706773},
        {"q_w", 0.9984878812375985},
        {"speed", 0.3},
        {"mode", 0}
    };
}

nlohmann::json mappingOriginPose()
{
    return {
        {"name", "mapping_origin"},
        {"x", 0.0},
        {"y", 0.0},
        {"z", 0.0},
        {"q_x", 0.0},
        {"q_y", 0.0},
        {"q_z", 0.0},
        {"q_w", 1.0},
        {"speed", 0.0},
        {"mode", 0}
    };
}

nlohmann::json registryJson()
{
    return {
        {"version", 1},
        {"default_map_id", "go2w_real_site"},
        {"maps", nlohmann::json::array({
            {
                {"map_id", "go2w_real_site"},
                {"pcd_path", "/home/unitree/test.pcd"},
                {"mapping_origin_anchor_id", "mapping_origin"},
                {"relocalization_anchors", nlohmann::json::array({
                    {
                        {"anchor_id", "mapping_origin"},
                        {"status", "verified_startup"},
                        {"pose", mappingOriginPose()}
                    },
                    {
                        {"anchor_id", "candidate_anchor"},
                        {"status", "candidate"},
                        {"pose", mappingOriginPose()}
                    }
                })},
                {"archived_relocalization_anchors", nlohmann::json::array({
                    {
                        {"anchor_id", "initial_point"},
                        {"status", "candidate_failed"},
                        {"pose", verifiedPose()}
                    }
                })},
                {"topology_nodes", nlohmann::json::array({
                    {
                        {"node_id", "initial_point"},
                        {"tags", nlohmann::json::array({"live_verified", "ui_verified"})},
                        {"pose", verifiedPose()}
                    },
                    {
                        {"node_id", "standing_pending"},
                        {"tags", nlohmann::json::array({
                            "live_verified",
                            "needs_standing_verification"
                        })},
                        {"pose", {
                            {"x", 1.0}, {"y", 2.0}, {"z", 0.0},
                            {"q_x", 0.0}, {"q_y", 0.0}, {"q_z", 0.0}, {"q_w", 1.0},
                            {"speed", 0.3}, {"mode", 0}
                        }}
                    }
                })}
            }
        })}
    };
}

}  // namespace

int main()
{
    const std::string path = "/tmp/go2w_navigation_target_authorizer_smoke.json";
    {
        std::ofstream output(path);
        output << registryJson().dump(2);
    }

    slam_gateway::NavigationTargetAuthorizer authorizer(path, "go2w_real_site");
    slam_gateway::CurrentPose current_pose;
    current_pose.map_path = "/home/unitree/test.pcd";

    const auto allowed = authorizer.authorize(
        "go2w_real_site",
        "/home/unitree/test.pcd",
        "initial_point",
        verifiedPose(),
        current_pose);
    require(allowed.authorized, "exact live-verified registry target should be authorized");
    require(
        std::abs(allowed.authorized_pose.x - (-0.39f)) < 1e-5f,
        "authorized pose must come from the registry snapshot");

    const auto relocation_allowed = authorizer.authorizeRelocation(
        "go2w_real_site",
        "/home/unitree/test.pcd",
        "mapping_origin",
        mappingOriginPose());
    require(
        relocation_allowed.authorized,
        "exact active verified relocation anchor should be authorized");
    require(
        relocation_allowed.authorized_pose.speed == 0.0f,
        "relocation speed must be forced to zero");

    auto modified_relocation_pose = mappingOriginPose();
    modified_relocation_pose["x"] = 0.2;
    const auto modified_relocation = authorizer.authorizeRelocation(
        "go2w_real_site",
        "/home/unitree/test.pcd",
        "mapping_origin",
        modified_relocation_pose);
    require(
        !modified_relocation.authorized,
        "modified relocation anchor pose must be rejected");
    require(
        modified_relocation.reason == "relocalization_anchor_pose_mismatch:x",
        "modified relocation pose rejection reason mismatch");

    const auto archived_relocation = authorizer.authorizeRelocation(
        "go2w_real_site",
        "/home/unitree/test.pcd",
        "initial_point",
        verifiedPose());
    require(
        !archived_relocation.authorized,
        "archived relocation anchor must not be executable");
    require(
        archived_relocation.reason == "relocalization_active_anchor_not_found",
        "archived relocation rejection reason mismatch");

    auto candidate_pose = mappingOriginPose();
    candidate_pose["name"] = "candidate_anchor";
    const auto candidate_relocation = authorizer.authorizeRelocation(
        "go2w_real_site",
        "/home/unitree/test.pcd",
        "candidate_anchor",
        candidate_pose);
    require(
        !candidate_relocation.authorized,
        "unverified active relocation anchor must be rejected");
    require(
        candidate_relocation.reason == "relocalization_verified_anchor_required",
        "unverified relocation rejection reason mismatch");

    auto modified_pose = verifiedPose();
    modified_pose["x"] = 0.5;
    const auto modified = authorizer.authorize(
        "go2w_real_site",
        "/home/unitree/test.pcd",
        "initial_point",
        modified_pose,
        current_pose);
    require(!modified.authorized, "modified target pose must be rejected");
    require(
        modified.reason == "navigation_target_pose_mismatch:x",
        "modified target pose rejection reason mismatch");

    auto lower_speed_pose = verifiedPose();
    lower_speed_pose["speed"] = 0.2;
    const auto lower_speed = authorizer.authorize(
        "go2w_real_site",
        "/home/unitree/test.pcd",
        "initial_point",
        lower_speed_pose,
        current_pose);
    require(lower_speed.authorized, "a lower requested speed should be authorized");
    require(
        std::abs(lower_speed.authorized_pose.speed - 0.2f) < 1e-5f,
        "authorized speed should retain the lower supervised request");

    auto excessive_speed_pose = verifiedPose();
    excessive_speed_pose["speed"] = 0.4;
    const auto excessive_speed = authorizer.authorize(
        "go2w_real_site",
        "/home/unitree/test.pcd",
        "initial_point",
        excessive_speed_pose,
        current_pose);
    require(!excessive_speed.authorized, "speed above registry limit must be rejected");
    require(
        excessive_speed.reason == "navigation_target_speed_exceeds_registry_limit",
        "excessive speed rejection reason mismatch");

    auto pending_pose = verifiedPose();
    pending_pose["name"] = "standing_pending";
    pending_pose["x"] = 1.0;
    pending_pose["y"] = 2.0;
    pending_pose["q_z"] = 0.0;
    pending_pose["q_w"] = 1.0;
    const auto pending = authorizer.authorize(
        "go2w_real_site",
        "/home/unitree/test.pcd",
        "standing_pending",
        pending_pose,
        current_pose);
    require(!pending.authorized, "standing-verification target must be rejected");
    require(
        pending.reason == "navigation_target_blocked_tag:needs_standing_verification",
        "standing-verification rejection reason mismatch");

    current_pose.map_path = "/home/unitree/other.pcd";
    const auto wrong_map = authorizer.authorize(
        "go2w_real_site",
        "/home/unitree/test.pcd",
        "initial_point",
        verifiedPose(),
        current_pose);
    require(!wrong_map.authorized, "active map path mismatch must be rejected");
    require(
        wrong_map.reason == "navigation_current_map_path_mismatch",
        "active map path rejection reason mismatch");

    current_pose.map_path = "/home/unitree/test.pcd";
    auto replaced_registry = registryJson();
    replaced_registry["maps"][0]["topology_nodes"][0]["pose"]["x"] = 9.0;
    std::ofstream(path) << replaced_registry.dump(2);
    const auto snapshot_still_authorized = authorizer.authorize(
        "go2w_real_site",
        "/home/unitree/test.pcd",
        "initial_point",
        verifiedPose(),
        current_pose);
    require(
        snapshot_still_authorized.authorized,
        "authorizer must use the startup registry snapshot instead of rereading a replaced file");

    std::remove(path.c_str());
    std::cout << "slam_gateway_navigation_target_authorizer_smoke_test=passed\n";
    return 0;
}
