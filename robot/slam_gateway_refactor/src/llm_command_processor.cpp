#include "slam_gateway/llm_command_processor.hpp"
#include "slam_gateway/obstacle_policy.hpp"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <string>
#include <utility>

namespace slam_gateway {
namespace {

bool hasFiniteNumber(const nlohmann::json& j, const char* key)
{
    return j.contains(key) && j.at(key).is_number() && std::isfinite(j.at(key).get<double>());
}

bool hasOptionalFiniteNumber(const nlohmann::json& j, const char* key)
{
    return !j.contains(key) || (j.at(key).is_number() && std::isfinite(j.at(key).get<double>()));
}

std::string validatePoseJson(const nlohmann::json& j, bool require_positive_speed)
{
    if (!hasFiniteNumber(j, "x")) return "target_pose.x must be a finite number";
    if (!hasFiniteNumber(j, "y")) return "target_pose.y must be a finite number";
    if (!hasOptionalFiniteNumber(j, "z")) return "target_pose.z must be a finite number";
    if (!hasOptionalFiniteNumber(j, "q_x")) return "target_pose.q_x must be a finite number";
    if (!hasOptionalFiniteNumber(j, "q_y")) return "target_pose.q_y must be a finite number";
    if (!hasOptionalFiniteNumber(j, "q_z")) return "target_pose.q_z must be a finite number";
    if (!hasOptionalFiniteNumber(j, "q_w")) return "target_pose.q_w must be a finite number";
    if (!hasOptionalFiniteNumber(j, "speed")) return "target_pose.speed must be a finite number";
    if (j.contains("speed")) {
        const double speed = j.at("speed").get<double>();
        if ((require_positive_speed && speed <= 0.0) || speed < 0.0 || speed > 0.8) {
            return "target_pose.speed out of safe range";
        }
    }
    if (j.contains("mode")) {
        if (!j.at("mode").is_number_integer()) return "target_pose.mode must be an integer";
        const int mode = j.at("mode").get<int>();
        if (mode != 0 && mode != 1) return "target_pose.mode must be 0 or 1";
    }
    return "";
}

bool hasOperatorAck(const nlohmann::json& cmd)
{
    const auto ack = cmd.find("operator_ack");
    if (ack != cmd.end() && ack->is_boolean() && ack->get<bool>()) return true;
    const auto confirm = cmd.find("confirm");
    return confirm != cmd.end() && confirm->is_boolean() && confirm->get<bool>();
}

bool hasNavigationSessionAuthority(const nlohmann::json& cmd,
                                   const std::string& configured_token)
{
    if (configured_token.empty()) return false;
    const auto token = cmd.find("navigation_session_token");
    if (token == cmd.end() || !token->is_string()) return false;
    return token->get<std::string>() == configured_token;
}

bool isFreshEnoughForWaypoint(const LocalizationState& loc)
{
    const bool status_ok =
        loc.status == "localized" || loc.status == "degraded" || loc.status == "localized_or_tracking" || loc.status == "tracking";
    return status_ok && loc.pose_age_ms >= 0 && loc.pose_age_ms <= 2000;
}

bool isRepositionDirection(const std::string& direction)
{
    return direction == "forward" || direction == "backward" ||
        direction == "left" || direction == "right";
}

double directionalClearance(
    const LocalObstacleSummary& obstacle,
    const std::string& direction)
{
    if (direction == "forward") return obstacle.front_clearance_m;
    if (direction == "backward") return obstacle.rear_clearance_m;
    if (direction == "left") return obstacle.left_clearance_m;
    if (direction == "right") return obstacle.right_clearance_m;
    return -1.0;
}

void offsetPose(
    PoseData& pose,
    const std::string& direction,
    double yaw,
    double distance_m)
{
    double forward_m = 0.0;
    double left_m = 0.0;
    if (direction == "forward") forward_m = distance_m;
    if (direction == "backward") forward_m = -distance_m;
    if (direction == "left") left_m = distance_m;
    if (direction == "right") left_m = -distance_m;
    pose.x += static_cast<float>(
        std::cos(yaw) * forward_m - std::sin(yaw) * left_m);
    pose.y += static_cast<float>(
        std::sin(yaw) * forward_m + std::cos(yaw) * left_m);
}

}  // namespace

LlmCommandProcessor::LlmCommandProcessor(SlamGateway& gateway,
                                         std::string navigation_session_token,
                                         const NavigationTargetAuthorizer* target_authorizer,
                                         std::function<std::string()> navigation_execution_guard)
    : gateway_(gateway),
      navigation_session_token_(std::move(navigation_session_token)),
      target_authorizer_(target_authorizer),
      navigation_execution_guard_(std::move(navigation_execution_guard))
{
}

nlohmann::json LlmCommandProcessor::reject(const std::string& reason) const
{
    return {
        {"accepted", false},
        {"reason", reason}
    };
}

nlohmann::json LlmCommandProcessor::ok(const std::string& action, const ServiceResult& result) const
{
    return {
        {"accepted", result.ok},
        {"action", action},
        {"status_code", result.status_code},
        {"data", result.data},
        {"world_state", gateway_.buildWorldStateJson()}
    };
}

PoseData LlmCommandProcessor::parsePose(const nlohmann::json& j)
{
    PoseData p;
    p.name = j.value("name", "llm_goal");
    p.x = j.value("x", 0.0f);
    p.y = j.value("y", 0.0f);
    p.z = j.value("z", 0.0f);
    p.q_x = j.value("q_x", 0.0f);
    p.q_y = j.value("q_y", 0.0f);
    p.q_z = j.value("q_z", 0.0f);
    p.q_w = j.value("q_w", 1.0f);
    p.mode = j.value("mode", 0);   // Obstacle mode: 0=avoid, 1=stop.
    p.speed = j.value("speed", 0.5f); // LLM path defaults to conservative speed.
    return p;
}

nlohmann::json LlmCommandProcessor::process(const nlohmann::json& cmd)
{
    // Security boundary: LLM must not call raw API IDs directly.
    if (cmd.contains("api_id") || cmd.contains("ROBOT_API_ID") || cmd.contains("raw_api")) {
        return reject("raw_api_id_is_forbidden; use validated action names only");
    }

    const std::string action = cmd.value("action", "");
    if (action.empty()) return reject("missing_action");

    if (action == "get_world_state") {
        return {{"accepted", true}, {"action", action}, {"world_state", gateway_.buildWorldStateJson()}};
    }

    if (action == "start_mapping") {
        if (!hasOperatorAck(cmd)) return reject("operator_ack_required_for_start_mapping");
        const std::string slam_type = cmd.value("slam_type", "indoor");
        return ok(action, gateway_.startMapping(slam_type));
    }

    if (action == "end_mapping") {
        if (!hasOperatorAck(cmd)) return reject("operator_ack_required_for_end_mapping");
        const std::string map_path = cmd.value("map_path", "/home/unitree/test.pcd");
        return ok(action, gateway_.endMapping(map_path));
    }

    if (action == "add_current_pose_waypoint") {
        if (!hasOperatorAck(cmd)) return reject("operator_ack_required_for_add_current_pose_waypoint");
        const auto loc = gateway_.getLocalizationState();
        if (!isFreshEnoughForWaypoint(loc)) {
            return {
                {"accepted", false},
                {"reason", "localization_not_fresh_enough_for_waypoint"},
                {"required_pose_age_ms_lte", 2000},
                {"localization", loc.toJson()},
                {"world_state", gateway_.buildWorldStateJson()}
            };
        }
        const std::string name = cmd.value("name", "");
        gateway_.addCurrentPoseAsWaypoint(name);
        return {
            {"accepted", true},
            {"action", action},
            {"waypoint_count", gateway_.waypointCount()},
            {"world_state", gateway_.buildWorldStateJson()}
        };
    }

    if (action == "relocate") {
        if (!hasOperatorAck(cmd)) {
            return reject("operator_ack_required_for_relocation");
        }
        if (target_authorizer_ == nullptr) {
            return reject("relocalization_anchor_authorizer_not_configured");
        }
        const std::string map_id = cmd.value("map_id", "");
        const std::string map_path = cmd.value("map_path", "");
        const std::string anchor_id = cmd.value("anchor_id", "");
        if (!cmd.contains("initial_pose") || !cmd["initial_pose"].is_object()) {
            return reject("missing_initial_pose");
        }
        const std::string pose_error = validatePoseJson(cmd["initial_pose"], false);
        if (!pose_error.empty()) {
            return reject(pose_error);
        }
        const auto authorization = target_authorizer_->authorizeRelocation(
            map_id,
            map_path,
            anchor_id,
            cmd["initial_pose"]);
        if (!authorization.authorized) {
            return {
                {"accepted", false},
                {"reason", authorization.reason},
                {"authorization", authorization.details},
                {"world_state", gateway_.buildWorldStateJson()}
            };
        }
        return ok(action, gateway_.startRelocation(map_path, authorization.authorized_pose));
    }

    if (action == "navigate_to_pose") {
        if (!hasNavigationSessionAuthority(cmd, navigation_session_token_)) {
            return reject("persistent_navigation_session_required");
        }
        if (!hasOperatorAck(cmd)) {
            return reject("operator_ack_required_for_navigation");
        }
        const std::string target_node = cmd.value("target_node", "");
        if (target_node.empty()) {
            return reject("target_node_required_for_navigation");
        }
        if (!cmd.contains("target_pose") || !cmd["target_pose"].is_object()) {
            return reject("missing_target_pose");
        }
        if (cmd["target_pose"].value("name", "") != target_node) {
            return reject("target_pose_name_must_match_target_node");
        }
        const std::string requested_map_path = cmd.value("map_path", "");
        if (requested_map_path.empty()) {
            return reject("map_path_required_for_navigation");
        }
        const auto current_pose = gateway_.getCurrentPose();
        if (current_pose.map_path.empty()) {
            return reject("current_localization_map_path_missing");
        }
        if (requested_map_path != current_pose.map_path) {
            return {
                {"accepted", false},
                {"reason", "navigation_map_path_mismatch"},
                {"requested_map_path", requested_map_path},
                {"current_map_path", current_pose.map_path},
                {"world_state", gateway_.buildWorldStateJson()}
            };
        }
        if (target_authorizer_ == nullptr) {
            return reject("navigation_target_authorizer_not_configured");
        }
        const auto authorization = target_authorizer_->authorize(
            cmd.value("map_id", ""),
            requested_map_path,
            target_node,
            cmd["target_pose"],
            current_pose);
        if (!authorization.authorized) {
            return {
                {"accepted", false},
                {"reason", authorization.reason},
                {"authorization", authorization.details},
                {"world_state", gateway_.buildWorldStateJson()}
            };
        }
        const std::string pose_error = validatePoseJson(cmd["target_pose"], true);
        if (!pose_error.empty()) {
            return reject(pose_error);
        }
        auto safety = gateway_.getSafetyDecision();
        if (!safety.allow_navigation) {
            return {{"accepted", false}, {"reason", "safety_blocked"}, {"safety", safety.toJson()}, {"world_state", gateway_.buildWorldStateJson()}};
        }
        if (safety.motion_direction == "unitree_pose_navigation_mode_0" &&
            authorization.authorized_pose.mode != 0) {
            return {
                {"accepted", false},
                {"reason", "supervised_navigation_requires_unitree_avoidance_mode_0"},
                {"safety", safety.toJson()},
                {"world_state", gateway_.buildWorldStateJson()}
            };
        }
        if (navigation_execution_guard_) {
            const std::string guard_reason = navigation_execution_guard_();
            if (!guard_reason.empty()) {
                return reject("navigation_session_guard_blocked:" + guard_reason);
            }
        }
        PoseData goal = authorization.authorized_pose;
        if (safety.recommended_mode == "conservative") {
            goal.speed = static_cast<float>(
                std::min<double>(
                    goal.speed,
                    safety.speed_limit_mps >= 0.0
                        ? safety.speed_limit_mps
                        : obstacle_policy::kConservativeSpeedMps));
        }
        return ok(action, gateway_.submitNavigationGoal(goal));
    }

    if (action == "supervised_departure" || action == "supervised_reposition") {
        const bool legacy_departure = action == "supervised_departure";
        const std::string direction =
            legacy_departure ? "forward" : cmd.value("direction", "");
        if (!hasNavigationSessionAuthority(cmd, navigation_session_token_)) {
            return reject("persistent_navigation_session_required");
        }
        if (!hasOperatorAck(cmd)) {
            return reject("operator_ack_required_for_supervised_reposition");
        }
        if (!isRepositionDirection(direction)) {
            return reject("supervised_reposition_direction_invalid");
        }
        if (!hasFiniteNumber(cmd, "distance_m")) {
            return reject("supervised_reposition_distance_required");
        }
        const double distance_m = cmd.at("distance_m").get<double>();
        const double requested_speed_mps = cmd.value(
            "speed_mps",
            obstacle_policy::kDepartureMaxSpeedMps);
        if (distance_m <= 0.0 ||
            (!legacy_departure &&
             distance_m < obstacle_policy::kRepositionMinDistanceM) ||
            distance_m > obstacle_policy::kDepartureMaxDistanceM) {
            return reject("supervised_reposition_distance_out_of_range");
        }
        if (!std::isfinite(requested_speed_mps) ||
            requested_speed_mps <= 0.0 ||
            requested_speed_mps > obstacle_policy::kDepartureMaxSpeedMps) {
            return reject("supervised_reposition_speed_out_of_range");
        }
        const auto obstacle = gateway_.getLocalObstacleSummary();
        if (!obstacle.supervised_release_active) {
            return reject("supervised_reposition_requires_engineering_release");
        }
        const auto safety = gateway_.getSafetyDecision();
        if (!safety.allow_navigation) {
            return {
                {"accepted", false},
                {"reason", "safety_blocked"},
                {"safety", safety.toJson()},
                {"world_state", gateway_.buildWorldStateJson()}
            };
        }
        const double required_front_m =
            distance_m +
            (legacy_departure
                ? obstacle_policy::kDepartureFrontReserveM
                : obstacle_policy::repositionReserveM(direction));
        const double observed_clearance_m =
            directionalClearance(obstacle, direction);
        if (observed_clearance_m < required_front_m) {
            return {
                {"accepted", false},
                {"reason", "supervised_reposition_clearance_insufficient"},
                {"direction", direction},
                {"required_directional_clearance_m", required_front_m},
                {"observed_directional_clearance_m", observed_clearance_m},
                {"world_state", gateway_.buildWorldStateJson()}
            };
        }
        if (navigation_execution_guard_) {
            const std::string guard_reason = navigation_execution_guard_();
            if (!guard_reason.empty()) {
                return reject("navigation_session_guard_blocked:" + guard_reason);
            }
        }
        const auto current = gateway_.getCurrentPose();
        const double current_yaw = quaternionToYaw(
            current.pose.q_x,
            current.pose.q_y,
            current.pose.q_z,
            current.pose.q_w);
        PoseData goal = current.pose;
        goal.name = "__supervised_reposition_" + direction + "__";
        offsetPose(goal, direction, current_yaw, distance_m);
        goal.mode = 0;
        goal.speed = static_cast<float>(requested_speed_mps);
        auto result = ok(
            action,
            gateway_.submitSupervisedReposition(
                direction,
                distance_m,
                requested_speed_mps));
        result["distance_m"] = distance_m;
        result["direction"] = direction;
        result["reposition_target_pose"] = goal.toJson();
        if (legacy_departure) {
            result["departure_target_pose"] = goal.toJson();
        }
        return result;
    }

    if (action == "pause_navigation") {
        return ok(action, gateway_.pauseNavigation());
    }

    if (action == "resume_navigation") {
        return reject("resume_requires_new_supervised_navigation_session");
    }

    if (action == "stop_slam") {
        if (!hasOperatorAck(cmd)) return reject("operator_ack_required_for_stop_slam");
        gateway_.taskThreadStop();
        return ok(action, gateway_.stopNode());
    }

    return reject("unknown_action:" + action);
}

}  // namespace slam_gateway
