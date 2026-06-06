#pragma once

#include <cmath>
#include <chrono>
#include <cstdint>
#include <iomanip>
#include <sstream>
#include <string>
#include <vector>

#include <json.hpp>

namespace slam_gateway {

inline int64_t nowMs()
{
    using namespace std::chrono;
    return duration_cast<milliseconds>(steady_clock::now().time_since_epoch()).count();
}

inline int64_t wallClockNowMs()
{
    using namespace std::chrono;
    return duration_cast<milliseconds>(system_clock::now().time_since_epoch()).count();
}

inline double quaternionToYaw(double qx, double qy, double qz, double qw)
{
    const double siny_cosp = 2.0 * (qw * qz + qx * qy);
    const double cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz);
    return std::atan2(siny_cosp, cosy_cosp);
}

struct PoseData {
    std::string name{""};
    float x{0.0f};
    float y{0.0f};
    float z{0.0f};
    float q_x{0.0f};
    float q_y{0.0f};
    float q_z{0.0f};
    float q_w{1.0f};
    int mode{0};
    float speed{0.8f};

    std::string toNavigationJson() const
    {
        nlohmann::json j;
        j["data"]["targetPose"]["x"] = x;
        j["data"]["targetPose"]["y"] = y;
        j["data"]["targetPose"]["z"] = z;
        j["data"]["targetPose"]["q_x"] = q_x;
        j["data"]["targetPose"]["q_y"] = q_y;
        j["data"]["targetPose"]["q_z"] = q_z;
        j["data"]["targetPose"]["q_w"] = q_w;
        j["data"]["mode"] = mode;
        j["data"]["speed"] = speed;
        return j.dump(4);
    }

    nlohmann::json toJson() const
    {
        return {
            {"name", name},
            {"x", x}, {"y", y}, {"z", z},
            {"q_x", q_x}, {"q_y", q_y}, {"q_z", q_z}, {"q_w", q_w},
            {"yaw", quaternionToYaw(q_x, q_y, q_z, q_w)},
            {"mode", mode},
            {"speed", speed}
        };
    }

    static PoseData fromJson(const nlohmann::json& j)
    {
        PoseData p;
        if (j.contains("name")) p.name = j.value("name", "");
        p.x = j.value("x", 0.0f);
        p.y = j.value("y", 0.0f);
        p.z = j.value("z", 0.0f);
        p.q_x = j.value("q_x", 0.0f);
        p.q_y = j.value("q_y", 0.0f);
        p.q_z = j.value("q_z", 0.0f);
        p.q_w = j.value("q_w", 1.0f);
        p.mode = j.value("mode", 0);
        p.speed = j.value("speed", 0.8f);
        return p;
    }

    std::string summary() const
    {
        std::ostringstream oss;
        oss << "name:" << name
            << " x:" << x << " y:" << y << " z:" << z
            << " q_x:" << q_x << " q_y:" << q_y
            << " q_z:" << q_z << " q_w:" << q_w
            << " yaw:" << quaternionToYaw(q_x, q_y, q_z, q_w)
            << " speed:" << speed;
        return oss.str();
    }
};

struct CurrentPose {
    int64_t timestamp_ms{0};
    std::string map_id{"debug_map"};
    std::string frame_id{"map"};
    PoseData pose;
    std::string source{"rt/slam_info"};

    nlohmann::json toJson() const
    {
        return {
            {"type", "current_pose"},
            {"timestamp_ms", timestamp_ms},
            {"map_id", map_id},
            {"frame_id", frame_id},
            {"pose", pose.toJson()},
            {"source", source}
        };
    }
};

struct LocalizationState {
    int64_t timestamp_ms{0};
    std::string map_id{"debug_map"};
    std::string status{"not_started"};
    double confidence{-1.0};
    int64_t pose_age_ms{-1};
    int64_t lost_duration_ms{0};

    nlohmann::json toJson() const
    {
        return {
            {"type", "localization_state"},
            {"timestamp_ms", timestamp_ms},
            {"map_id", map_id},
            {"status", status},
            {"confidence", confidence},
            {"pose_age_ms", pose_age_ms},
            {"lost_duration_ms", lost_duration_ms}
        };
    }
};

struct SlamHealth {
    int64_t timestamp_ms{0};
    bool slam_alive{false};
    bool lidar_alive{false};
    bool imu_alive{false};
    bool odom_alive{false};
    bool localization_alive{false};
    int64_t last_pose_age_ms{-1};
    std::string status{"failed"};

    nlohmann::json toJson() const
    {
        return {
            {"type", "slam_health"},
            {"timestamp_ms", timestamp_ms},
            {"slam_alive", slam_alive},
            {"lidar_alive", lidar_alive},
            {"imu_alive", imu_alive},
            {"odom_alive", odom_alive},
            {"localization_alive", localization_alive},
            {"last_pose_age_ms", last_pose_age_ms},
            {"status", status}
        };
    }
};

struct NavigationTaskState {
    int64_t timestamp_ms{0};
    std::string task_id{""};
    std::string target_node{""};
    PoseData target_pose;
    std::string state{"idle"};  // idle/accepted/running/paused/arrived/failed/cancelled/timeout
    double distance_to_goal_m{-1.0};
    bool is_arrived{false};
    std::string failure_reason{""};
    std::string last_service_reply{""};
    int32_t last_status_code{0};

    nlohmann::json toJson() const
    {
        return {
            {"type", "navigation_task_state"},
            {"timestamp_ms", timestamp_ms},
            {"task_id", task_id},
            {"target_node", target_node},
            {"target_pose", target_pose.toJson()},
            {"state", state},
            {"distance_to_goal_m", distance_to_goal_m},
            {"is_arrived", is_arrived},
            {"failure_reason", failure_reason},
            {"last_status_code", last_status_code},
            {"last_service_reply", last_service_reply}
        };
    }
};

struct LocalObstacleSummary {
    int64_t timestamp_ms{0};
    std::string frame_id{"base_link"};
    std::string source{"manual_stub"};
    double range_m{6.0};
    double front_clearance_m{6.0};
    double left_clearance_m{6.0};
    double right_clearance_m{6.0};
    double rear_clearance_m{6.0};
    double body_front_clearance_m{-1.0};
    double body_left_clearance_m{-1.0};
    double body_right_clearance_m{-1.0};
    double body_rear_clearance_m{-1.0};
    double low_hazard_front_clearance_m{-1.0};
    double low_hazard_left_clearance_m{-1.0};
    double low_hazard_right_clearance_m{-1.0};
    double low_hazard_rear_clearance_m{-1.0};
    double confidence{0.0};
    double front_confidence{0.0};
    double left_confidence{0.0};
    double right_confidence{0.0};
    double body_front_confidence{0.0};
    double body_left_confidence{0.0};
    double body_right_confidence{0.0};
    double body_rear_confidence{0.0};
    double low_hazard_front_confidence{0.0};
    double low_hazard_left_confidence{0.0};
    double low_hazard_right_confidence{0.0};
    double low_hazard_rear_confidence{0.0};
    int64_t age_ms{-1};
    bool stale{true};
    std::vector<std::string> blocked_directions;
    std::vector<std::string> low_hazard_directions;
    bool narrow_passage{false};
    std::string recommended_action{"normal"};

    nlohmann::json toJson() const
    {
        const auto optional_clearance = [](double value) -> nlohmann::json {
            return std::isfinite(value) && value >= 0.0 ? nlohmann::json(value) : nlohmann::json(nullptr);
        };
        return {
            {"type", "local_obstacle_summary"},
            {"schema_version", 2},
            {"timestamp_ms", timestamp_ms},
            {"frame_id", frame_id},
            {"source", source},
            {"range_m", range_m},
            {"front_clearance_m", front_clearance_m},
            {"left_clearance_m", left_clearance_m},
            {"right_clearance_m", right_clearance_m},
            {"rear_clearance_m", rear_clearance_m},
            {"body_clearance_m", {
                {"front", optional_clearance(body_front_clearance_m)},
                {"left", optional_clearance(body_left_clearance_m)},
                {"right", optional_clearance(body_right_clearance_m)},
                {"rear", optional_clearance(body_rear_clearance_m)}
            }},
            {"low_hazard_clearance_m", {
                {"front", optional_clearance(low_hazard_front_clearance_m)},
                {"left", optional_clearance(low_hazard_left_clearance_m)},
                {"right", optional_clearance(low_hazard_right_clearance_m)},
                {"rear", optional_clearance(low_hazard_rear_clearance_m)}
            }},
            {"confidence", confidence},
            {"front_confidence", front_confidence},
            {"left_confidence", left_confidence},
            {"right_confidence", right_confidence},
            {"body_roi_confidence", {
                {"front", body_front_confidence},
                {"left", body_left_confidence},
                {"right", body_right_confidence},
                {"rear", body_rear_confidence}
            }},
            {"low_hazard_roi_confidence", {
                {"front", low_hazard_front_confidence},
                {"left", low_hazard_left_confidence},
                {"right", low_hazard_right_confidence},
                {"rear", low_hazard_rear_confidence}
            }},
            {"age_ms", age_ms},
            {"stale", stale},
            {"blocked_directions", blocked_directions},
            {"low_hazard_directions", low_hazard_directions},
            {"narrow_passage", narrow_passage},
            {"recommended_action", recommended_action}
        };
    }
};

struct SafetyDecision {
    bool allow_navigation{false};
    bool should_pause{false};
    std::string recommended_mode{"stop"};
    std::string reason{"uninitialized"};

    nlohmann::json toJson() const
    {
        return {
            {"allow_navigation", allow_navigation},
            {"should_pause", should_pause},
            {"recommended_mode", recommended_mode},
            {"reason", reason}
        };
    }
};

}  // namespace slam_gateway
