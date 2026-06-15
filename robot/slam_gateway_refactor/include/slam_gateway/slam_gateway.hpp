#pragma once

#include <atomic>
#include <future>
#include <mutex>
#include <string>
#include <thread>

#include <unitree/robot/client/client.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>
#include <unitree/idl/ros2/String_.hpp>
#include <json.hpp>

#include "slam_gateway/models.hpp"
#include "slam_gateway/topology_manager.hpp"
#include "slam_gateway/lidar_geometry_perception.hpp"
#include "slam_gateway/safety_supervisor.hpp"

namespace slam_gateway {

constexpr const char* SLAM_INFO_TOPIC = "rt/slam_info";
constexpr const char* SLAM_KEY_INFO_TOPIC = "rt/slam_key_info";
constexpr const char* TEST_SERVICE_NAME = "slam_operate";
constexpr const char* TEST_API_VERSION = "1.0.0.1";

constexpr int32_t ROBOT_API_ID_STOP_NODE = 1901;
constexpr int32_t ROBOT_API_ID_START_MAPPING_PL = 1801;
constexpr int32_t ROBOT_API_ID_END_MAPPING_PL = 1802;
constexpr int32_t ROBOT_API_ID_START_RELOCATION_PL = 1804;
constexpr int32_t ROBOT_API_ID_POSE_NAV_PL = 1102;
constexpr int32_t ROBOT_API_ID_PAUSE_NAV = 1201;
constexpr int32_t ROBOT_API_ID_RESUME_NAV = 1202;

struct ServiceResult {
    int32_t status_code{0};
    std::string data;
    bool ok{false};
};

class SlamGateway : public unitree::robot::Client {
public:
    SlamGateway();
    ~SlamGateway();

    void Init() override;
    void initApis();

    ServiceResult startMapping(const std::string& slam_type = "indoor");
    ServiceResult endMapping(const std::string& map_path = "/home/unitree/test.pcd");
    ServiceResult startRelocation(const std::string& map_path = "/home/unitree/test.pcd",
                                  const PoseData& init_pose = PoseData{});
    ServiceResult submitNavigationGoal(const PoseData& goal);
    ServiceResult pauseNavigation();
    ServiceResult resumeNavigation();
    ServiceResult stopNode();

    void addCurrentPoseAsWaypoint(const std::string& name = "");
    void clearWaypoints();
    void printWaypoints() const;
    std::size_t waypointCount() const;
    bool saveWaypoints(const std::string& path) const;
    bool loadWaypoints(const std::string& path);

    void taskThreadRun(bool loop_patrol = true);
    void taskThreadStop();

    CurrentPose getCurrentPose() const;
    LocalizationState getLocalizationState() const;
    SlamHealth getSlamHealth() const;
    NavigationTaskState getNavigationTaskState() const;
    LocalObstacleSummary getLocalObstacleSummary() const;
    SafetyDecision getSafetyDecision() const;
    nlohmann::json buildWorldStateJson() const;

private:
    using StringMsg = std_msgs::msg::dds_::String_;
    unitree::robot::ChannelSubscriberPtr<StringMsg> sub_slam_info_;
    unitree::robot::ChannelSubscriberPtr<StringMsg> sub_slam_key_info_;

    void slamInfoHandler(const void* message);
    void slamKeyInfoHandler(const void* message);

    ServiceResult callApi(int32_t api_id, const std::string& parameter);
    void taskLoop(bool loop_patrol);
    void updateDistanceToGoalLocked();

private:
    mutable std::mutex state_mutex_;
    CurrentPose current_pose_;
    NavigationTaskState nav_state_;
    std::string last_slam_info_raw_;
    std::string last_slam_key_info_raw_;
    int64_t last_pose_update_ms_{0};
    int64_t last_pose_source_timestamp_ns_{0};
    int64_t localization_lost_since_ms_{0};

    TopologyManager topology_;
    LidarGeometryPerception lidar_perception_;
    SafetySupervisor safety_supervisor_;

    std::atomic<bool> is_arrived_{false};
    std::atomic<bool> thread_control_{false};
    std::thread task_thread_;
};

}  // namespace slam_gateway
