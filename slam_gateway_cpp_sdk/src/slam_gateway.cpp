#include "slam_gateway/slam_gateway.hpp"

#include <algorithm>
#include <chrono>
#include <iostream>
#include <thread>

namespace slam_gateway {

SlamGateway::SlamGateway()
    : unitree::robot::Client(TEST_SERVICE_NAME, false)
{
    sub_slam_info_ = unitree::robot::ChannelSubscriberPtr<StringMsg>(new unitree::robot::ChannelSubscriber<StringMsg>(SLAM_INFO_TOPIC));
    sub_slam_info_->InitChannel(std::bind(&SlamGateway::slamInfoHandler, this, std::placeholders::_1), 1);

    sub_slam_key_info_ = unitree::robot::ChannelSubscriberPtr<StringMsg>(new unitree::robot::ChannelSubscriber<StringMsg>(SLAM_KEY_INFO_TOPIC));
    sub_slam_key_info_->InitChannel(std::bind(&SlamGateway::slamKeyInfoHandler, this, std::placeholders::_1), 1);

    current_pose_.timestamp_ms = 0;
    nav_state_.timestamp_ms = nowMs();
    nav_state_.state = "idle";

    // Phase-1 placeholder. If real LiDAR perception is connected later,
    // replace this manual clearance with point-cloud-derived clearance.
    lidar_perception_.setManualClearance(6.0, 6.0, 6.0, 6.0);
}

SlamGateway::~SlamGateway()
{
    taskThreadStop();
    // Do not stop the SLAM backend just because this client process exits.
    // Shutdown is now explicit through stop_slam / stopNode().
}

void SlamGateway::Init()
{
    initApis();
}

void SlamGateway::initApis()
{
    SetApiVersion(TEST_API_VERSION);
    UT_ROBOT_CLIENT_REG_API_NO_PROI(ROBOT_API_ID_POSE_NAV_PL);
    UT_ROBOT_CLIENT_REG_API_NO_PROI(ROBOT_API_ID_PAUSE_NAV);
    UT_ROBOT_CLIENT_REG_API_NO_PROI(ROBOT_API_ID_RESUME_NAV);
    UT_ROBOT_CLIENT_REG_API_NO_PROI(ROBOT_API_ID_STOP_NODE);
    UT_ROBOT_CLIENT_REG_API_NO_PROI(ROBOT_API_ID_START_MAPPING_PL);
    UT_ROBOT_CLIENT_REG_API_NO_PROI(ROBOT_API_ID_END_MAPPING_PL);
    UT_ROBOT_CLIENT_REG_API_NO_PROI(ROBOT_API_ID_START_RELOCATION_PL);
}

ServiceResult SlamGateway::callApi(int32_t api_id, const std::string& parameter)
{
    ServiceResult r;
    r.status_code = Call(api_id, parameter, r.data);
    r.ok = (r.status_code == 0);
    std::cout << "statusCode:" << r.status_code << std::endl;
    std::cout << "data:" << r.data << std::endl;
    return r;
}

ServiceResult SlamGateway::startMapping(const std::string& slam_type)
{
    nlohmann::json j;
    j["data"]["slam_type"] = slam_type;
    return callApi(ROBOT_API_ID_START_MAPPING_PL, j.dump());
}

ServiceResult SlamGateway::endMapping(const std::string& map_path)
{
    nlohmann::json j;
    j["data"]["address"] = map_path;
    return callApi(ROBOT_API_ID_END_MAPPING_PL, j.dump());
}

ServiceResult SlamGateway::startRelocation(const std::string& map_path, const PoseData& init_pose)
{
    nlohmann::json j;
    j["data"]["x"] = init_pose.x;
    j["data"]["y"] = init_pose.y;
    j["data"]["z"] = init_pose.z;
    j["data"]["q_x"] = init_pose.q_x;
    j["data"]["q_y"] = init_pose.q_y;
    j["data"]["q_z"] = init_pose.q_z;
    j["data"]["q_w"] = init_pose.q_w;
    j["data"]["address"] = map_path;

    auto r = callApi(ROBOT_API_ID_START_RELOCATION_PL, j.dump(4));
    std::lock_guard<std::mutex> lk(state_mutex_);
    nav_state_.state = r.ok ? "idle" : "failed";
    nav_state_.failure_reason = r.ok ? "" : "relocation_call_failed";
    return r;
}

ServiceResult SlamGateway::submitNavigationGoal(const PoseData& goal)
{
    {
        std::lock_guard<std::mutex> lk(state_mutex_);
        nav_state_.timestamp_ms = nowMs();
        nav_state_.task_id = "nav_" + std::to_string(nav_state_.timestamp_ms);
        nav_state_.target_node = goal.name;
        nav_state_.target_pose = goal;
        nav_state_.state = "accepted";
        nav_state_.is_arrived = false;
        nav_state_.failure_reason.clear();
        updateDistanceToGoalLocked();
    }

    std::cout << "parameter:" << goal.toNavigationJson() << std::endl;
    auto r = callApi(ROBOT_API_ID_POSE_NAV_PL, goal.toNavigationJson());

    {
        std::lock_guard<std::mutex> lk(state_mutex_);
        nav_state_.timestamp_ms = nowMs();
        nav_state_.last_status_code = r.status_code;
        nav_state_.last_service_reply = r.data;
        if (r.ok) {
            nav_state_.state = "running";
        } else {
            nav_state_.state = "failed";
            nav_state_.failure_reason = "pose_nav_call_failed";
        }
    }
    return r;
}

ServiceResult SlamGateway::pauseNavigation()
{
    nlohmann::json j;
    j["data"] = nlohmann::json::object();
    auto r = callApi(ROBOT_API_ID_PAUSE_NAV, j.dump());
    std::lock_guard<std::mutex> lk(state_mutex_);
    nav_state_.state = r.ok ? "paused" : nav_state_.state;
    return r;
}

ServiceResult SlamGateway::resumeNavigation()
{
    nlohmann::json j;
    j["data"] = nlohmann::json::object();
    auto r = callApi(ROBOT_API_ID_RESUME_NAV, j.dump());
    std::lock_guard<std::mutex> lk(state_mutex_);
    nav_state_.state = r.ok ? "running" : nav_state_.state;
    return r;
}

ServiceResult SlamGateway::stopNode()
{
    nlohmann::json j;
    j["data"] = nlohmann::json::object();
    return callApi(ROBOT_API_ID_STOP_NODE, j.dump());
}

void SlamGateway::addCurrentPoseAsWaypoint()
{
    PoseData pose;
    {
        std::lock_guard<std::mutex> lk(state_mutex_);
        pose = current_pose_.pose;
        pose.mode = 0;  // 统一设置为绕障模式
    }

    topology_.addPose(pose);
    std::cout << "Add pose to task list: " << pose.summary() << std::endl;
    if (saveWaypoints(topology_.topology_points_path)) {
        std::cout << "Saved task list to " << topology_.topology_points_path << std::endl;
    } else {
        std::cout << "Failed to save task list to " << topology_.topology_points_path << std::endl;
    }
}

void SlamGateway::clearWaypoints()
{
    topology_.clear();
    if (saveWaypoints(topology_.topology_points_path)) {
        std::cout << "Clear task list and saved empty file: " << topology_.topology_points_path << std::endl;
    } else {
        std::cout << "Clear task list, but failed to save empty file: " << topology_.topology_points_path << std::endl;
    }
}

void SlamGateway::printWaypoints() const
{
    topology_.print();
}

std::size_t SlamGateway::waypointCount() const
{
    return topology_.size();
}

bool SlamGateway::saveWaypoints(const std::string& path) const
{
    const std::string save_path = path.empty() ? topology_.topology_points_path : path;
    return topology_.saveJson(save_path);
}

bool SlamGateway::loadWaypoints(const std::string& path)
{
    const std::string load_path = path.empty() ? topology_.topology_points_path : path;
    return topology_.loadJson(load_path);
}

void SlamGateway::taskThreadRun(bool loop_patrol)
{
    taskThreadStop();
    thread_control_.store(true);
    task_thread_ = std::thread(&SlamGateway::taskLoop, this, loop_patrol);
}

void SlamGateway::taskThreadStop()
{
    thread_control_.store(false);
    if (task_thread_.joinable()) {
        task_thread_.join();
    }
}

void SlamGateway::taskLoop(bool loop_patrol)
{
    const std::string path = topology_.topology_points_path;
    if (!loadWaypoints(path)) {
        topology_.clear();
        std::cout << "No valid task list file found at " << path << std::endl;
        std::cout << "Please execute Add pose to task list first." << std::endl;
        thread_control_.store(false);
        return;
    }

    std::vector<PoseData> poses = topology_.list();
    std::cout << "Loaded task list from " << path << std::endl;
    std::cout << "task list num:" << poses.size() << std::endl;
    if (poses.empty()) {
        std::cout << "No waypoint in task list. Please execute Add pose to task list first." << std::endl;
        thread_control_.store(false);
        return;
    }

    while (thread_control_.load()) {
        for (std::size_t i = 0; i < poses.size() && thread_control_.load(); ++i) {
            auto safety = getSafetyDecision();
            if (!safety.allow_navigation) {
                std::cout << "Safety blocked navigation: " << safety.reason << std::endl;
                pauseNavigation();
                thread_control_.store(false);
                return;
            }

            is_arrived_.store(false);
            submitNavigationGoal(poses[i]);

            const auto start = std::chrono::steady_clock::now();
            while (!is_arrived_.load() && thread_control_.load()) {
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
                {
                    std::lock_guard<std::mutex> lk(state_mutex_);
                    updateDistanceToGoalLocked();
                }

                const double elapsed = std::chrono::duration_cast<std::chrono::seconds>(
                    std::chrono::steady_clock::now() - start).count();
                if (elapsed > 60.0) {
                    std::lock_guard<std::mutex> lk(state_mutex_);
                    nav_state_.state = "timeout";
                    nav_state_.failure_reason = "navigation_timeout";
                    std::cout << "Navigation timeout on waypoint: " << poses[i].name << std::endl;
                    break;
                }
            }
        }

        if (!loop_patrol || !thread_control_.load()) break;
        std::reverse(poses.begin(), poses.end());
    }
}

CurrentPose SlamGateway::getCurrentPose() const
{
    std::lock_guard<std::mutex> lk(state_mutex_);
    return current_pose_;
}

LocalizationState SlamGateway::getLocalizationState() const
{
    LocalizationState s;
    s.timestamp_ms = nowMs();
    s.map_id = current_pose_.map_id;
    if (last_pose_update_ms_ <= 0) {
        s.status = "not_started";
        s.pose_age_ms = -1;
        s.confidence = 0.0;
        return s;
    }

    s.pose_age_ms = s.timestamp_ms - last_pose_update_ms_;
    if (s.pose_age_ms <= 500) {
        s.status = "localized";
        s.confidence = 0.9;
        s.lost_duration_ms = 0;
    } else if (s.pose_age_ms <= 2000) {
        s.status = "degraded";
        s.confidence = 0.5;
        s.lost_duration_ms = 0;
    } else {
        s.status = "lost";
        s.confidence = 0.0;
        s.lost_duration_ms = localization_lost_since_ms_ > 0 ? (s.timestamp_ms - localization_lost_since_ms_) : s.pose_age_ms;
    }
    return s;
}

SlamHealth SlamGateway::getSlamHealth() const
{
    SlamHealth h;
    h.timestamp_ms = nowMs();
    if (last_pose_update_ms_ <= 0) {
        h.status = "failed";
        h.slam_alive = false;
        h.localization_alive = false;
        return h;
    }

    h.last_pose_age_ms = h.timestamp_ms - last_pose_update_ms_;
    h.slam_alive = h.last_pose_age_ms < 5000;
    h.localization_alive = h.last_pose_age_ms < 2000;
    // Phase-1 assumption: LiDAR/IMU/odom health should be wired from real topics later.
    h.lidar_alive = true;
    h.imu_alive = true;
    h.odom_alive = true;

    if (h.last_pose_age_ms <= 500) h.status = "ok";
    else if (h.last_pose_age_ms <= 2000) h.status = "degraded";
    else h.status = "failed";

    return h;
}

NavigationTaskState SlamGateway::getNavigationTaskState() const
{
    std::lock_guard<std::mutex> lk(state_mutex_);
    auto s = nav_state_;
    s.timestamp_ms = nowMs();
    return s;
}

LocalObstacleSummary SlamGateway::getLocalObstacleSummary() const
{
    return lidar_perception_.getLocalObstacleSummary();
}

SafetyDecision SlamGateway::getSafetyDecision() const
{
    return safety_supervisor_.evaluate(getSlamHealth(), getLocalizationState(), getLocalObstacleSummary());
}

nlohmann::json SlamGateway::buildWorldStateJson() const
{
    auto pose = getCurrentPose();
    auto loc = getLocalizationState();
    auto health = getSlamHealth();
    auto nav = getNavigationTaskState();
    auto obstacle = getLocalObstacleSummary();
    auto safety = safety_supervisor_.evaluate(health, loc, obstacle);

    nlohmann::json j;
    j["type"] = "world_state";
    j["timestamp_ms"] = nowMs();
    j["current_pose"] = pose.toJson();
    j["localization"] = loc.toJson();
    j["slam_health"] = health.toJson();
    j["navigation"] = nav.toJson();
    j["local_obstacle"] = obstacle.toJson();
    j["safety"] = safety.toJson();
    return j;
}

void SlamGateway::slamInfoHandler(const void* message)
{
    StringMsg currentMsg = *(StringMsg*)message;
    const std::string raw = currentMsg.data();

    try {
        auto jsonData = nlohmann::json::parse(raw);
        if (jsonData.value("errorCode", 0) != 0) {
            std::cout << "\033[33m" << jsonData.value("info", "slam_info error") << "\033[0m" << std::endl;
            return;
        }

        if (jsonData.value("type", "") == "pos_info") {
            CurrentPose p;
            p.timestamp_ms = nowMs();
            const auto& cp = jsonData["data"]["currentPose"];
            p.pose.x = cp.value("x", 0.0f);
            p.pose.y = cp.value("y", 0.0f);
            p.pose.z = cp.value("z", 0.0f);
            p.pose.q_x = cp.value("q_x", 0.0f);
            p.pose.q_y = cp.value("q_y", 0.0f);
            p.pose.q_z = cp.value("q_z", 0.0f);
            p.pose.q_w = cp.value("q_w", 1.0f);

            std::lock_guard<std::mutex> lk(state_mutex_);
            current_pose_ = p;
            last_pose_update_ms_ = p.timestamp_ms;
            localization_lost_since_ms_ = 0;
            last_slam_info_raw_ = raw;
            updateDistanceToGoalLocked();
        }
    } catch (const std::exception& e) {
        std::cout << "slam_info parse failed: " << e.what() << " raw:" << raw << std::endl;
    }
}

void SlamGateway::slamKeyInfoHandler(const void* message)
{
    StringMsg currentMsg = *(StringMsg*)message;
    const std::string raw = currentMsg.data();
    std::cout << "slam_key_info raw: " << raw << std::endl;

    try {
        auto jsonData = nlohmann::json::parse(raw);
        if (jsonData.value("errorCode", 0) != 0) {
            std::cout << "\033[33m" << jsonData.value("info", "slam_key_info error") << "\033[0m" << std::endl;
            std::lock_guard<std::mutex> lk(state_mutex_);
            nav_state_.state = "failed";
            nav_state_.failure_reason = jsonData.value("info", "slam_key_info error");
            return;
        }

        std::lock_guard<std::mutex> lk(state_mutex_);
        last_slam_key_info_raw_ = raw;
        nav_state_.timestamp_ms = nowMs();

        const std::string type = jsonData.value("type", "");
        if (type == "task_result") {
            const bool arrived = jsonData["data"].value("is_arrived", false);
            is_arrived_.store(arrived);
            nav_state_.is_arrived = arrived;
            nav_state_.target_node = jsonData["data"].value("targetNodeName", nav_state_.target_node);
            nav_state_.state = arrived ? "arrived" : "failed";
            if (!arrived) nav_state_.failure_reason = "task_result_not_arrived";

            if (arrived) {
                std::cout << "I arrived " << nav_state_.target_node << std::endl;
            } else {
                std::cout << "I not arrived " << nav_state_.target_node << " Please help me!!" << std::endl;
            }
        } else {
            // Keep full raw message for later analysis. Official binary may publish extra states.
            nav_state_.last_service_reply = raw;
        }
    } catch (const std::exception& e) {
        std::cout << "slam_key_info parse failed: " << e.what() << " raw:" << raw << std::endl;
    }
}

void SlamGateway::updateDistanceToGoalLocked()
{
    const auto dx = static_cast<double>(nav_state_.target_pose.x - current_pose_.pose.x);
    const auto dy = static_cast<double>(nav_state_.target_pose.y - current_pose_.pose.y);
    nav_state_.distance_to_goal_m = std::sqrt(dx * dx + dy * dy);
}

}  // namespace slam_gateway
