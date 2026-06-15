#include "slam_gateway/slam_gateway.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <thread>

namespace slam_gateway {
namespace {

std::string lidarGeometrySummaryPath()
{
    const char* configured = std::getenv("GO2W_LIDAR_GEOMETRY_SUMMARY_PATH");
    return configured && *configured
        ? configured
        : "/home/unitree/Go2W_SLAM_AI/artifacts/lidar_geometry_summary.json";
}

int64_t lidarGeometrySummaryMaxAgeMs()
{
    const char* configured = std::getenv("GO2W_LIDAR_GEOMETRY_STALE_MS");
    if (!configured || !*configured) return 1000;
    try {
        return std::max<int64_t>(1, std::stoll(configured));
    } catch (...) {
        return 1000;
    }
}

std::string stereoSummaryPath()
{
    const char* configured = std::getenv("GO2W_STEREO_SUMMARY_PATH");
    return configured && *configured
        ? configured
        : "/home/unitree/Go2W_SLAM_AI/artifacts/stereo_depth_summary.json";
}

int64_t stereoSummaryMaxAgeMs()
{
    const char* configured = std::getenv("GO2W_STEREO_SAFETY_STALE_MS");
    if (!configured || !*configured) return 1000;
    try {
        return std::max<int64_t>(1, std::stoll(configured));
    } catch (...) {
        return 1000;
    }
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
    stopSupervisedReposition();
    taskThreadStop();
    // Do not stop the SLAM backend from short-lived status/query clients.
    // Use the explicit stop_slam command when the backend really should stop.
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
    sport_client_.SetTimeout(2.0f);
    sport_client_.Init();
    sport_initialized_ = true;
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

ServiceResult SlamGateway::submitSupervisedReposition(
    const std::string& direction,
    double distance_m,
    double speed_mps)
{
    stopSupervisedReposition();
    if (!sport_initialized_) {
        return {
            -1,
            "sport client is not initialized",
            false
        };
    }
    const CurrentPose start_pose = getCurrentPose();
    const double yaw = quaternionToYaw(
        start_pose.pose.q_x,
        start_pose.pose.q_y,
        start_pose.pose.q_z,
        start_pose.pose.q_w);
    PoseData target = start_pose.pose;
    target.name = "__supervised_reposition_" + direction + "__";
    target.mode = 0;
    target.speed = static_cast<float>(speed_mps);
    offsetPose(target, direction, yaw, distance_m);

    {
        std::lock_guard<std::mutex> lk(state_mutex_);
        nav_state_.timestamp_ms = nowMs();
        nav_state_.task_id = "reposition_" + std::to_string(nav_state_.timestamp_ms);
        nav_state_.target_node = target.name;
        nav_state_.target_pose = target;
        nav_state_.state = "running";
        nav_state_.is_arrived = false;
        nav_state_.failure_reason.clear();
        nav_state_.last_status_code = 0;
        nav_state_.last_service_reply = "supervised sport translation accepted";
        updateDistanceToGoalLocked();
    }

    reposition_stop_requested_.store(false);
    reposition_active_.store(true);
    reposition_thread_ = std::thread(
        &SlamGateway::supervisedRepositionLoop,
        this,
        direction,
        distance_m,
        speed_mps,
        start_pose);

    ServiceResult result;
    result.ok = true;
    result.status_code = 0;
    result.data = nlohmann::json(
        {
            {"controller", "go2_sport_move"},
            {"direction", direction},
            {"distance_m", distance_m},
            {"speed_mps", speed_mps},
            {"yaw_rate_rps", 0.0}
        }).dump();
    return result;
}

void SlamGateway::supervisedRepositionLoop(
    std::string direction,
    double distance_m,
    double speed_mps,
    CurrentPose start_pose)
{
    const double start_x = start_pose.pose.x;
    const double start_y = start_pose.pose.y;
    const double max_duration_s = distance_m / speed_mps + 2.0;
    const auto started = std::chrono::steady_clock::now();
    std::string failure_reason;
    bool arrived = false;

    while (!reposition_stop_requested_.load()) {
        const auto safety = getSafetyDecision();
        const auto obstacle = getLocalObstacleSummary();
        if (!safety.allow_navigation) {
            failure_reason = "reposition_safety_blocked:" + safety.reason;
            break;
        }
        if (directionalClearance(obstacle, direction) <
            obstacle_policy::kRepositionReserveM) {
            failure_reason = "reposition_directional_clearance_exhausted";
            break;
        }

        const auto current = getCurrentPose();
        const double travelled_m = std::hypot(
            static_cast<double>(current.pose.x) - start_x,
            static_cast<double>(current.pose.y) - start_y);
        if (travelled_m >= std::max(0.0, distance_m - 0.03)) {
            arrived = true;
            break;
        }
        const double elapsed_s = std::chrono::duration<double>(
            std::chrono::steady_clock::now() - started).count();
        if (elapsed_s > max_duration_s) {
            failure_reason = "reposition_distance_not_reached_before_timeout";
            break;
        }

        float vx = 0.0f;
        float vy = 0.0f;
        if (direction == "forward") vx = static_cast<float>(speed_mps);
        if (direction == "backward") vx = static_cast<float>(-speed_mps);
        if (direction == "left") vy = static_cast<float>(speed_mps);
        if (direction == "right") vy = static_cast<float>(-speed_mps);
        int32_t status = 0;
        {
            std::lock_guard<std::mutex> lock(sport_mutex_);
            status = sport_client_.Move(vx, vy, 0.0f);
        }
        if (status < 0) {
            failure_reason = "sport_move_rejected:" + std::to_string(status);
            break;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    {
        if (sport_initialized_) {
            std::lock_guard<std::mutex> lock(sport_mutex_);
            sport_client_.StopMove();
        }
    }
    reposition_active_.store(false);
    {
        std::lock_guard<std::mutex> lk(state_mutex_);
        updateDistanceToGoalLocked();
        if (arrived) {
            nav_state_.state = "arrived";
            nav_state_.is_arrived = true;
            nav_state_.failure_reason.clear();
        } else if (!reposition_stop_requested_.load()) {
            nav_state_.state = "failed";
            nav_state_.failure_reason =
                failure_reason.empty()
                    ? "supervised_reposition_stopped"
                    : failure_reason;
        }
    }
}

int32_t SlamGateway::stopSupervisedReposition()
{
    reposition_stop_requested_.store(true);
    int32_t status = 0;
    if (sport_initialized_) {
        std::lock_guard<std::mutex> lock(sport_mutex_);
        status = sport_client_.StopMove();
    }
    if (reposition_thread_.joinable() &&
        reposition_thread_.get_id() != std::this_thread::get_id()) {
        reposition_thread_.join();
    }
    reposition_active_.store(false);
    return status;
}

ServiceResult SlamGateway::pauseNavigation()
{
    bool reposition_task = false;
    {
        std::lock_guard<std::mutex> lk(state_mutex_);
        reposition_task =
            nav_state_.target_node.rfind("__supervised_reposition_", 0) == 0;
    }
    if (reposition_task || reposition_active_.load()) {
        ServiceResult result;
        result.status_code = stopSupervisedReposition();
        result.ok = result.status_code >= 0;
        result.data = nlohmann::json(
            {
                {"controller", "go2_sport_move"},
                {"stopped", result.ok}
            }).dump();
        std::lock_guard<std::mutex> lk(state_mutex_);
        nav_state_.state = result.ok ? "paused" : nav_state_.state;
        return result;
    }
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

void SlamGateway::addCurrentPoseAsWaypoint(const std::string& name)
{
    const auto localization = getLocalizationState();
    if (localization.status != "localized" || localization.pose_age_ms < 0 || localization.pose_age_ms > 500) {
        std::cout << "Reject waypoint capture: localization is not fresh." << std::endl;
        return;
    }

    PoseData pose;
    {
        std::lock_guard<std::mutex> lk(state_mutex_);
        pose = current_pose_.pose;
        pose.mode = 0;  // Default obstacle-avoidance mode: 0=avoid, 1=stop.
    }

    topology_.addPose(pose, name);
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
            const auto navigation_result = submitNavigationGoal(poses[i]);
            if (!navigation_result.ok) {
                std::cout << "Navigation request rejected for waypoint: " << poses[i].name << std::endl;
                const auto pause_result = pauseNavigation();
                std::cout << "Pause after rejected navigation accepted="
                          << (pause_result.ok ? "true" : "false") << std::endl;
                thread_control_.store(false);
                return;
            }

            const auto start = std::chrono::steady_clock::now();
            while (!is_arrived_.load() && thread_control_.load()) {
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
                auto runtime_safety = getSafetyDecision();
                if (!runtime_safety.allow_navigation) {
                    std::cout << "Runtime safety blocked navigation: " << runtime_safety.reason << std::endl;
                    pauseNavigation();
                    thread_control_.store(false);
                    return;
                }
                {
                    std::lock_guard<std::mutex> lk(state_mutex_);
                    updateDistanceToGoalLocked();
                }

                const double elapsed = std::chrono::duration_cast<std::chrono::seconds>(
                    std::chrono::steady_clock::now() - start).count();
                if (elapsed > 60.0) {
                    {
                        std::lock_guard<std::mutex> lk(state_mutex_);
                        nav_state_.state = "timeout";
                        nav_state_.failure_reason = "navigation_timeout";
                    }
                    std::cout << "Navigation timeout on waypoint: " << poses[i].name << std::endl;
                    const auto pause_result = pauseNavigation();
                    std::cout << "Pause after timeout accepted="
                              << (pause_result.ok ? "true" : "false") << std::endl;
                    thread_control_.store(false);
                    return;
                }
            }

            if (is_arrived_.load()) {
                const auto pause_result = pauseNavigation();
                if (!pause_result.ok) {
                    std::cout << "Arrival confirmed but pause was rejected." << std::endl;
                    thread_control_.store(false);
                    return;
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
    std::lock_guard<std::mutex> lk(state_mutex_);
    LocalizationState s;
    s.timestamp_ms = nowMs();
    s.map_id = current_pose_.map_id;
    s.map_path = current_pose_.map_path;
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
    std::lock_guard<std::mutex> lk(state_mutex_);
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
    // The pose stream proves the SLAM/localization chain is updating, but it
    // does not independently prove each upstream sensor is alive.
    h.direct_sensor_health_observed = false;
    h.sensor_health_source = "derived_from_slam_pose_only";

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
    return lidar_perception_.getFusedSummaryOrFallback(
        lidarGeometrySummaryPath(),
        lidarGeometrySummaryMaxAgeMs(),
        stereoSummaryPath(),
        stereoSummaryMaxAgeMs());
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
            if (!jsonData.contains("data") || !jsonData["data"].is_object() ||
                !jsonData["data"].contains("currentPose") || !jsonData["data"]["currentPose"].is_object()) {
                std::cout << "slam_info rejected: missing data.currentPose" << std::endl;
                return;
            }
            const auto& cp = jsonData["data"]["currentPose"];
            const auto finite_number = [&](const char* key) {
                return cp.contains(key) && cp.at(key).is_number() &&
                    std::isfinite(cp.at(key).get<double>());
            };
            for (const char* key : {"x", "y", "z", "q_x", "q_y", "q_z", "q_w"}) {
                if (!finite_number(key)) {
                    std::cout << "slam_info rejected: invalid currentPose." << key << std::endl;
                    return;
                }
            }
            const double qx = cp.at("q_x").get<double>();
            const double qy = cp.at("q_y").get<double>();
            const double qz = cp.at("q_z").get<double>();
            const double qw = cp.at("q_w").get<double>();
            const double quaternion_norm = std::sqrt(qx * qx + qy * qy + qz * qz + qw * qw);
            if (!std::isfinite(quaternion_norm) || quaternion_norm < 0.5 || quaternion_norm > 1.5) {
                std::cout << "slam_info rejected: invalid quaternion norm" << std::endl;
                return;
            }
            if (!jsonData.contains("sec") || !jsonData.at("sec").is_number_integer() ||
                !jsonData.contains("nanosec") || !jsonData.at("nanosec").is_number_integer()) {
                std::cout << "slam_info rejected: missing source timestamp" << std::endl;
                return;
            }
            const int64_t sec = jsonData.at("sec").get<int64_t>();
            const int64_t nanosec = jsonData.at("nanosec").get<int64_t>();
            if (sec < 0 || nanosec < 0 || nanosec >= 1000000000LL) {
                std::cout << "slam_info rejected: invalid source timestamp" << std::endl;
                return;
            }
            const int64_t source_timestamp_ns = sec * 1000000000LL + nanosec;

            CurrentPose p;
            p.timestamp_ms = nowMs();
            const auto& data = jsonData["data"];
            p.map_id = data.value("pcdName", "");
            p.map_path = data.value("address", "");
            if (p.map_id.empty() && !p.map_path.empty()) {
                const auto slash = p.map_path.find_last_of('/');
                const auto dot = p.map_path.find_last_of('.');
                const auto begin = slash == std::string::npos ? 0 : slash + 1;
                const auto end = dot == std::string::npos || dot < begin ? p.map_path.size() : dot;
                p.map_id = p.map_path.substr(begin, end - begin);
            }
            p.pose.x = cp.at("x").get<float>();
            p.pose.y = cp.at("y").get<float>();
            p.pose.z = cp.at("z").get<float>();
            p.pose.q_x = static_cast<float>(qx);
            p.pose.q_y = static_cast<float>(qy);
            p.pose.q_z = static_cast<float>(qz);
            p.pose.q_w = static_cast<float>(qw);

            std::lock_guard<std::mutex> lk(state_mutex_);
            if (source_timestamp_ns <= last_pose_source_timestamp_ns_) {
                std::cout << "slam_info rejected: source timestamp did not advance" << std::endl;
                return;
            }
            current_pose_ = p;
            last_pose_update_ms_ = p.timestamp_ms;
            last_pose_source_timestamp_ns_ = source_timestamp_ns;
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
