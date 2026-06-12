#include <atomic>
#include <chrono>
#include <cstdint>
#include <iomanip>
#include <iostream>
#include <mutex>
#include <random>
#include <sstream>
#include <string>
#include <thread>

#include <unitree/robot/channel/channel_factory.hpp>
#include <json.hpp>

#include "slam_gateway/llm_command_processor.hpp"
#include "slam_gateway/navigation_target_authorizer.hpp"
#include "slam_gateway/slam_gateway.hpp"

namespace {

constexpr int64_t kNavigationLeaseTimeoutMs = 2000;
constexpr int64_t kNavigationHeartbeatIntervalMs = 500;
constexpr int64_t kNavigationSessionStartupTimeoutMs = 5000;
constexpr const char* kDefaultMapRegistry =
    "/home/unitree/Go2W_SLAM_AI/configs/maps/go2w_real_site_map_registry.json";
constexpr const char* kDefaultRegistryMapId = "go2w_real_site";

int64_t monotonicNowMs()
{
    using namespace std::chrono;
    return duration_cast<milliseconds>(steady_clock::now().time_since_epoch()).count();
}

std::string makeSessionToken()
{
    std::random_device random;
    std::ostringstream token;
    token << std::hex << std::setfill('0');
    for (int i = 0; i < 4; ++i) {
        token << std::setw(8) << random();
    }
    return token.str();
}

class NavigationPauseClient : public unitree::robot::Client {
public:
    NavigationPauseClient()
        : unitree::robot::Client(slam_gateway::TEST_SERVICE_NAME, false)
    {
    }

    void Init() override
    {
        SetApiVersion(slam_gateway::TEST_API_VERSION);
        UT_ROBOT_CLIENT_REG_API_NO_PROI(slam_gateway::ROBOT_API_ID_PAUSE_NAV);
        SetTimeout(2.0f);
    }

    slam_gateway::ServiceResult pause()
    {
        nlohmann::json request;
        request["data"] = nlohmann::json::object();
        slam_gateway::ServiceResult result;
        result.status_code =
            Call(slam_gateway::ROBOT_API_ID_PAUSE_NAV, request.dump(), result.data);
        result.ok = result.status_code == 0;
        return result;
    }
};

struct NavigationLease {
    std::mutex mutex;
    bool active{false};
    bool invalid{false};
    uint64_t generation{0};
    int64_t deadline_ms{0};
    std::string invalid_reason;
};

struct PauseOutcome {
    slam_gateway::ServiceResult result;
    int attempts{0};
};

PauseOutcome pauseWithRetries(
    NavigationPauseClient& client,
    std::mutex& client_mutex,
    int max_attempts = 3)
{
    PauseOutcome outcome;
    std::lock_guard<std::mutex> lock(client_mutex);
    for (int attempt = 1; attempt <= max_attempts; ++attempt) {
        outcome.attempts = attempt;
        outcome.result = client.pause();
        if (outcome.result.ok) break;
        if (attempt < max_attempts) {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
    }
    return outcome;
}

std::string runtimeSessionBlockReason(
    const slam_gateway::SlamGateway& gateway,
    const std::string& session_map_id,
    const std::string& session_map_path)
{
    const auto pose = gateway.getCurrentPose();
    const auto localization = gateway.getLocalizationState();
    if (pose.map_id != session_map_id || pose.map_path != session_map_path) {
        return "navigation_session_map_identity_changed";
    }
    if (localization.status != "localized") {
        return "navigation_session_localization_invalid";
    }
    if (localization.pose_age_ms < 0 || localization.pose_age_ms > 2000) {
        return "navigation_session_localization_stale";
    }
    const auto safety = gateway.getSafetyDecision();
    if (!safety.allow_navigation) {
        return "navigation_session_safety_blocked:" + safety.reason;
    }
    return "";
}

void writeJsonLine(const nlohmann::json& value, std::mutex& output_mutex)
{
    std::lock_guard<std::mutex> lock(output_mutex);
    std::cout << value.dump() << std::endl;
}

}  // namespace

int main(int argc, const char** argv)
{
    if (argc < 2) {
        std::cout << "Usage: " << argv[0]
                  << " networkInterface [--persistent-navigation-session|--persistent-world-state-session]"
                  << std::endl;
        return -1;
    }

    const bool persistent_navigation_session =
        argc >= 3 && std::string(argv[2]) == "--persistent-navigation-session";
    const bool persistent_world_state_session =
        argc >= 3 && std::string(argv[2]) == "--persistent-world-state-session";
    const bool persistent_session =
        persistent_navigation_session || persistent_world_state_session;
    const std::string session_token =
        persistent_navigation_session ? makeSessionToken() : "";

    unitree::robot::ChannelFactory::Instance()->Init(0, argv[1]);
    slam_gateway::SlamGateway gateway;
    gateway.initApis();
    gateway.SetTimeout(10.0f);
    slam_gateway::NavigationTargetAuthorizer target_authorizer(
        kDefaultMapRegistry,
        kDefaultRegistryMapId);
    NavigationPauseClient pause_client;
    pause_client.Init();
    std::mutex pause_client_mutex;

    std::atomic<bool> monitor_running{persistent_navigation_session};
    std::mutex gateway_mutex;
    std::mutex output_mutex;
    NavigationLease lease;
    std::thread lease_monitor;
    std::string session_map_id;
    std::string session_map_path;

    slam_gateway::LlmCommandProcessor processor(
        gateway,
        session_token,
        &target_authorizer,
        [&]() -> std::string {
            if (!persistent_navigation_session) return "persistent_navigation_session_required";
            std::lock_guard<std::mutex> lock(lease.mutex);
            if (lease.invalid) return lease.invalid_reason;
            if (!lease.active) return "navigation_lease_not_active";
            if (monotonicNowMs() > lease.deadline_ms) return "navigation_heartbeat_timeout";
            return "";
        });

    if (persistent_navigation_session) {
        if (!target_authorizer.ready()) {
            writeJsonLine(
                {
                    {"type", "navigation_session_unready"},
                    {"reason", target_authorizer.loadError()},
                    {"registry_path", target_authorizer.registryPath()}
                },
                output_mutex);
            return 2;
        }
        const int64_t startup_deadline_ms =
            monotonicNowMs() + kNavigationSessionStartupTimeoutMs;
        slam_gateway::CurrentPose startup_pose;
        slam_gateway::LocalizationState startup_localization;
        while (monotonicNowMs() < startup_deadline_ms) {
            startup_pose = gateway.getCurrentPose();
            startup_localization = gateway.getLocalizationState();
            if (!startup_pose.map_id.empty() &&
                !startup_pose.map_path.empty() &&
                startup_localization.status == "localized" &&
                startup_localization.pose_age_ms >= 0 &&
                startup_localization.pose_age_ms <= 500) {
                break;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
        }
        if (startup_pose.map_id.empty() ||
            startup_pose.map_path.empty() ||
            startup_localization.status != "localized" ||
            startup_localization.pose_age_ms < 0 ||
            startup_localization.pose_age_ms > 500) {
            writeJsonLine(
                {
                    {"type", "navigation_session_unready"},
                    {"reason", "fresh_localization_with_map_identity_required"},
                    {"current_pose", startup_pose.toJson()},
                    {"localization", startup_localization.toJson()}
                },
                output_mutex);
            return 2;
        }
        session_map_id = startup_pose.map_id;
        session_map_path = startup_pose.map_path;
        writeJsonLine(
            {
                {"type", "navigation_session_ready"},
                {"session_token", session_token},
                {"heartbeat_interval_ms", kNavigationHeartbeatIntervalMs},
                {"lease_timeout_ms", kNavigationLeaseTimeoutMs},
                {"map_id", startup_pose.map_id},
                {"map_path", startup_pose.map_path},
                {"registry_map_id", target_authorizer.registryMapId()},
                {"registry_path", target_authorizer.registryPath()}
            },
            output_mutex);
        lease_monitor = std::thread([&]() {
            while (monitor_running.load()) {
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
                {
                    std::lock_guard<std::mutex> lock(lease.mutex);
                    if (!lease.active || lease.invalid) continue;
                }

                const int64_t now_ms = monotonicNowMs();
                std::string reason =
                    runtimeSessionBlockReason(gateway, session_map_id, session_map_path);
                uint64_t generation = 0;
                int64_t heartbeat_age_ms = 0;
                {
                    std::lock_guard<std::mutex> lock(lease.mutex);
                    if (!lease.active || lease.invalid) continue;
                    generation = lease.generation;
                    if (reason.empty() && now_ms > lease.deadline_ms) {
                        reason = "navigation_heartbeat_timeout";
                        heartbeat_age_ms =
                            now_ms - (lease.deadline_ms - kNavigationLeaseTimeoutMs);
                    }
                }
                if (reason.empty()) continue;

                {
                    std::lock_guard<std::mutex> lock(lease.mutex);
                    if (!lease.active || lease.invalid || lease.generation != generation) continue;
                    if (reason == "navigation_heartbeat_timeout" &&
                        monotonicNowMs() <= lease.deadline_ms) {
                        continue;
                    }
                    lease.active = false;
                    lease.invalid = true;
                    lease.invalid_reason = reason;
                }
                const auto paused = pauseWithRetries(pause_client, pause_client_mutex);
                writeJsonLine(
                    {
                        {"type", "navigation_lease_expired"},
                        {"event_id", makeSessionToken()},
                        {"accepted", paused.result.ok},
                        {"action", "pause_navigation"},
                        {"reason", reason},
                        {"heartbeat_age_ms", heartbeat_age_ms},
                        {"pause_attempts", paused.attempts},
                        {"status_code", paused.result.status_code},
                        {"data", paused.result.data}
                    },
                    output_mutex);
            }
        });
    } else if (persistent_world_state_session) {
        writeJsonLine(
            {
                {"type", "world_state_session_ready"},
                {"read_only", true}
            },
            output_mutex);
    } else {
        writeJsonLine({{"type", "command_client_ready"}}, output_mutex);
    }

    std::string line;
    while (std::getline(std::cin, line)) {
        if (line.empty()) continue;
        try {
            const auto cmd = nlohmann::json::parse(line);
            const std::string action = cmd.value("action", "");
            const std::string request_id = cmd.value("request_id", "");
            if (persistent_session && request_id.empty()) {
                writeJsonLine(
                    {
                        {"accepted", false},
                        {"action", action},
                        {"request_id", nullptr},
                        {"reason", "request_id_required"}
                    },
                    output_mutex);
                continue;
            }
            if (persistent_world_state_session && action != "get_world_state") {
                writeJsonLine(
                    {
                        {"accepted", false},
                        {"action", action},
                        {"request_id", request_id},
                        {"reason", "world_state_session_is_read_only"}
                    },
                    output_mutex);
                continue;
            }
            if (action == "navigation_heartbeat") {
                const bool token_ok =
                    persistent_navigation_session &&
                    cmd.value("navigation_session_token", "") == session_token;
                if (!token_ok) {
                    writeJsonLine(
                        {
                            {"accepted", false},
                            {"action", action},
                            {"request_id", request_id},
                            {"reason", "invalid_navigation_session_token"}
                        },
                        output_mutex);
                    continue;
                }
                const std::string runtime_reason =
                    runtimeSessionBlockReason(gateway, session_map_id, session_map_path);
                bool lease_ok = false;
                bool pause_for_runtime_block = false;
                std::string lease_reason;
                {
                    std::lock_guard<std::mutex> lock(lease.mutex);
                    if (!runtime_reason.empty()) {
                        pause_for_runtime_block = lease.active;
                        lease.active = false;
                        lease.invalid = true;
                        lease.invalid_reason = runtime_reason;
                    }
                    if (lease.invalid) {
                        lease_reason = lease.invalid_reason;
                    } else if (!lease.active) {
                        lease_reason = "navigation_lease_not_active";
                    } else {
                        lease.deadline_ms = monotonicNowMs() + kNavigationLeaseTimeoutMs;
                        lease_ok = true;
                    }
                }
                if (pause_for_runtime_block) {
                    pauseWithRetries(pause_client, pause_client_mutex);
                }
                if (!lease_ok) {
                    writeJsonLine(
                        {
                            {"accepted", false},
                            {"action", action},
                            {"request_id", request_id},
                            {"reason", lease_reason}
                        },
                        output_mutex);
                    continue;
                }
                writeJsonLine(
                    {
                        {"accepted", true},
                        {"action", action},
                        {"request_id", request_id},
                        {"lease_timeout_ms", kNavigationLeaseTimeoutMs}
                    },
                    output_mutex);
                continue;
            }

            uint64_t navigation_generation = 0;
            if (action == "navigate_to_pose") {
                std::lock_guard<std::mutex> lock(lease.mutex);
                if (lease.invalid) {
                    writeJsonLine(
                        {
                            {"accepted", false},
                            {"action", action},
                            {"request_id", request_id},
                            {"reason", "navigation_session_invalid:" + lease.invalid_reason}
                        },
                        output_mutex);
                    continue;
                }
                lease.active = true;
                navigation_generation = ++lease.generation;
                lease.deadline_ms = monotonicNowMs() + kNavigationLeaseTimeoutMs;
            }

            nlohmann::json result;
            {
                std::lock_guard<std::mutex> lock(gateway_mutex);
                result = processor.process(cmd);
            }
            result["action"] = action;
            result["request_id"] = request_id;
            if (action == "navigate_to_pose") {
                bool must_pause = false;
                std::string invalid_reason;
                {
                    std::lock_guard<std::mutex> lock(lease.mutex);
                    if (lease.generation == navigation_generation) {
                        if (!result.value("accepted", false)) {
                            lease.active = false;
                        } else if (lease.invalid ||
                                   monotonicNowMs() > lease.deadline_ms) {
                            must_pause = true;
                            invalid_reason = lease.invalid
                                ? lease.invalid_reason
                                : "navigation_heartbeat_timeout_during_submit";
                            lease.active = false;
                            lease.invalid = true;
                            lease.invalid_reason = invalid_reason;
                        }
                    }
                }
                if (must_pause) {
                    const auto paused = pauseWithRetries(pause_client, pause_client_mutex);
                    result = {
                        {"accepted", false},
                        {"action", action},
                        {"request_id", request_id},
                        {"reason", "navigation_session_invalid_after_submit:" + invalid_reason},
                        {"pause_accepted", paused.result.ok},
                        {"pause_attempts", paused.attempts},
                        {"pause_status_code", paused.result.status_code}
                    };
                }
            } else if (action == "pause_navigation" || action == "stop_slam") {
                std::lock_guard<std::mutex> lock(lease.mutex);
                lease.active = false;
            }
            writeJsonLine(result, output_mutex);
        } catch (const std::exception& e) {
            writeJsonLine(
                {
                    {"accepted", false},
                    {"reason", std::string("invalid_json:") + e.what()}
                },
                output_mutex);
        }
    }

    bool pause_on_close = false;
    {
        std::lock_guard<std::mutex> lock(lease.mutex);
        pause_on_close = lease.active;
        lease.active = false;
    }
    if (pause_on_close) {
        const auto paused = pauseWithRetries(pause_client, pause_client_mutex);
        writeJsonLine(
            {
                {"type", "navigation_session_closed"},
                {"event_id", makeSessionToken()},
                {"accepted", paused.result.ok},
                {"action", "pause_navigation"},
                {"reason", "navigation_session_disconnected"},
                {"pause_attempts", paused.attempts},
                {"status_code", paused.result.status_code},
                {"data", paused.result.data}
            },
            output_mutex);
    }
    monitor_running.store(false);
    if (lease_monitor.joinable()) lease_monitor.join();
    return 0;
}
