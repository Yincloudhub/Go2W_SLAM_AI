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

struct NavigationLease {
    std::mutex mutex;
    bool active{false};
    bool invalid{false};
    bool pause_pending{false};
    uint64_t generation{0};
    int64_t deadline_ms{0};
    int64_t next_pause_retry_ms{0};
    int pause_attempts_total{0};
    std::string invalid_reason;
};

struct PauseOutcome {
    slam_gateway::ServiceResult result;
    int attempts{0};
};

PauseOutcome pauseWithRetries(
    slam_gateway::SlamGateway& gateway,
    std::mutex& gateway_mutex,
    int max_attempts = 3)
{
    PauseOutcome outcome;
    std::lock_guard<std::mutex> lock(gateway_mutex);
    for (int attempt = 1; attempt <= max_attempts; ++attempt) {
        outcome.attempts = attempt;
        outcome.result = gateway.pauseNavigation();
        if (outcome.result.ok) break;
        if (attempt < max_attempts) {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
    }
    return outcome;
}

struct RuntimeSessionCheck {
    std::string reason;
    nlohmann::json world_state;
};

RuntimeSessionCheck runtimeSessionCheck(
    const slam_gateway::SlamGateway& gateway,
    const std::string& session_map_id,
    const std::string& session_map_path)
{
    RuntimeSessionCheck check;
    check.world_state = gateway.buildWorldStateJson();
    const auto pose =
        check.world_state.value("current_pose", nlohmann::json::object());
    const auto localization =
        check.world_state.value("localization", nlohmann::json::object());
    if (pose.value("map_id", "") != session_map_id ||
        pose.value("map_path", "") != session_map_path) {
        check.reason = "navigation_session_map_identity_changed";
        return check;
    }
    if (localization.value("status", "") != "localized") {
        check.reason = "navigation_session_localization_invalid";
        return check;
    }
    const int64_t pose_age_ms = localization.value("pose_age_ms", int64_t{-1});
    if (pose_age_ms < 0 || pose_age_ms > 500) {
        check.reason = "navigation_session_localization_stale";
        return check;
    }
    const auto safety =
        check.world_state.value("safety", nlohmann::json::object());
    if (!safety.value("allow_navigation", false)) {
        check.reason =
            "navigation_session_safety_blocked:" +
            safety.value("reason", "unknown");
    }
    return check;
}

bool isNavigationMotionAction(const std::string& action)
{
    return action == "navigate_to_pose" ||
        action == "supervised_departure" ||
        action == "supervised_reposition";
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
                const int64_t loop_now_ms = monotonicNowMs();
                bool retry_pending_pause = false;
                std::string pending_reason;
                {
                    std::lock_guard<std::mutex> lock(lease.mutex);
                    retry_pending_pause =
                        lease.pause_pending && loop_now_ms >= lease.next_pause_retry_ms;
                    if (retry_pending_pause) {
                        lease.next_pause_retry_ms = loop_now_ms + 1000;
                        pending_reason = lease.invalid_reason;
                    }
                }
                if (retry_pending_pause) {
                    const auto paused = pauseWithRetries(gateway, gateway_mutex);
                    int attempts_total = 0;
                    {
                        std::lock_guard<std::mutex> lock(lease.mutex);
                        lease.pause_attempts_total += paused.attempts;
                        attempts_total = lease.pause_attempts_total;
                        if (paused.result.ok) {
                            lease.pause_pending = false;
                            lease.active = false;
                        }
                    }
                    writeJsonLine(
                        {
                            {"type", "navigation_pause_retry"},
                            {"event_id", makeSessionToken()},
                            {"accepted", paused.result.ok},
                            {"action", "pause_navigation"},
                            {"reason", pending_reason},
                            {"pause_attempts", paused.attempts},
                            {"pause_attempts_total", attempts_total},
                            {"status_code", paused.result.status_code},
                            {"data", paused.result.data}
                        },
                        output_mutex);
                    continue;
                }
                {
                    std::lock_guard<std::mutex> lock(lease.mutex);
                    if (!lease.active || lease.invalid) continue;
                }

                const int64_t now_ms = loop_now_ms;
                const auto runtime_check =
                    runtimeSessionCheck(gateway, session_map_id, session_map_path);
                std::string reason = runtime_check.reason;
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
                    lease.pause_pending = true;
                    lease.next_pause_retry_ms = monotonicNowMs() + 1000;
                    lease.invalid_reason = reason;
                }
                const auto paused = pauseWithRetries(gateway, gateway_mutex);
                int attempts_total = 0;
                {
                    std::lock_guard<std::mutex> lock(lease.mutex);
                    lease.pause_attempts_total += paused.attempts;
                    attempts_total = lease.pause_attempts_total;
                    if (paused.result.ok) lease.pause_pending = false;
                }
                writeJsonLine(
                    {
                        {"type", "navigation_lease_expired"},
                        {"event_id", makeSessionToken()},
                        {"accepted", paused.result.ok},
                        {"action", "pause_navigation"},
                        {"reason", reason},
                        {"world_state", runtime_check.world_state},
                        {"heartbeat_age_ms", heartbeat_age_ms},
                        {"pause_attempts", paused.attempts},
                        {"pause_attempts_total", attempts_total},
                        {"pause_pending", !paused.result.ok},
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
                const auto runtime_check =
                    runtimeSessionCheck(gateway, session_map_id, session_map_path);
                const std::string runtime_reason = runtime_check.reason;
                bool lease_ok = false;
                bool pause_for_runtime_block = false;
                std::string lease_reason;
                {
                    std::lock_guard<std::mutex> lock(lease.mutex);
                    if (!runtime_reason.empty()) {
                        pause_for_runtime_block = lease.active;
                        lease.active = false;
                        lease.invalid = true;
                        lease.pause_pending = pause_for_runtime_block;
                        lease.next_pause_retry_ms = monotonicNowMs() + 1000;
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
                    const auto paused = pauseWithRetries(gateway, gateway_mutex);
                    std::lock_guard<std::mutex> lock(lease.mutex);
                    lease.pause_attempts_total += paused.attempts;
                    if (paused.result.ok) lease.pause_pending = false;
                }
                if (!lease_ok) {
                    writeJsonLine(
                        {
                            {"accepted", false},
                            {"action", action},
                            {"request_id", request_id},
                            {"reason", lease_reason},
                            {"world_state", runtime_check.world_state}
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
            if (isNavigationMotionAction(action)) {
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
            if (isNavigationMotionAction(action)) {
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
                            lease.pause_pending = true;
                            lease.next_pause_retry_ms = monotonicNowMs() + 1000;
                            lease.invalid_reason = invalid_reason;
                        }
                    }
                }
                if (must_pause) {
                    const auto paused = pauseWithRetries(gateway, gateway_mutex);
                    int attempts_total = 0;
                    {
                        std::lock_guard<std::mutex> lock(lease.mutex);
                        lease.pause_attempts_total += paused.attempts;
                        attempts_total = lease.pause_attempts_total;
                        if (paused.result.ok) lease.pause_pending = false;
                    }
                    result = {
                        {"accepted", false},
                        {"action", action},
                        {"request_id", request_id},
                        {"reason", "navigation_session_invalid_after_submit:" + invalid_reason},
                        {"pause_accepted", paused.result.ok},
                        {"pause_attempts", paused.attempts},
                        {"pause_attempts_total", attempts_total},
                        {"pause_pending", !paused.result.ok},
                        {"pause_status_code", paused.result.status_code}
                    };
                }
            } else if (action == "pause_navigation" || action == "stop_slam") {
                std::lock_guard<std::mutex> lock(lease.mutex);
                if (result.value("accepted", false)) {
                    lease.active = false;
                    lease.pause_pending = false;
                } else if (action == "pause_navigation" && lease.active) {
                    lease.active = false;
                    lease.invalid = true;
                    lease.pause_pending = true;
                    lease.next_pause_retry_ms = monotonicNowMs();
                    lease.invalid_reason = "explicit_pause_rejected";
                }
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
        pause_on_close = lease.active || lease.pause_pending;
        lease.active = false;
        lease.pause_pending = false;
    }
    if (pause_on_close) {
        const auto paused = pauseWithRetries(gateway, gateway_mutex);
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
