/** sport_bridge — stdin velocity control with deadman watchdog.
 *
 * Usage: ./sport_bridge eth0
 *
 * Input (stdin, one JSON per line):
 *   {"vx": 0.2, "vy": 0.0, "vyaw": 0.0}    // continuous move at velocity
 *   {"stop": true}                           // stop immediately
 *
 * Deadman: if no command received within 500ms, auto-Stop.
 * Python must send a heartbeat (move or stop) at least every 500ms.
 *
 * Output (stdout): {"ok": "true"} or {"error": "reason"}
 *
 * On stdin EOF or process exit: Stop is guaranteed.
 */

#include <unistd.h>
#include <chrono>
#include <cstdlib>
#include <iostream>
#include <string>
#include <thread>
#include <atomic>
#include <mutex>

#include <json.hpp>
#include <unitree/robot/channel/channel_factory.hpp>
#include <unitree/robot/client/client.hpp>

namespace {

constexpr const char* SPORT_SERVICE = "sport";
constexpr const char* SPORT_API_VERSION = "1.0.0.0";
constexpr int32_t SPORT_API_ID_MOVE = 1001;
constexpr int32_t SPORT_API_ID_STOP = 1002;

// ── Deadman watchdog ──
constexpr int64_t DEADMAN_TIMEOUT_MS = 500;
constexpr int64_t WATCHDOG_TICK_MS = 50;

// Last-line command clamp. Python/nav_core should already limit these,
// but the C++ bridge must not blindly pass unsafe speeds to the robot.
constexpr float MAX_ABS_VX = 0.25f;
constexpr float MAX_ABS_VY = 0.10f;
constexpr float MAX_ABS_VYAW = 0.50f;

std::atomic<int64_t> g_last_command_ms{0};
std::atomic<bool> g_watchdog_running{true};
std::atomic<bool> g_stop_requested{true};
std::mutex g_client_mu;  // protects all SportClient calls

class SportClient : public unitree::robot::Client {
public:
    SportClient() : unitree::robot::Client(SPORT_SERVICE, false) {}

    void Init() override {
        SetApiVersion(SPORT_API_VERSION);
        UT_ROBOT_CLIENT_REG_API_NO_PROI(SPORT_API_ID_MOVE);
        UT_ROBOT_CLIENT_REG_API_NO_PROI(SPORT_API_ID_STOP);
    }

    bool move(float vx, float vy, float vyaw) {
        nlohmann::json param;
        param["data"]["vx"] = vx;
        param["data"]["vy"] = vy;
        param["data"]["vyaw"] = vyaw;
        std::string reply;
        int32_t code = Call(SPORT_API_ID_MOVE, param.dump(), reply);
        return code == 0;
    }

    bool stopMove() {
        nlohmann::json param;
        param["data"] = nlohmann::json::object();
        std::string reply;
        int32_t code = Call(SPORT_API_ID_STOP, param.dump(), reply);
        return code == 0;
    }
};

// ── Thread-safe wrappers ──

bool safeMove(SportClient& client, float vx, float vy, float vyaw) {
    std::lock_guard<std::mutex> lk(g_client_mu);
    return client.move(vx, vy, vyaw);
}

bool safeStop(SportClient& client) {
    std::lock_guard<std::mutex> lk(g_client_mu);
    return client.stopMove();
}

// ── Helpers ──

int64_t now_ms() {
    return std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::steady_clock::now().time_since_epoch()
    ).count();
}

float clampAbs(float v, float limit) {
    if (v > limit) return limit;
    if (v < -limit) return -limit;
    return v;
}

void respond(const std::string& key, const std::string& value) {
    nlohmann::json j;
    j[key] = value;
    std::cout << j.dump() << std::endl;
}

void watchdog_loop(SportClient* client) {
    while (g_watchdog_running) {
        std::this_thread::sleep_for(std::chrono::milliseconds(WATCHDOG_TICK_MS));

        int64_t now = now_ms();
        int64_t last = g_last_command_ms.load();

        if (last == 0) continue;
        if (g_stop_requested.load()) continue;

        if (now - last > DEADMAN_TIMEOUT_MS) {
            std::cerr << "[sport_bridge] DEADMAN triggered: "
                      << (now - last) << "ms since last command" << std::endl;
            g_stop_requested = true;
            safeStop(*client);
        }
    }
}

}  // namespace

int main(int argc, const char** argv) {
    if (argc < 2) {
        std::cerr << "Usage: " << argv[0] << " networkInterface" << std::endl;
        return 1;
    }

    unitree::robot::ChannelFactory::Instance()->Init(0, argv[1]);

    SportClient client;
    client.Init();
    client.SetTimeout(2.0f);

    // Ensure robot is stopped when bridge starts idle. The watchdog is disabled
    // while g_stop_requested=true and becomes active after the first Move.
    g_last_command_ms = now_ms();
    g_stop_requested = true;
    safeStop(client);

    // Launch watchdog thread (joinable — clean exit)
    std::thread watchdog(watchdog_loop, &client);

    std::string line;
    while (std::getline(std::cin, line)) {
        if (line.empty()) continue;

        auto cmd = nlohmann::json::parse(line, nullptr, false);
        if (cmd.is_discarded()) {
            respond("error", "invalid_json");
            continue;
        }

        g_last_command_ms = now_ms();

        if (cmd.value("stop", false)) {
            g_stop_requested = true;
            bool ok = safeStop(client);
            respond(ok ? "ok" : "error", ok ? "true" : "stop_failed");
            continue;
        }

        float vx = cmd.value("vx", 0.0f);
        float vy = cmd.value("vy", 0.0f);
        float vyaw = cmd.value("vyaw", 0.0f);

        // Final safety clamp at the bridge layer. This protects the robot if
        // an upstream script sends an out-of-range command by mistake.
        vx = clampAbs(vx, MAX_ABS_VX);
        vy = clampAbs(vy, MAX_ABS_VY);
        vyaw = clampAbs(vyaw, MAX_ABS_VYAW);

        if (vx == 0.0f && vy == 0.0f && vyaw == 0.0f) {
            g_stop_requested = true;
            bool ok = safeStop(client);
            respond(ok ? "ok" : "error", ok ? "true" : "stop_failed");
            continue;
        }

        g_stop_requested = false;
        bool ok = safeMove(client, vx, vy, vyaw);
        if (!ok) {
            // If Move fails, force a Stop instead of leaving the last command active.
            g_stop_requested = true;
            safeStop(client);
            respond("error", "move_call_failed");
            continue;
        }

        respond("ok", "true");
    }

    // ── Clean exit sequence ──
    g_watchdog_running = false;
    g_stop_requested = true;
    safeStop(client);

    if (watchdog.joinable()) {
        watchdog.join();
    }

    std::cerr << "[sport_bridge] exiting, robot stopped" << std::endl;
    return 0;
}
