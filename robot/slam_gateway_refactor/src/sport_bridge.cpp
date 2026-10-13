/** sport_bridge — stdin velocity control for Unitree Go2/Go2W Sport service.
 *
 * Usage: ./sport_bridge eth0
 *
 * Input (stdin, one JSON per line):
 *   {"vx": 0.2, "vy": 0.0, "vyaw": 0.0}  // continuous velocity command
 *   {"stop": true}                         // zero velocity, keep standing
 *
 * Optional manual/debug commands:
 *   {"stand_up": true}                      // StandUp (idle stand, locked joints)
 *   {"balance_stand": true}                 // BalanceStand (dynamic stand, ready to move)
 *   {"stand_down": true}                    // StandDown (controlled lie-down)
 *   {"stop_move": true}                     // official StopMove; verified safe on Go2W
 *   {"recovery": true}                      // RecoveryStand (stand up from lying down)
 *   {"static_walk": true}                   // StaticWalk — slow, quiet step-by-step
 *   {"economic_gait": true}                 // EconomicGait — energy-saving smooth gait
 *   {"trot_run": true}                      // TrotRun — standard/fast trot
 *   {"free_walk": true}                     // FreeWalk — full-motion free walking
 *   {"cross_step": true}                    // CrossStep — lateral-capable cross-stepping
 *   {"speed_level": 1}                      // SpeedLevel — set speed tier (1=slow, 2=fast)
 *
 * Safety:
 *   - Python/nav_core should send a heartbeat every 100ms.
 *   - This bridge has a 500ms deadman. If heartbeat is lost, it sends Move(0,0,0).
 *   - Stop semantics intentionally use Move(0,0,0), not StopMove(), to avoid
 *     unintended posture/state-machine transitions during navigation.
 *
 * Uses the OFFICIAL unitree::robot::go2::SportClient (not a custom DDS Client).
 * Root cause: raw Client::Call() bypasses SportClient's internal Init/Move logic.
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
#include <unitree/robot/go2/sport/sport_client.hpp>

namespace {

// Deadman watchdog.
constexpr int64_t DEADMAN_TIMEOUT_MS = 500;
constexpr int64_t WATCHDOG_TICK_MS = 50;

// Last-line command clamp. nav_core should already limit these,
// but the C++ bridge must not blindly pass unsafe speeds to the robot.
constexpr float MAX_ABS_VX = 0.50f;   // raised from 0.25 for adaptive-speed navigation
constexpr float MAX_ABS_VY = 0.30f;
constexpr float MAX_ABS_VYAW = 0.50f;

std::atomic<int64_t> g_last_command_ms{0};
std::atomic<bool> g_watchdog_running{true};
std::atomic<bool> g_stop_requested{true};
std::mutex g_client_mu;  // protects all SportClient calls

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

// ── Thread-safe wrappers around official SportClient ──

bool safeMove(unitree::robot::go2::SportClient& client, float vx, float vy, float vyaw) {
    std::lock_guard<std::mutex> lk(g_client_mu);
    int32_t code = client.Move(vx, vy, vyaw);
    if (code != 0) {
        std::cerr << "[sport_bridge] Move failed, code=" << code << std::endl;
    }
    return code == 0;
}

bool safeZeroVelocity(unitree::robot::go2::SportClient& client) {
    std::lock_guard<std::mutex> lk(g_client_mu);
    int32_t code = client.Move(0.0f, 0.0f, 0.0f);
    if (code != 0) {
        std::cerr << "[sport_bridge] Move(0,0,0) failed, code=" << code << std::endl;
    }
    return code == 0;
}

bool safeStopMove(unitree::robot::go2::SportClient& client) {
    std::lock_guard<std::mutex> lk(g_client_mu);
    int32_t code = client.StopMove();
    if (code != 0) {
        std::cerr << "[sport_bridge] StopMove failed, code=" << code << std::endl;
    }
    return code == 0;
}

bool safeStandUp(unitree::robot::go2::SportClient& client) {
    std::lock_guard<std::mutex> lk(g_client_mu);
    int32_t code = client.StandUp();
    if (code != 0) {
        std::cerr << "[sport_bridge] StandUp failed, code=" << code << std::endl;
    }
    return code == 0;
}

bool safeStandDown(unitree::robot::go2::SportClient& client) {
    std::lock_guard<std::mutex> lk(g_client_mu);
    int32_t code = client.StandDown();
    if (code != 0) {
        std::cerr << "[sport_bridge] StandDown failed, code=" << code << std::endl;
    }
    return code == 0;
}

bool safeRecoveryStand(unitree::robot::go2::SportClient& client) {
    std::lock_guard<std::mutex> lk(g_client_mu);
    int32_t code = client.RecoveryStand();
    if (code != 0) {
        std::cerr << "[sport_bridge] RecoveryStand failed, code=" << code << std::endl;
    }
    return code == 0;
}

bool safeBalanceStand(unitree::robot::go2::SportClient& client) {
    std::lock_guard<std::mutex> lk(g_client_mu);
    int32_t code = client.BalanceStand();
    if (code != 0) {
        std::cerr << "[sport_bridge] BalanceStand failed, code=" << code << std::endl;
    }
    return code == 0;
}

// ── Gait control ──

bool safeStaticWalk(unitree::robot::go2::SportClient& client) {
    std::lock_guard<std::mutex> lk(g_client_mu);
    int32_t code = client.StaticWalk();
    if (code != 0) {
        std::cerr << "[sport_bridge] StaticWalk failed, code=" << code << std::endl;
    }
    return code == 0;
}

bool safeEconomicGait(unitree::robot::go2::SportClient& client) {
    std::lock_guard<std::mutex> lk(g_client_mu);
    int32_t code = client.EconomicGait();
    if (code != 0) {
        std::cerr << "[sport_bridge] EconomicGait failed, code=" << code << std::endl;
    }
    return code == 0;
}

bool safeTrotRun(unitree::robot::go2::SportClient& client) {
    std::lock_guard<std::mutex> lk(g_client_mu);
    int32_t code = client.TrotRun();
    if (code != 0) {
        std::cerr << "[sport_bridge] TrotRun failed, code=" << code << std::endl;
    }
    return code == 0;
}

bool safeFreeWalk(unitree::robot::go2::SportClient& client) {
    std::lock_guard<std::mutex> lk(g_client_mu);
    int32_t code = client.FreeWalk();
    if (code != 0) {
        std::cerr << "[sport_bridge] FreeWalk failed, code=" << code << std::endl;
    }
    return code == 0;
}

bool safeCrossStep(unitree::robot::go2::SportClient& client, bool flag) {
    std::lock_guard<std::mutex> lk(g_client_mu);
    int32_t code = client.CrossStep(flag);
    if (code != 0) {
        std::cerr << "[sport_bridge] CrossStep(" << (flag ? "true" : "false") << ") failed, code=" << code << std::endl;
    }
    return code == 0;
}

bool safeSpeedLevel(unitree::robot::go2::SportClient& client, int level) {
    std::lock_guard<std::mutex> lk(g_client_mu);
    int32_t code = client.SpeedLevel(level);
    if (code != 0) {
        std::cerr << "[sport_bridge] SpeedLevel(" << level << ") failed, code=" << code << std::endl;
    }
    return code == 0;
}

// ── Watchdog ──

void watchdog_loop(unitree::robot::go2::SportClient* client) {
    while (g_watchdog_running) {
        std::this_thread::sleep_for(std::chrono::milliseconds(WATCHDOG_TICK_MS));

        int64_t now = now_ms();
        int64_t last = g_last_command_ms.load();

        if (last == 0) continue;
        if (g_stop_requested.load()) continue;

        if (now - last > DEADMAN_TIMEOUT_MS) {
            std::cerr << "[sport_bridge] DEADMAN triggered: "
                      << (now - last) << "ms since last command; sending Move(0,0,0)"
                      << std::endl;
            g_stop_requested = true;
            safeZeroVelocity(*client);
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

    // Use the OFFICIAL SportClient — not a hand-rolled DDS Client.
    unitree::robot::go2::SportClient client;
    client.SetTimeout(10.0f);
    client.Init();

    // Bridge starts with robot already standing (operator ensures this).
    g_last_command_ms = now_ms();
    g_stop_requested = true;

    // Launch watchdog thread (joinable — clean exit)
    std::thread watchdog(watchdog_loop, &client);

    std::cerr << "[sport_bridge] ready, waiting for commands on stdin" << std::endl;

    std::string line;
    while (std::getline(std::cin, line)) {
        if (line.empty()) continue;

        auto cmd = nlohmann::json::parse(line, nullptr, false);
        if (cmd.is_discarded()) {
            respond("error", "invalid_json");
            continue;
        }

        g_last_command_ms = now_ms();

        // Navigation-safe stop: zero velocity through Move API.
        if (cmd.value("stop", false)) {
            g_stop_requested = true;
            bool ok = safeZeroVelocity(client);
            respond(ok ? "ok" : "error", ok ? "true" : "zero_velocity_failed");
            continue;
        }

        // Manual/debug: explicit official StopMove API.
        if (cmd.value("stop_move", false)) {
            g_stop_requested = true;
            bool ok = safeStopMove(client);
            respond(ok ? "ok" : "error", ok ? "true" : "stop_move_failed");
            continue;
        }

        if (cmd.value("stand_up", false)) {
            g_stop_requested = true;
            bool ok = safeStandUp(client);
            if (ok) {
                std::this_thread::sleep_for(std::chrono::milliseconds(500));
                safeZeroVelocity(client);
            }
            respond(ok ? "ok" : "error", ok ? "true" : "stand_up_failed");
            continue;
        }

        if (cmd.value("balance_stand", false)) {
            g_stop_requested = true;
            bool ok = safeBalanceStand(client);
            if (ok) {
                std::this_thread::sleep_for(std::chrono::milliseconds(500));
                safeZeroVelocity(client);
            }
            respond(ok ? "ok" : "error", ok ? "true" : "balance_stand_failed");
            continue;
        }

        if (cmd.value("stand_down", false)) {
            g_stop_requested = true;
            bool ok = safeStandDown(client);
            respond(ok ? "ok" : "error", ok ? "true" : "stand_down_failed");
            continue;
        }

        // ── Gait control ──

        if (cmd.value("static_walk", false)) {
            bool ok = safeStaticWalk(client);
            respond(ok ? "ok" : "error", ok ? "true" : "static_walk_failed");
            continue;
        }

        if (cmd.value("economic_gait", false)) {
            bool ok = safeEconomicGait(client);
            respond(ok ? "ok" : "error", ok ? "true" : "economic_gait_failed");
            continue;
        }

        if (cmd.value("trot_run", false)) {
            bool ok = safeTrotRun(client);
            respond(ok ? "ok" : "error", ok ? "true" : "trot_run_failed");
            continue;
        }

        if (cmd.value("free_walk", false)) {
            bool ok = safeFreeWalk(client);
            respond(ok ? "ok" : "error", ok ? "true" : "free_walk_failed");
            continue;
        }

        if (cmd.contains("cross_step")) {
            bool flag = cmd.value("cross_step", true);
            bool ok = safeCrossStep(client, flag);
            respond(ok ? "ok" : "error", ok ? "true" : "cross_step_failed");
            continue;
        }

        if (cmd.contains("speed_level")) {
            int level = cmd.value("speed_level", 1);
            bool ok = safeSpeedLevel(client, level);
            respond(ok ? "ok" : "error", ok ? "true" : "speed_level_failed");
            continue;
        }

        if (cmd.value("recovery", false)) {
            g_stop_requested = true;
            bool ok = safeRecoveryStand(client);
            if (ok) {
                std::this_thread::sleep_for(std::chrono::milliseconds(500));
                safeZeroVelocity(client);
            }
            respond(ok ? "ok" : "error", ok ? "true" : "recovery_failed");
            continue;
        }

        float vx = cmd.value("vx", 0.0f);
        float vy = cmd.value("vy", 0.0f);
        float vyaw = cmd.value("vyaw", 0.0f);

        // Final safety clamp at the bridge layer.
        vx = clampAbs(vx, MAX_ABS_VX);
        vy = clampAbs(vy, MAX_ABS_VY);
        vyaw = clampAbs(vyaw, MAX_ABS_VYAW);

        if (vx == 0.0f && vy == 0.0f && vyaw == 0.0f) {
            g_stop_requested = true;
            bool ok = safeZeroVelocity(client);
            respond(ok ? "ok" : "error", ok ? "true" : "zero_velocity_failed");
            continue;
        }

        g_stop_requested = false;
        bool ok = safeMove(client, vx, vy, vyaw);
        if (!ok) {
            g_stop_requested = true;
            safeZeroVelocity(client);
            respond("error", "move_call_failed");
            continue;
        }

        respond("ok", "true");
    }

    // Clean exit sequence.
    g_watchdog_running = false;
    g_stop_requested = true;
    safeZeroVelocity(client);

    if (watchdog.joinable()) {
        watchdog.join();
    }

    std::cerr << "[sport_bridge] exiting, zero velocity sent" << std::endl;
    return 0;
}
