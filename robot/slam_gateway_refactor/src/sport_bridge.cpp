/** Minimal sport bridge — stdin velocity control.
 * 
 * Usage: ./sport_bridge eth0
 * Input (stdin, one JSON per line):
 *   {"vx": 0.2, "vy": 0.0, "vyaw": 0.0, "duration_ms": 1000}   // move 1s
 *   {"vx": 0.0, "vy": 0.0, "vyaw": 0.5, "duration_ms": 500}    // rotate
 *   {"stop": true}                                                // stop
 *
 * Output (stdout): {"ok": true} or {"error": "reason"}
 */

#include <unistd.h>
#include <chrono>
#include <cstdlib>
#include <iostream>
#include <string>
#include <thread>

#include <json.hpp>
#include <unitree/robot/channel/channel_factory.hpp>
#include <unitree/robot/client/client.hpp>

namespace {

constexpr const char* SPORT_SERVICE = "sport";
constexpr const char* SPORT_API_VERSION = "1.0.0.0";
constexpr int32_t SPORT_API_ID_MOVE = 1001;
constexpr int32_t SPORT_API_ID_STOP = 1002;

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

void respond(const std::string& key, const std::string& value) {
    nlohmann::json j;
    j[key] = value;
    std::cout << j.dump() << std::endl;
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

    std::string line;
    while (std::getline(std::cin, line)) {
        if (line.empty()) continue;

        auto cmd = nlohmann::json::parse(line, nullptr, false);
        if (cmd.is_discarded()) {
            respond("error", "invalid_json");
            continue;
        }

        if (cmd.value("stop", false)) {
            bool ok = client.stopMove();
            respond(ok ? "ok" : "error", ok ? "true" : "stop_failed");
            continue;
        }

        float vx = cmd.value("vx", 0.0f);
        float vy = cmd.value("vy", 0.0f);
        float vyaw = cmd.value("vyaw", 0.0f);
        int duration_ms = cmd.value("duration_ms", 0);

        bool ok = client.move(vx, vy, vyaw);
        if (!ok) {
            respond("error", "move_call_failed");
            continue;
        }

        if (duration_ms > 0) {
            std::this_thread::sleep_for(std::chrono::milliseconds(duration_ms));
            client.stopMove();
        }

        respond("ok", "true");
    }

    client.stopMove();
    return 0;
}
