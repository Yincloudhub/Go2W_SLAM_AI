#include <iostream>
#include <string>

#include <unitree/robot/channel/channel_factory.hpp>
#include <json.hpp>

#include "slam_gateway/slam_gateway.hpp"
#include "slam_gateway/llm_command_processor.hpp"

int main(int argc, const char** argv)
{
    if (argc < 2) {
        std::cout << "Usage: " << argv[0] << " networkInterface" << std::endl;
        std::cout << "Then send one JSON command per line via stdin." << std::endl;
        return -1;
    }

    unitree::robot::ChannelFactory::Instance()->Init(0, argv[1]);
    slam_gateway::SlamGateway gateway;
    gateway.initApis();
    gateway.SetTimeout(10.0f);
    slam_gateway::LlmCommandProcessor processor(gateway);  //processor初始化

    std::cout << "slam_llm_command_client ready. Input JSON lines." << std::endl;
    std::cout << "Example: {\"action\":\"get_world_state\"}" << std::endl;

    std::string line;
    while (std::getline(std::cin, line)) {
        if (line.empty()) continue;
        try {
            const auto cmd = nlohmann::json::parse(line);
            const auto result = processor.process(cmd);
            std::cout << result.dump(4) << std::endl;
        } catch (const std::exception& e) {
            nlohmann::json err;
            err["accepted"] = false;
            err["reason"] = std::string("invalid_json:") + e.what();
            std::cout << err.dump(4) << std::endl;
        }
    }

    return 0;
}
