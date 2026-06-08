#include <iostream>
#include <termio.h>
#include <unistd.h>

#include <unitree/robot/channel/channel_factory.hpp>

#include "slam_gateway/slam_gateway.hpp"

namespace {

unsigned char keyDetection()
{
    termios tms_old{}, tms_new{};
    tcgetattr(0, &tms_old);
    tms_new = tms_old;
    tms_new.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(0, TCSANOW, &tms_new);
    unsigned char ch = getchar();
    tcsetattr(0, TCSANOW, &tms_old);
    std::cout << "\033[1;32m" << "Key " << ch << " pressed." << "\033[0m" << std::endl;
    return ch;
}

void printMenu()
{
    std::cout << "*********************** Unitree SLAM Gateway Keyboard ***********************\n";
    std::cout << "---------------            q    w                -----------------\n";
    std::cout << "---------------            a    s   d   f        -----------------\n";
    std::cout << "---------------            z    x                -----------------\n";
    std::cout << "------------------------------------------------------------------\n";
    std::cout << "------------------ q: Start mapping            -------------------\n";
    std::cout << "------------------ w: End mapping              -------------------\n";
    std::cout << "------------------ a: Start relocation         -------------------\n";
    std::cout << "------------------ s: Add pose to task list    -------------------\n";
    std::cout << "------------------ d: Execute task list        -------------------\n";
    std::cout << "------------------ f: Clear task list          -------------------\n";
    std::cout << "------------------ z: Pause navigation         -------------------\n";
    std::cout << "------------------ x: Resume navigation        -------------------\n";
    std::cout << "---------------- Press any other key to stop SLAM ----------------\n";
    std::cout << "------------------------------------------------------------------\n";
    std::cout << "No extra keyboard command is added in this executable.\n";
    std::cout << "LLM commands are handled by the separate slam_llm_command_client.\n";
    std::cout << "------------------------------------------------------------------\n" << std::endl;
}

}  // namespace

int main(int argc, const char** argv)
{
    if (argc < 2) {
        std::cout << "Usage: " << argv[0] << " networkInterface" << std::endl;
        return -1;
    }

    unitree::robot::ChannelFactory::Instance()->Init(0, argv[1]);
    slam_gateway::SlamGateway gateway;
    gateway.initApis();
    gateway.SetTimeout(10.0f);

    printMenu();

    while (true) {
        unsigned char currentKey = keyDetection();
        switch (currentKey) {
        case 'q':
            gateway.startMapping("indoor");
            break;
        case 'w':
            gateway.endMapping("/home/unitree/test.pcd");
            break;
        case 'a':
            gateway.startRelocation("/home/unitree/test.pcd");
            break;
        case 's':
            gateway.addCurrentPoseAsWaypoint();
            break;
        case 'd':
            // Keep original key behavior: execute waypoint list as patrol loop.
            gateway.taskThreadRun(true);
            break;
        case 'f':
            gateway.clearWaypoints();
            break;
        case 'z':
            gateway.pauseNavigation();
            break;
        case 'x':
            if (gateway.getSafetyDecision().allow_navigation) {
                gateway.resumeNavigation();
            } else {
                std::cout << "Safety blocked resume_navigation." << std::endl;
            }
            break;
        default:
            gateway.taskThreadStop();
            gateway.stopNode();
            break;
        }
    }

    return 0;
}
