#include "go2w/plan_executor.hpp"

#include <fstream>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <string>

namespace {

std::string readAll(std::istream& input)
{
    std::ostringstream ss;
    ss << input.rdbuf();
    return ss.str();
}

nlohmann::json readJsonFile(const std::string& path)
{
    std::ifstream file(path);
    if (!file) throw std::runtime_error("failed to open " + path);
    return nlohmann::json::parse(readAll(file));
}

void usage(const char* argv0)
{
    std::cerr << "Usage: " << argv0 << " --registry MAP_REGISTRY_JSON [--plan PLAN_JSON]\n"
              << "If --plan is omitted, LocalLlmPlan JSON is read from stdin.\n";
}

}  // namespace

int main(int argc, char** argv)
{
    std::string registry_path;
    std::string plan_path;

    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        if (arg == "--registry" && i + 1 < argc) {
            registry_path = argv[++i];
        } else if (arg == "--plan" && i + 1 < argc) {
            plan_path = argv[++i];
        } else if (arg == "-h" || arg == "--help") {
            usage(argv[0]);
            return 0;
        } else {
            usage(argv[0]);
            return 2;
        }
    }

    if (registry_path.empty()) {
        usage(argv[0]);
        return 2;
    }

    try {
        const auto registry = readJsonFile(registry_path);
        nlohmann::json plan;
        if (!plan_path.empty()) {
            plan = readJsonFile(plan_path);
        } else {
            plan = nlohmann::json::parse(readAll(std::cin));
        }

        go2w::PlanExecutor executor(registry);
        const auto result = executor.dryRun(plan);
        nlohmann::json output = {
            {"accepted", result.accepted},
            {"reason", result.reason},
            {"slam_command", result.slam_command.is_null() ? nlohmann::json(nullptr) : result.slam_command},
            {"dry_run_sequence", result.dry_run_sequence.is_null() ? nlohmann::json::array() : result.dry_run_sequence},
        };
        std::cout << output.dump(2) << "\n";
        return result.accepted ? 0 : 1;
    } catch (const std::exception& exc) {
        std::cerr << "error: " << exc.what() << "\n";
        return 1;
    }
}
