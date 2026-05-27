#include "go2w/operator_panel.hpp"

#include <cstdlib>
#include <iostream>
#include <string>

namespace {

void usage(const char* argv0)
{
    std::cerr << "Usage: " << argv0 << " [options]\n"
              << "Options:\n"
              << "  --repo-root PATH             Repo root, default ..\n"
              << "  --gateway-client PATH         slam_llm_command_client path\n"
              << "  --start-slam-script PATH      SLAM startup script relative to repo root or absolute\n"
              << "  --ensure-slam-on-start        Start/check LiDAR driver and SLAM before opening panel\n"
              << "  --interface IFACE             Network interface, default eth0\n"
              << "  --gateway-timeout-s SECONDS   Gateway command timeout, default 30\n"
              << "  --llm-http-url URL            OpenAI-compatible local HTTP endpoint, disabled by default\n"
              << "  --llm-http-model MODEL        HTTP LLM model name, default local\n"
              << "  --llm-http-timeout-s SECONDS  HTTP LLM timeout, default 20\n"
              << "  --llm-http-max-tokens N       HTTP LLM max tokens, default 256\n"
              << "  --slam-poll-interval-s SEC    World-state polling interval during execution\n"
              << "  --ui-refresh-interval-s SEC   Console/UI progress refresh interval\n"
              << "  --feedback-interval-s SEC     Operator feedback message interval\n"
              << "  --llm-feedback-interval-s SEC LLM/UI feedback request interval\n"
              << "  --max-arrival-samples N       Max stored arrival samples, default 120\n"
              << "  --max-feedback-events N       Max stored operator feedback events, default 120\n"
              << "  --max-llm-feedback-events N   Max stored LLM feedback events, default 40\n"
              << "  --gateway-error-limit N        Consecutive world-state failures before blocking\n"
              << "  --python PATH                 Python executable, default python3\n"
              << "  --current-node NODE_ID        Known current anchor node\n"
              << "  --execute                     Allow real robot execution\n"
              << "  --weak                        Start in weak-link summary mode\n"
              << "  --watch SECONDS               Non-interactive watch mode; 0 means forever\n";
}

}  // namespace

int main(int argc, char** argv)
{
    go2w::OperatorPanelConfig config;
    int watch_seconds = -1;

    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        if (arg == "--repo-root" && i + 1 < argc) {
            config.repo_root = argv[++i];
        } else if (arg == "--gateway-client" && i + 1 < argc) {
            config.gateway_client = argv[++i];
        } else if (arg == "--start-slam-script" && i + 1 < argc) {
            config.start_slam_script = argv[++i];
        } else if (arg == "--ensure-slam-on-start") {
            config.ensure_slam_on_start = true;
        } else if (arg == "--interface" && i + 1 < argc) {
            config.network_interface = argv[++i];
        } else if (arg == "--gateway-timeout-s" && i + 1 < argc) {
            config.gateway_timeout_s = std::atoi(argv[++i]);
        } else if (arg == "--llm-http-url" && i + 1 < argc) {
            config.llm_http_url = argv[++i];
        } else if (arg == "--llm-http-model" && i + 1 < argc) {
            config.llm_http_model = argv[++i];
        } else if (arg == "--llm-http-timeout-s" && i + 1 < argc) {
            config.llm_http_timeout_s = std::atoi(argv[++i]);
        } else if (arg == "--llm-http-max-tokens" && i + 1 < argc) {
            config.llm_http_max_tokens = std::atoi(argv[++i]);
        } else if (arg == "--slam-poll-interval-s" && i + 1 < argc) {
            config.slam_poll_interval_s = std::atof(argv[++i]);
        } else if (arg == "--ui-refresh-interval-s" && i + 1 < argc) {
            config.ui_refresh_interval_s = std::atof(argv[++i]);
        } else if (arg == "--feedback-interval-s" && i + 1 < argc) {
            config.operator_feedback_interval_s = std::atof(argv[++i]);
        } else if (arg == "--llm-feedback-interval-s" && i + 1 < argc) {
            config.llm_feedback_interval_s = std::atof(argv[++i]);
        } else if (arg == "--max-arrival-samples" && i + 1 < argc) {
            config.max_arrival_samples = std::atoi(argv[++i]);
        } else if (arg == "--max-feedback-events" && i + 1 < argc) {
            config.max_feedback_events = std::atoi(argv[++i]);
        } else if (arg == "--max-llm-feedback-events" && i + 1 < argc) {
            config.max_llm_feedback_events = std::atoi(argv[++i]);
        } else if (arg == "--gateway-error-limit" && i + 1 < argc) {
            config.max_consecutive_gateway_errors = std::atoi(argv[++i]);
        } else if (arg == "--python" && i + 1 < argc) {
            config.python = argv[++i];
        } else if (arg == "--current-node" && i + 1 < argc) {
            config.current_node = argv[++i];
        } else if (arg == "--execute") {
            config.execute_enabled = true;
        } else if (arg == "--weak") {
            config.weak_link_mode = true;
        } else if (arg == "--watch" && i + 1 < argc) {
            watch_seconds = std::atoi(argv[++i]);
        } else if (arg == "-h" || arg == "--help") {
            usage(argv[0]);
            return 0;
        } else {
            usage(argv[0]);
            return 2;
        }
    }

    go2w::OperatorPanel panel(config);
    if (watch_seconds >= 0) {
        panel.watchWorld(watch_seconds);
        return 0;
    }
    return panel.runInteractive();
}
