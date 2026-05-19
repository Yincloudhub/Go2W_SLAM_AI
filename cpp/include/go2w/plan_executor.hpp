#pragma once

#include <string>
#include <vector>

#include <nlohmann/json.hpp>

namespace go2w {

struct ExecutorResult {
    bool accepted = false;
    std::string reason;
    nlohmann::json slam_command;
    nlohmann::json dry_run_sequence;
};

class PlanExecutor {
public:
    explicit PlanExecutor(nlohmann::json map_registry);

    ExecutorResult dryRun(const nlohmann::json& plan) const;

private:
    const nlohmann::json* findMap(const std::string& map_id) const;
    const nlohmann::json* findNode(const nlohmann::json& map, const std::string& node_id) const;
    nlohmann::json poseToUnitreeJson(const nlohmann::json& node, double speed_override, int mode_override) const;
    ExecutorResult reject(const std::string& reason) const;

private:
    nlohmann::json registry_;
};

void validateLocalLlmPlanShape(const nlohmann::json& plan);
std::vector<std::string> collectTools(const nlohmann::json& plan);

}  // namespace go2w
