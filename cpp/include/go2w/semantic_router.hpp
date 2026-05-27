#pragma once

#include <string>
#include <vector>

#include <nlohmann/json.hpp>

namespace go2w {

struct ResolvedTarget {
    std::string node_id;
    std::string name;
    std::vector<std::string> matched_terms;
    std::size_t first_index = 0;
    bool photo_required = false;
    bool needs_calibration = false;
};

struct SemanticRoute {
    bool matched = false;
    bool multi_target = false;
    bool capture_requested = false;
    std::string reason;
    std::vector<ResolvedTarget> targets;
    nlohmann::json task_queue;
    nlohmann::json slam_commands;
};

class SemanticRouter {
public:
    SemanticRouter(nlohmann::json registry, std::string map_id);

    SemanticRoute planText(const std::string& text, double speed_mps, int mode) const;
    nlohmann::json buildNavigateCommand(const std::string& node_id, double speed_mps, int mode) const;
    const nlohmann::json* findNode(const std::string& node_id) const;
    std::string mapPath(const std::string& fallback = "/home/unitree/test.pcd") const;

private:
    const nlohmann::json* findMap() const;
    std::vector<ResolvedTarget> resolveTargets(const std::string& text) const;
    bool commandRequestsCapture(const std::string& text) const;
    bool nodeHasTag(const nlohmann::json& node, const std::string& tag) const;
    nlohmann::json poseToUnitreeJson(const nlohmann::json& node, double speed_mps, int mode) const;

private:
    nlohmann::json registry_;
    std::string map_id_;
};

}  // namespace go2w
