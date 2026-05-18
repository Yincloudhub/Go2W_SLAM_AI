#pragma once

#include <string>
#include <vector>
#include <optional>

#include "slam_gateway/models.hpp"

namespace slam_gateway {

class TopologyManager {
public:
    static constexpr const char* topology_points_path = "/home/unitree/topology_points.json";
    void addPose(PoseData pose, const std::string& name = "");
    void clear();
    std::size_t size() const;
    std::vector<PoseData> list() const;
    std::optional<PoseData> findByName(const std::string& name) const;
    bool saveJson(const std::string& path) const;
    bool loadJson(const std::string& path);
    void print() const;

private:
    std::vector<PoseData> poses_;
    std::string makeAutoName() const;
};

}  // namespace slam_gateway
