#include "slam_gateway/topology_manager.hpp"

#include <fstream>
#include <iostream>

namespace slam_gateway {

std::string TopologyManager::makeAutoName() const
{
    return "wp_" + std::to_string(poses_.size());
}

void TopologyManager::addPose(PoseData pose, const std::string& name)
{
    pose.name = name.empty() ? makeAutoName() : name;
    poses_.push_back(pose);
}

void TopologyManager::clear()
{
    poses_.clear();
}

std::size_t TopologyManager::size() const
{
    return poses_.size();
}

std::vector<PoseData> TopologyManager::list() const
{
    return poses_;
}

std::optional<PoseData> TopologyManager::findByName(const std::string& name) const
{
    for (const auto& p : poses_) {
        if (p.name == name) return p;
    }
    return std::nullopt;
}

bool TopologyManager::saveJson(const std::string& path) const
{
    nlohmann::json j;
    j["version"] = 1;
    j["waypoints"] = nlohmann::json::array();
    for (const auto& p : poses_) {
        j["waypoints"].push_back(p.toJson());
    }

    std::ofstream ofs(path);
    if (!ofs.is_open()) return false;
    ofs << j.dump(4) << std::endl;
    return true;
}

bool TopologyManager::loadJson(const std::string& path)
{
    std::ifstream ifs(path);
    if (!ifs.is_open()) return false;

    nlohmann::json j;
    ifs >> j;
    if (!j.contains("waypoints") || !j["waypoints"].is_array()) return false;

    poses_.clear();
    for (const auto& item : j["waypoints"]) {
        poses_.push_back(PoseData::fromJson(item));
    }
    return true;
}

void TopologyManager::print() const
{
    std::cout << "waypoint num:" << poses_.size() << std::endl;
    for (std::size_t i = 0; i < poses_.size(); ++i) {
        std::cout << "  [" << i << "] " << poses_[i].summary() << std::endl;
    }
}

}  // namespace slam_gateway
