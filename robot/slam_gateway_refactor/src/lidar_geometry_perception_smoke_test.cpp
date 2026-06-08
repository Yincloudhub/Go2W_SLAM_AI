#include "slam_gateway/lidar_geometry_perception.hpp"

#include <cmath>
#include <cstdio>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <string>

#include <nlohmann/json.hpp>

namespace {

void require(bool condition, const char* message)
{
    if (!condition) throw std::runtime_error(message);
}

bool near(double actual, double expected)
{
    return std::abs(actual - expected) < 1e-9;
}

std::string temporaryPath(const char* name)
{
    return std::string("/tmp/go2w_") + name + "_" +
        std::to_string(slam_gateway::wallClockNowMs()) + ".json";
}

void writeSummary(
    const std::string& path,
    const char* source,
    bool stale,
    double front,
    double left,
    double right,
    double front_confidence,
    double left_confidence,
    double right_confidence)
{
    nlohmann::json summary;
    summary["timestamp_ms"] = slam_gateway::wallClockNowMs();
    summary["source"] = source;
    summary["stale"] = stale;
    summary["confidence"] = 0.9;
    summary["front_clearance_m"] = front;
    summary["left_clearance_m"] = left;
    summary["right_clearance_m"] = right;
    summary["rear_clearance_m"] = 4.0;
    summary["roi_confidence"] = {
        {"front", front_confidence},
        {"left", left_confidence},
        {"right", right_confidence},
    };
    std::ofstream(path) << summary.dump();
}

}  // namespace

int main()
{
    const std::string lidar_path = temporaryPath("lidar");
    const std::string stereo_path = temporaryPath("stereo");
    slam_gateway::LidarGeometryPerception perception;

    writeSummary(lidar_path, "lidar_pointcloud", true, 2.0, 2.0, 2.0, 0.8, 0.7, 0.6);
    writeSummary(stereo_path, "stereo_depth", false, 3.0, 3.0, 3.0, 0.9, 0.9, 0.9);
    const auto stale_primary = perception.getFusedSummaryOrFallback(lidar_path, 1000, stereo_path, 1000);
    require(stale_primary.source == "lidar_pointcloud", "stale XT16 must remain the selected source");
    require(stale_primary.stale, "stale XT16 must remain fail-closed");

    writeSummary(lidar_path, "lidar_pointcloud", false, 2.0, 2.0, 2.0, 0.8, 0.7, 0.6);
    writeSummary(stereo_path, "stereo_depth", false, 0.6, 3.0, 1.0, 0.95, 0.9, 0.85);
    const auto fused = perception.getFusedSummaryOrFallback(lidar_path, 1000, stereo_path, 1000);
    require(fused.source == "lidar_pointcloud+stereo_depth", "fresh sensors should be fused");
    require(near(fused.front_clearance_m, 0.6), "fusion must keep the nearest front obstacle");
    require(near(fused.front_confidence, 0.95), "front confidence must follow the selected clearance");
    require(near(fused.left_clearance_m, 2.0), "fusion must retain the nearer XT16 left clearance");
    require(near(fused.left_confidence, 0.7), "left confidence must follow the selected clearance");
    require(near(fused.right_clearance_m, 1.0), "fusion must keep the nearer stereo right clearance");
    require(near(fused.right_confidence, 0.85), "right confidence must follow the selected clearance");
    require(fused.recommended_action == "pause", "close fused obstacle must request pause");

    const auto stereo_only = perception.getFusedSummaryOrFallback(
        lidar_path + ".missing",
        1000,
        stereo_path,
        1000);
    require(stereo_only.source == "stereo_depth", "stereo may be used only when XT16 is absent");
    require(!stereo_only.stale, "fresh stereo-only summary should remain fresh");

    std::remove(lidar_path.c_str());
    std::remove(stereo_path.c_str());
    std::cout << "slam_gateway_lidar_geometry_perception_smoke_test=passed\n";
    return 0;
}
