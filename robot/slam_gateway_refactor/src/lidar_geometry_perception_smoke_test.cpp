#include "slam_gateway/lidar_geometry_perception.hpp"

#include <chrono>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <string>
#include <thread>

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
    double right_confidence,
    bool calibrated = true,
    const char* calibration_id = "xt16-smoke-verified",
    double latency_ms = -1.0,
    bool supervised_release = false)
{
    nlohmann::json summary;
    summary["timestamp_ms"] = slam_gateway::wallClockNowMs();
    summary["source"] = source;
    if (std::string(source) == "lidar_pointcloud") {
        summary["parameters"] = {
            {"calibrated", calibrated},
            {"calibration_id", calibration_id},
            {"supervised_release", {
                {"active", supervised_release},
                {"release_id", supervised_release ? "xt16-engineering-smoke" : ""},
                {"max_speed_mps", supervised_release ? 0.2 : 0.0}
            }}
        };
    }
    summary["stale"] = stale;
    if (latency_ms >= 0.0) summary["latency_ms"] = latency_ms;
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

    writeSummary(
        lidar_path,
        "lidar_pointcloud",
        false,
        2.0,
        2.0,
        2.0,
        0.8,
        0.7,
        0.6,
        true,
        "");
    const auto missing_calibration_id = perception.getFusedSummaryOrFallback(
        lidar_path,
        1000,
        stereo_path,
        1000);
    require(
        missing_calibration_id.stale,
        "XT16 summary without a verified calibration ID must fail closed");

    writeSummary(
        lidar_path,
        "lidar_pointcloud",
        false,
        2.0,
        2.0,
        2.0,
        0.8,
        0.7,
        0.6,
        false,
        "",
        -1.0,
        true);
    const auto supervised_release = perception.getFusedSummaryOrFallback(
        lidar_path,
        1000,
        stereo_path,
        1000);
    require(!supervised_release.stale, "guarded supervised XT16 release should be fresh");
    require(
        supervised_release.supervised_release_id == "xt16-engineering-smoke",
        "supervised XT16 release ID must be preserved");

    writeSummary(
        lidar_path,
        "lidar_pointcloud",
        false,
        2.0,
        2.0,
        2.0,
        0.8,
        0.7,
        0.6,
        true,
        "xt16-smoke-verified",
        1500.0);
    const auto delayed_lidar = perception.getFusedSummaryOrFallback(
        lidar_path,
        1000,
        stereo_path,
        1000);
    require(delayed_lidar.stale, "sensor latency must count toward XT16 age");
    require(delayed_lidar.age_ms >= 1500, "effective XT16 age must include sensor latency");

    nlohmann::json python_contract;
    python_contract["timestamp_ms"] = slam_gateway::wallClockNowMs();
    python_contract["source"] = "lidar_pointcloud";
    python_contract["summary"] = {
        {"calibrated", true},
        {"calibration_id", "python-producer-contract"}
    };
    python_contract["stale"] = false;
    python_contract["front_clearance_m"] = 2.0;
    python_contract["left_clearance_m"] = 2.0;
    python_contract["right_clearance_m"] = 2.0;
    python_contract["rear_clearance_m"] = 4.0;
    python_contract["roi_confidence"] = {
        {"front", 0.8},
        {"left", 0.7},
        {"right", 0.6}
    };
    std::ofstream(lidar_path) << python_contract.dump();
    const auto nested_calibration = perception.getFusedSummaryOrFallback(
        lidar_path,
        1000,
        stereo_path,
        1000);
    require(!nested_calibration.stale, "Python producer calibration contract must be accepted");
    require(
        nested_calibration.calibration_id == "python-producer-contract",
        "nested Python calibration ID must be preserved");

    writeSummary(lidar_path, "lidar_pointcloud", false, 2.0, 0.5, 2.0, 0.8, 0.7, 0.6);
    const auto corridor_side = perception.getExternalSummaryOrFallback(lidar_path, 1000);
    require(corridor_side.recommended_action == "go_slow",
            "corridor side clearance should request low speed");
    require(corridor_side.blocked_directions.empty(),
            "corridor side clearance must not become a hard block");

    writeSummary(lidar_path, "lidar_pointcloud", false, 2.0, 0.1, 2.0, 0.8, 0.7, 0.6);
    const auto extreme_side = perception.getExternalSummaryOrFallback(lidar_path, 1000);
    require(extreme_side.recommended_action == "pause",
            "extremely close side obstacle should request pause");
    require(
        !extreme_side.blocked_directions.empty() &&
        extreme_side.blocked_directions.front() == "left",
        "extremely close side obstacle should block left");

    writeSummary(lidar_path, "lidar_pointcloud", false, 2.0, 2.0, 2.0, 0.8, 0.7, 0.6);
    writeSummary(stereo_path, "stereo_depth", false, 0.6, 3.0, 1.0, 0.95, 0.9, 0.85);
    const auto transient = perception.getFusedSummaryOrFallback(
        lidar_path,
        1000,
        stereo_path,
        1000);
    require(
        transient.source == "lidar_pointcloud+stereo_depth",
        "fresh sensors should be fused");
    require(
        near(transient.front_clearance_m, 2.0),
        "one stereo near frame must not override clear XT16 geometry");
    require(
        transient.secondary_front_block_confirmation_count == 1,
        "first stereo near frame must start confirmation");
    require(
        !transient.secondary_front_block_confirmed,
        "first stereo near frame must remain advisory");

    const auto repeated = perception.getFusedSummaryOrFallback(
        lidar_path,
        1000,
        stereo_path,
        1000);
    require(
        repeated.secondary_front_block_confirmation_count == 1,
        "re-reading one stereo frame must not advance confirmation");

    std::this_thread::sleep_for(std::chrono::milliseconds(2));
    writeSummary(stereo_path, "stereo_depth", false, 0.6, 3.0, 1.0, 0.95, 0.9, 0.85);
    const auto confirmed = perception.getFusedSummaryOrFallback(
        lidar_path,
        1000,
        stereo_path,
        1000);
    require(
        near(confirmed.front_clearance_m, 0.6),
        "two fresh stereo near frames must confirm the nearer obstacle");
    require(
        confirmed.secondary_front_block_confirmed,
        "second fresh stereo near frame must confirm the hard block");
    require(
        confirmed.front_clearance_source == "stereo_depth",
        "confirmed stereo obstacle must own fused front clearance");
    require(
        near(confirmed.front_confidence, 0.95),
        "front confidence must follow the selected clearance");
    require(
        near(confirmed.left_clearance_m, 2.0),
        "fusion must retain XT16 left clearance");
    require(
        near(confirmed.right_clearance_m, 2.0),
        "forward stereo must not overwrite robot-side clearance");
    require(
        confirmed.recommended_action == "pause",
        "confirmed close fused obstacle must request pause");

    std::this_thread::sleep_for(std::chrono::milliseconds(2));
    writeSummary(stereo_path, "stereo_depth", false, 2.5, 3.0, 1.0, 0.95, 0.9, 0.85);
    const auto recovered = perception.getFusedSummaryOrFallback(
        lidar_path,
        1000,
        stereo_path,
        1000);
    require(
        recovered.secondary_front_block_confirmation_count == 0,
        "clear stereo geometry must reset confirmation");
    require(
        near(recovered.front_clearance_m, 2.0),
        "recovered fusion must select the nearer clear XT16 value");

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
