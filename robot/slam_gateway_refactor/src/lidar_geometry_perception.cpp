#include "slam_gateway/lidar_geometry_perception.hpp"

#include <algorithm>
#include <cmath>
#include <fstream>

namespace slam_gateway {
namespace {

double numberOr(const nlohmann::json& value, const char* key, double fallback)
{
    const auto it = value.find(key);
    if (it == value.end() || !it->is_number()) return fallback;
    return it->get<double>();
}

bool knownClearance(double value)
{
    return std::isfinite(value) && value >= 0.0;
}

bool clearanceBelow(double value, double threshold)
{
    return knownClearance(value) && value < threshold;
}

void updateDerivedState(LocalObstacleSummary& summary)
{
    summary.blocked_directions.clear();
    if (clearanceBelow(summary.front_clearance_m, 0.8)) summary.blocked_directions.push_back("front");
    if (clearanceBelow(summary.left_clearance_m, 0.8)) summary.blocked_directions.push_back("left");
    if (clearanceBelow(summary.right_clearance_m, 0.8)) summary.blocked_directions.push_back("right");
    if (clearanceBelow(summary.rear_clearance_m, 0.6)) summary.blocked_directions.push_back("rear");

    summary.narrow_passage = clearanceBelow(summary.left_clearance_m, 0.8) && clearanceBelow(summary.right_clearance_m, 0.8);
    if (clearanceBelow(summary.front_clearance_m, 0.8) ||
        clearanceBelow(summary.left_clearance_m, 0.8) ||
        clearanceBelow(summary.right_clearance_m, 0.8)) {
        summary.recommended_action = "pause";
    } else if (clearanceBelow(summary.front_clearance_m, 1.5) ||
               clearanceBelow(summary.left_clearance_m, 1.0) ||
               clearanceBelow(summary.right_clearance_m, 1.0)) {
        summary.recommended_action = "go_slow";
    } else {
        summary.recommended_action = "normal";
    }
}

}  // namespace

void LidarGeometryPerception::setManualClearance(double front, double left, double right, double rear)
{
    summary_.timestamp_ms = nowMs();
    summary_.source = "manual_stub";
    summary_.front_clearance_m = front;
    summary_.left_clearance_m = left;
    summary_.right_clearance_m = right;
    summary_.rear_clearance_m = rear;
    summary_.confidence = 0.0;
    summary_.front_confidence = 0.0;
    summary_.left_confidence = 0.0;
    summary_.right_confidence = 0.0;
    summary_.age_ms = -1;
    summary_.stale = true;
    updateDerivedState(summary_);
}

LocalObstacleSummary LidarGeometryPerception::getLocalObstacleSummary() const
{
    return summary_;
}

LocalObstacleSummary LidarGeometryPerception::getExternalSummaryOrFallback(const std::string& path, int64_t max_age_ms) const
{
    LocalObstacleSummary fallback = summary_;
    if (path.empty()) return fallback;

    std::ifstream input(path);
    if (!input) return fallback;

    try {
        nlohmann::json j;
        input >> j;
        if (!j.is_object()) return fallback;

        LocalObstacleSummary summary;
        summary.timestamp_ms = j.value("timestamp_ms", int64_t{0});
        summary.frame_id = j.value("frame_id", "camera_depth_optical_frame");
        summary.source = j.value("source", "unverified");
        summary.range_m = numberOr(j, "range_m", 6.0);
        summary.front_clearance_m = numberOr(j, "front_clearance_m", -1.0);
        summary.left_clearance_m = numberOr(j, "left_clearance_m", -1.0);
        summary.right_clearance_m = numberOr(j, "right_clearance_m", -1.0);
        summary.rear_clearance_m = numberOr(j, "rear_clearance_m", 6.0);
        summary.confidence = numberOr(j, "confidence", 0.0);
        const auto roi = j.value("roi_confidence", nlohmann::json::object());
        summary.front_confidence = numberOr(roi, "front", 0.0);
        summary.left_confidence = numberOr(roi, "left", 0.0);
        summary.right_confidence = numberOr(roi, "right", 0.0);
        summary.age_ms = summary.timestamp_ms > 0 ? wallClockNowMs() - summary.timestamp_ms : -1;
        const bool has_required_clearance =
            knownClearance(summary.front_clearance_m) &&
            knownClearance(summary.left_clearance_m) &&
            knownClearance(summary.right_clearance_m);
        summary.stale =
            j.value("stale", false) ||
            summary.timestamp_ms <= 0 ||
            summary.age_ms < 0 ||
            summary.age_ms > std::max<int64_t>(1, max_age_ms) ||
            !has_required_clearance;
        updateDerivedState(summary);
        return summary;
    } catch (...) {
        return fallback;
    }
}

LocalObstacleSummary LidarGeometryPerception::getFusedSummaryOrFallback(
    const std::string& lidar_path,
    int64_t lidar_max_age_ms,
    const std::string& stereo_path,
    int64_t stereo_max_age_ms) const
{
    const LocalObstacleSummary lidar = getExternalSummaryOrFallback(lidar_path, lidar_max_age_ms);
    if (lidar.source == "lidar_pointcloud" && !lidar.stale) return lidar;

    const LocalObstacleSummary stereo = getExternalSummaryOrFallback(stereo_path, stereo_max_age_ms);
    if ((stereo.source == "stereo_depth" || stereo.source == "lidar_pointcloud+stereo_depth") && !stereo.stale) {
        return stereo;
    }

    if (lidar.source == "lidar_pointcloud") return lidar;
    return stereo;
}

}  // namespace slam_gateway
