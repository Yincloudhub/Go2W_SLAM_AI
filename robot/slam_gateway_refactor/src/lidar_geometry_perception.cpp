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

double optionalNestedNumberOr(
    const nlohmann::json& value,
    const char* object_key,
    const char* key,
    double object_missing_fallback)
{
    const auto object_it = value.find(object_key);
    if (object_it == value.end() || !object_it->is_object()) return object_missing_fallback;
    return numberOr(*object_it, key, -1.0);
}

bool knownClearance(double value)
{
    return std::isfinite(value) && value >= 0.0;
}

bool clearanceBelow(double value, double threshold)
{
    return knownClearance(value) && value < threshold;
}

void keepNearestClearance(
    double primary_clearance,
    double primary_confidence,
    double secondary_clearance,
    double secondary_confidence,
    double& output_clearance,
    double& output_confidence)
{
    if (!knownClearance(primary_clearance)) {
        output_clearance = secondary_clearance;
        output_confidence = secondary_confidence;
        return;
    }
    if (!knownClearance(secondary_clearance) || primary_clearance <= secondary_clearance) {
        output_clearance = primary_clearance;
        output_confidence = primary_confidence;
        return;
    }
    output_clearance = secondary_clearance;
    output_confidence = secondary_confidence;
}

void updateDerivedState(LocalObstacleSummary& summary)
{
    summary.blocked_directions.clear();
    if (clearanceBelow(summary.front_clearance_m, 0.8)) summary.blocked_directions.push_back("front");
    if (clearanceBelow(summary.left_clearance_m, 0.8)) summary.blocked_directions.push_back("left");
    if (clearanceBelow(summary.right_clearance_m, 0.8)) summary.blocked_directions.push_back("right");
    if (clearanceBelow(summary.rear_clearance_m, 0.6)) summary.blocked_directions.push_back("rear");

    summary.low_hazard_directions.clear();
    if (clearanceBelow(summary.low_hazard_front_clearance_m, 0.8)) summary.low_hazard_directions.push_back("front");
    if (clearanceBelow(summary.low_hazard_left_clearance_m, 0.8)) summary.low_hazard_directions.push_back("left");
    if (clearanceBelow(summary.low_hazard_right_clearance_m, 0.8)) summary.low_hazard_directions.push_back("right");
    if (clearanceBelow(summary.low_hazard_rear_clearance_m, 0.6)) summary.low_hazard_directions.push_back("rear");

    summary.narrow_passage = clearanceBelow(summary.left_clearance_m, 0.8) && clearanceBelow(summary.right_clearance_m, 0.8);
    if (clearanceBelow(summary.front_clearance_m, 0.8) ||
        clearanceBelow(summary.left_clearance_m, 0.8) ||
        clearanceBelow(summary.right_clearance_m, 0.8) ||
        clearanceBelow(summary.rear_clearance_m, 0.6)) {
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
    summary_.body_front_clearance_m = front;
    summary_.body_left_clearance_m = left;
    summary_.body_right_clearance_m = right;
    summary_.body_rear_clearance_m = rear;
    summary_.low_hazard_front_clearance_m = -1.0;
    summary_.low_hazard_left_clearance_m = -1.0;
    summary_.low_hazard_right_clearance_m = -1.0;
    summary_.low_hazard_rear_clearance_m = -1.0;
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
        summary.body_front_clearance_m = optionalNestedNumberOr(j, "body_clearance_m", "front", -1.0);
        summary.body_left_clearance_m = optionalNestedNumberOr(j, "body_clearance_m", "left", -1.0);
        summary.body_right_clearance_m = optionalNestedNumberOr(j, "body_clearance_m", "right", -1.0);
        summary.body_rear_clearance_m = optionalNestedNumberOr(j, "body_clearance_m", "rear", -1.0);
        summary.low_hazard_front_clearance_m = optionalNestedNumberOr(j, "low_hazard_clearance_m", "front", -1.0);
        summary.low_hazard_left_clearance_m = optionalNestedNumberOr(j, "low_hazard_clearance_m", "left", -1.0);
        summary.low_hazard_right_clearance_m = optionalNestedNumberOr(j, "low_hazard_clearance_m", "right", -1.0);
        summary.low_hazard_rear_clearance_m = optionalNestedNumberOr(j, "low_hazard_clearance_m", "rear", -1.0);
        summary.confidence = numberOr(j, "confidence", 0.0);
        const auto roi = j.value("roi_confidence", nlohmann::json::object());
        summary.front_confidence = numberOr(roi, "front", 0.0);
        summary.left_confidence = numberOr(roi, "left", 0.0);
        summary.right_confidence = numberOr(roi, "right", 0.0);
        const auto body_roi = j.value("body_roi_confidence", nlohmann::json::object());
        summary.body_front_confidence = numberOr(body_roi, "front", 0.0);
        summary.body_left_confidence = numberOr(body_roi, "left", 0.0);
        summary.body_right_confidence = numberOr(body_roi, "right", 0.0);
        summary.body_rear_confidence = numberOr(body_roi, "rear", 0.0);
        const auto low_roi = j.value("low_hazard_roi_confidence", nlohmann::json::object());
        summary.low_hazard_front_confidence = numberOr(low_roi, "front", 0.0);
        summary.low_hazard_left_confidence = numberOr(low_roi, "left", 0.0);
        summary.low_hazard_right_confidence = numberOr(low_roi, "right", 0.0);
        summary.low_hazard_rear_confidence = numberOr(low_roi, "rear", 0.0);
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
    const bool has_lidar = lidar.source == "lidar_pointcloud";
    if (has_lidar && lidar.stale) {
        // XT16 is the primary safety sensor. A present but stale/uncalibrated
        // summary must not be hidden by a secondary perception source.
        return lidar;
    }

    const LocalObstacleSummary stereo = getExternalSummaryOrFallback(stereo_path, stereo_max_age_ms);
    const bool has_fresh_stereo =
        (stereo.source == "stereo_depth" || stereo.source == "lidar_pointcloud+stereo_depth") &&
        !stereo.stale;

    if (!has_lidar) {
        return stereo;
    }
    if (!has_fresh_stereo) {
        return lidar;
    }

    LocalObstacleSummary fused = lidar;
    fused.source = "lidar_pointcloud+stereo_depth";
    fused.timestamp_ms = std::min(lidar.timestamp_ms, stereo.timestamp_ms);
    fused.age_ms = std::max(lidar.age_ms, stereo.age_ms);
    fused.stale = false;
    fused.confidence = std::min(lidar.confidence, stereo.confidence);

    keepNearestClearance(
        lidar.front_clearance_m,
        lidar.front_confidence,
        stereo.front_clearance_m,
        stereo.front_confidence,
        fused.front_clearance_m,
        fused.front_confidence);
    keepNearestClearance(
        lidar.left_clearance_m,
        lidar.left_confidence,
        stereo.left_clearance_m,
        stereo.left_confidence,
        fused.left_clearance_m,
        fused.left_confidence);
    keepNearestClearance(
        lidar.right_clearance_m,
        lidar.right_confidence,
        stereo.right_clearance_m,
        stereo.right_confidence,
        fused.right_clearance_m,
        fused.right_confidence);

    // The stereo sidecar is forward-facing and has no independent rear
    // confidence field. Keep XT16 rear/body/low-hazard measurements intact.
    updateDerivedState(fused);
    return fused;
}

}  // namespace slam_gateway
