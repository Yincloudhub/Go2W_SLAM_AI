#include "slam_gateway/lidar_geometry_perception.hpp"

namespace slam_gateway {

void LidarGeometryPerception::setManualClearance(double front, double left, double right, double rear)
{
    summary_.timestamp_ms = nowMs();
    summary_.front_clearance_m = front;
    summary_.left_clearance_m = left;
    summary_.right_clearance_m = right;
    summary_.rear_clearance_m = rear;
    summary_.blocked_directions.clear();

    if (front < 0.8) summary_.blocked_directions.push_back("front");
    if (left < 0.6) summary_.blocked_directions.push_back("left");
    if (right < 0.6) summary_.blocked_directions.push_back("right");
    if (rear < 0.6) summary_.blocked_directions.push_back("rear");

    if (front < 0.8) {
        summary_.recommended_action = "pause";
    } else if (front < 1.5 || left < 0.8 || right < 0.8) {
        summary_.recommended_action = "go_slow";
    } else {
        summary_.recommended_action = "normal";
    }
}

LocalObstacleSummary LidarGeometryPerception::getLocalObstacleSummary() const
{
    return summary_;
}

}  // namespace slam_gateway
