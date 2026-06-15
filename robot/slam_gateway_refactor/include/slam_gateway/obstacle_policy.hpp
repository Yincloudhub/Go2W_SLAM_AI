#pragma once

#include <cmath>

namespace slam_gateway::obstacle_policy {

inline constexpr const char* kPolicyVersion = "semantic_mobility_v4";
inline constexpr double kFrontPauseM = 0.80;
inline constexpr double kFrontSlowM = 1.50;
inline constexpr double kSidePauseM = 0.20;
inline constexpr double kSideSlowM = 0.60;
inline constexpr double kRearPauseM = 0.30;
inline constexpr double kRearSlowM = 0.50;
inline constexpr double kConservativeSpeedMps = 0.20;
inline constexpr double kInitialTurnThresholdRad = 0.20;
inline constexpr double kTurningSideClearanceM = 0.35;
inline constexpr double kTurningRearClearanceM = 0.30;
inline constexpr double kDepartureMaxDistanceM = 0.50;
inline constexpr double kDepartureMaxSpeedMps = 0.10;
inline constexpr double kDepartureFrontReserveM = 0.50;
inline constexpr double kRepositionReserveM = 0.35;
inline constexpr double kRepositionMinDistanceM = 0.20;
inline constexpr int kSecondaryFrontBlockConfirmFrames = 2;

inline double normalizeAngle(double angle)
{
    constexpr double kPi = 3.14159265358979323846;
    while (angle > kPi) angle -= 2.0 * kPi;
    while (angle < -kPi) angle += 2.0 * kPi;
    return angle;
}

inline double targetBearingError(
    double current_x,
    double current_y,
    double current_yaw,
    double target_x,
    double target_y)
{
    return normalizeAngle(
        std::atan2(target_y - current_y, target_x - current_x) - current_yaw);
}

inline bool requiresInitialTurn(double bearing_error_rad)
{
    return std::abs(bearing_error_rad) > kInitialTurnThresholdRad;
}

inline bool turningEnvelopeClear(
    double left_clearance_m,
    double right_clearance_m,
    double rear_clearance_m)
{
    return left_clearance_m >= kTurningSideClearanceM &&
        right_clearance_m >= kTurningSideClearanceM &&
        rear_clearance_m >= kTurningRearClearanceM;
}

}  // namespace slam_gateway::obstacle_policy
