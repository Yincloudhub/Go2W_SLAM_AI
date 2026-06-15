#pragma once

#include <cstdint>
#include <string_view>

namespace slam_gateway::obstacle_policy {

inline constexpr const char* kPolicyVersion = "semantic_mobility_v6";
inline constexpr double kFrontPauseM = 0.80;
inline constexpr double kFrontSlowM = 1.50;
inline constexpr double kSidePauseM = 0.20;
inline constexpr double kSideSlowM = 0.60;
inline constexpr double kRearPauseM = 0.30;
inline constexpr double kRearSlowM = 0.50;
inline constexpr double kConservativeSpeedMps = 0.20;
inline constexpr double kNativeNavigationMinSpeedMps = 0.20;
inline constexpr double kDepartureMaxDistanceM = 0.50;
inline constexpr double kDepartureMaxSpeedMps = 0.10;
inline constexpr double kDepartureFrontReserveM = 0.50;
inline constexpr double kRepositionForwardReserveM = 0.50;
inline constexpr double kRepositionSideReserveM = 0.35;
inline constexpr double kRepositionRearReserveM = 0.30;
inline constexpr double kRepositionMinDistanceM = 0.20;
inline constexpr int kSecondaryFrontBlockConfirmFrames = 2;
inline constexpr int64_t kSupervisedObstacleMaxAgeMs = 1500;

inline double repositionReserveM(std::string_view direction)
{
    if (direction == "forward") return kRepositionForwardReserveM;
    if (direction == "backward") return kRepositionRearReserveM;
    return kRepositionSideReserveM;
}

}  // namespace slam_gateway::obstacle_policy
