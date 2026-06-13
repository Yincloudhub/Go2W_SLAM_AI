#pragma once

namespace slam_gateway::obstacle_policy {

inline constexpr const char* kPolicyVersion = "corridor_clearance_v1";
inline constexpr double kFrontPauseM = 0.80;
inline constexpr double kFrontSlowM = 1.50;
inline constexpr double kSidePauseM = 0.20;
inline constexpr double kSideSlowM = 0.60;
inline constexpr double kRearPauseM = 0.30;
inline constexpr double kRearSlowM = 0.50;
inline constexpr double kConservativeSpeedMps = 0.20;

}  // namespace slam_gateway::obstacle_policy
