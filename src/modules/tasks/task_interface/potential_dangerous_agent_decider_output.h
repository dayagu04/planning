#pragma once
#include <unordered_map>
namespace planning {

enum class RiskLevel {
  NO_RISK = 0,
  LOW_RISK = 1,
  HIGH_RISK = 2,
};

enum class LateralManeuver {
  IGNORE = 0,
  LEFT_NUDGE = 1,
  RIGHT_NUDGE = 2,
  BOTH_SIDE_NUDGE = 3,
  LEFT_SLIGHTLY_NUDGE = 4,
  RIGHT_SLIGHTLY_NUDGE = 5,
  BOTH_SIDE_SLIGHTLY_NUDGE = 6,
};

enum class LongitudinalManeuver {
  IGNORE = 0,
  SLIGHTLY_BRAKE = 1,
  HARD_BRAKE = 2,
  ACCELERATION = 3,
};

struct RecommendedManeuver {
  LateralManeuver lateral_maneuver{LateralManeuver::IGNORE};
  LongitudinalManeuver longitudinal_maneuver{LongitudinalManeuver::IGNORE};
};

struct DangerousAgentInfo {
  DangerousAgentInfo(int32_t id, RiskLevel rk, const RecommendedManeuver& rm, double lat, double lon):
  id(id), risk_level(rk), recommended_maneuver(rm), lateral_distance(lat), longitudinal_distance(lon) {}
  int32_t id;
  RiskLevel risk_level;
  RecommendedManeuver recommended_maneuver;
  double lateral_distance;
  double longitudinal_distance;
};
struct PotentialDangerousAgentDeciderOutput {
  std::vector<DangerousAgentInfo> dangerous_agent_info;
};
}  // namespace planning