#pragma once
#include <array>

#include "modules/common/task_basic_types.h"
#include "modules/context/ego_planning_config.h"
namespace planning {
class RssModel {
 public:
  // first: s state: s, s_dot, s_ddot;
  // second: l state: l, l_dot, l_ddot;
  using State = std::array<double, 3UL>;
  using FrenetState = std::pair<State, State>;
  RssModel() = default;
  ~RssModel() = default;

  static bool CalculateSafeDistance(
      const FrenetState& agent, const FrenetState& ego,
      const AgentPosType& pos_type,
      const PotentialDangerousAgentDeciderConfig::RSSModelConfig& rss_config,
      double* lateral_safe_distance, double* longitudinal_safe_distance);

  static double CalculateLongitudinalSafeDistance(
      const State& front_state, const State& rear_state,
      const AgentPosType& pos_type,
      const PotentialDangerousAgentDeciderConfig::RSSModelConfig& rss_config);

  static double CalculateLateralSafeDistance(
      const State& agent, const State& ego, const AgentPosType& pos_type,
      const PotentialDangerousAgentDeciderConfig::RSSModelConfig& rss_config);
};
}  // namespace planning