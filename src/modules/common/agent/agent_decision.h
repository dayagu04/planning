#pragma once

namespace planning {
namespace agent {

enum class AgentDecisionType {
  NOT_SET,
  YIELD,
  OVERTAKE,
  NUDGE,
  SIDE_PASS,
  NEIGHBOR_YIELD,
  NEIGHBOR_OVERTAKE,
  IGNORE,
};

enum class AgentLateralDecisionType {
  NOT_SET,
  BLOCK,  // block target lane，not nudge
  NUDGE,
  IGNORE  // not consider in lateral decision
};

enum class NudgeDirection {
  NOT_SET,
  NUDGE_LEFT,  // nudge from left, this is right obs
  NUDGE_RIGHT
};

class AgentDecision {
 public:
  AgentDecision() = default;
  ~AgentDecision() = default;

  const AgentDecisionType agent_decision_type() const;
  const AgentLateralDecisionType agent_lateral_decision_type() const;
  const NudgeDirection nudge_direction() const;
  double nudge_left_safe_distance() const;
  double nudge_right_safe_distance() const;
  const bool is_lateral_smooth_cutin() const;
  const bool is_lane_change_neighbor_lane_agent() const;
  const bool is_intersection() const;
  void set_agent_decision_type(const AgentDecisionType& agent_decision_type);
  void set_agent_lateral_decision_type(
      const AgentLateralDecisionType& agent_lateral_decision_type);
  void set_nudge_direction(const NudgeDirection& nudge_direction);
  void set_is_lateral_smooth_cutin(const bool is_smooth_cutin);
  void set_is_lane_change_neighbor_lane_agent(
      const bool is_neighbor_lane_agent);
  void set_nudge_left_safe_distance(const double nudge_left_safe_distance);
  void set_nudge_right_safe_distance(const double nudge_right_safe_distance);
  void set_is_intersection(const bool is_intersection);

 private:
  AgentDecisionType agent_decision_type_ = AgentDecisionType::NOT_SET;
  AgentLateralDecisionType agent_lateral_decision_type_ =
      AgentLateralDecisionType::NOT_SET;
  NudgeDirection nudge_direction_ = NudgeDirection::NOT_SET;
  bool is_lateral_smooth_cutin_ = false;
  bool is_lane_change_neighbor_lane_agent_ = false;
  double nudge_left_safe_distance_ = 10.0;
  double nudge_right_safe_distance_ = 10.0;
  bool is_intersection_ = false;
};

}  // namespace agent
}  // namespace planning