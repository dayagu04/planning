#include "agent_decision.h"

namespace planning {
namespace agent {

const AgentDecisionType AgentDecision::agent_decision_type() const {
  return agent_decision_type_;
}

const AgentLateralDecisionType AgentDecision::agent_lateral_decision_type()
    const {
  return agent_lateral_decision_type_;
}

const NudgeDirection AgentDecision::nudge_direction() const {
  return nudge_direction_;
}

double AgentDecision::nudge_left_safe_distance() const {
  return nudge_left_safe_distance_;
}

double AgentDecision::nudge_right_safe_distance() const {
  return nudge_right_safe_distance_;
}

const bool AgentDecision::is_lateral_smooth_cutin() const {
  return is_lateral_smooth_cutin_;
}

const bool AgentDecision::is_lane_change_neighbor_lane_agent() const {
  return is_lane_change_neighbor_lane_agent_;
}
const bool AgentDecision::is_intersection() const { return is_intersection_; }

void AgentDecision::set_agent_decision_type(
    const AgentDecisionType& agent_decision_type) {
  agent_decision_type_ = agent_decision_type;
}

void AgentDecision::set_agent_lateral_decision_type(
    const AgentLateralDecisionType& agent_lateral_decision_type) {
  agent_lateral_decision_type_ = agent_lateral_decision_type;
}

void AgentDecision::set_nudge_direction(const NudgeDirection& nudge_direction) {
  nudge_direction_ = nudge_direction;
}

void AgentDecision::set_is_lateral_smooth_cutin(const bool is_smooth_cutin) {
  is_lateral_smooth_cutin_ = is_smooth_cutin;
}

void AgentDecision::set_is_lane_change_neighbor_lane_agent(
    const bool is_neighbor_lane_agent) {
  is_lane_change_neighbor_lane_agent_ = is_neighbor_lane_agent;
}

void AgentDecision::set_nudge_left_safe_distance(
    const double nudge_left_safe_distance) {
  nudge_left_safe_distance_ = nudge_left_safe_distance;
}

void AgentDecision::set_nudge_right_safe_distance(
    const double nudge_right_safe_distance) {
  nudge_right_safe_distance_ = nudge_right_safe_distance;
}

void AgentDecision::set_is_intersection(const bool is_intersection) {
  is_intersection_ = is_intersection;
}

}  // namespace agent
}  // namespace planning