#pragma once

#include <memory>
#include "crossing_agent_decider/crossing_agent_decider.h"
#include "dynamic_world/dynamic_world.h"
#include "ego_state_manager.h"
#include "session.h"
#include "tasks/task.h"
#include "utils/kd_path.h"
#include "virtual_lane_manager.h"

namespace planning {

class AgentLongitudinalDecider : public Task {
 public:
  explicit AgentLongitudinalDecider(
      const EgoPlanningConfigBuilder* config_builder,
      framework::Session* session);
  virtual ~AgentLongitudinalDecider() = default;

  bool Execute() override;

  bool Update();

  bool Reset();

  void Init();

  void DeciderCutInAndOutAgents();

  void AddCutInForLaneChange();

  /** Rule base cut in check condition:
   * 1.ego car is not stopped
   * 2.obj is getting closer in lat
   * 3.obj is ahead of ego
   * 4.obj is not too far(far obj doesn't need quick cut-in detection)
   * 5.lat ttc is small & obj vy is obvious & lat dist is short
   * 6.low speed large yaw obj **/
  void DeciderCutInAgent(const bool is_lane_change, const agent::Agent& agent,
                         const double ego_speed_mps, const double ego_theta,
                         const double ego_half_length,
                         const double ego_half_width,
                         const PlanningInitPoint init_point,
                         const double cut_in_distance_range_m,
                         const double cut_in_lateral_threshold_m,
                         agent::AgentManager* const mutable_agent_manager);

  void DeciderCutOutAgent();

  bool IsLargeAgentCutIn(const std::shared_ptr<VirtualLane> ego_lane,
                         const std::shared_ptr<KDPath>& planned_path,
                         const agent::Agent& agent, const double agent_max_s,
                         const double cut_in_distance_range_m,
                         const double ego_half_length, const double ego_s,
                         const double ego_theta, const double ego_speed_mps);

  void IsSlowSpeedCutinSuppression(const std::shared_ptr<KDPath>& planned_path,
                                   const PlanningInitPoint init_point,
                                   const bool is_lane_change,
                                   const agent::Agent& agent,
                                   bool* is_slow_need_suppression);

  void CalculateAgentLateralDistance(
      const double object_l_speed_mps, const double min_l, const double max_l,
      const double max_s, const double ego_speed_mps, const double ego_s,
      const double ego_l, double* const ptr_small_lateral_distance,
      double* const ptr_small_lateral_distance_with_ego_l,
      double* const ptr_large_lateral_distance);

  bool IsLargeAgent(const agent::Agent& agent);

  void UpdateCutInAgentTable();

  // filter agent for st graph
  // 1.rear agent (lane keep)
  // 2.rear agent except target lane rear agent (lane change)
  // 3.ultradistant agent
  // 4.reverse agent
  void FilterRearAgents();

  void FilterUltradistantObs();

  void FilterReverseAgents();

  bool IsConsiderBackObs(const std::shared_ptr<KDPath> planned_path,
                         const PlanningInitPoint& init_point,
                         const agent::Agent* agent, const double ego_front_s,
                         const double front_corner_s, const double ego_center_s,
                         const double front_edge_s_diff) const;

  bool FilterRearNoCutInAgent(const std::shared_ptr<KDPath> planned_path,
                              const PlanningInitPoint& init_point,
                              const double ego_front_s,
                              const double agent_front_s,
                              const agent::Agent* agent) const;

  double GetFilterUltraDistanceWithEgoVel(const double ego_vel) const;

 private:
  // framework::Session *session_ = nullptr;
  std::shared_ptr<planning_data::DynamicWorld> dynamic_world_;
  std::shared_ptr<VirtualLaneManager> virtual_lane_manager_;
  std::shared_ptr<EgoStateManager> ego_state_manager_;

  // cut-in data
  std::unordered_map<int32_t, int32_t> cut_in_agent_count_;
  std::unordered_map<int32_t, int32_t> pred_cut_in_agent_count_;
  std::unordered_map<int32_t, int32_t> rule_based_cut_in_agent_count_;
  std::unordered_set<int32_t> current_agent_ids_;

  std::shared_ptr<CrossingAgentDecider> crossing_agent_decider_;
};
}  // namespace planning