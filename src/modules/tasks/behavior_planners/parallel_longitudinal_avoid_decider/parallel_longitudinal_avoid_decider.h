#pragma once
#include "agent/agent.h"
#include "config/basic_type.h"
#include "context/lateral_obstacle.h"
#include "st_graph/st_graph_helper.h"
#include "trajectory1d/variable_coordinate_time_optimal_trajectory.h"
#include "virtual_lane.h"

namespace planning {

class ParallelLongitudinalAvoidDecider {
 public:
  ParallelLongitudinalAvoidDecider(
      const EgoPlanningConfigBuilder* config_builder,
      framework::Session* session);
  ~ParallelLongitudinalAvoidDecider() = default;

  bool Execute();
  const std::string& Name() const { return name_; }

 private:
  enum class DeciderState : int32_t {
    IDLE = 0,
    RUNNING = 1,
    EXITING = 2,
    COOLDOWN = 3
  };

  void ProcessStateMachine();
  void TransitionToState(DeciderState new_state);

  bool IsStartRunning();
  bool IsStopRunning();
  void OutputRunning();
  void ResetInnerParam();

  bool CheckIfNeedOvertake(const agent::Agent* agent);
  bool CheckIfNeedYield(const agent::Agent* agent);
  bool CheckLateralConflict(const agent::Agent* agent,
                            const double lane_width_ratio);
  bool CheckLateralTTC(const agent::Agent* agent,
                       const double lateral_ttc_threshold_s);
  bool CheckLeadAndTargetIsTruck(const agent::Agent* agent);
  bool CheckIfTheTruckIsParallel();

  bool CalculateOvertakeReachingTrajectory(
      const PlanningInitPoint& init_point,
      const agent::Agent* const truck_agent,
      const speed::StGraphHelper* const st_graph_helper,
      std::shared_ptr<VariableCoordinateTimeOptimalTrajectory>& trajectory);

  bool CalculateYieldReachingTrajectory(
      const PlanningInitPoint& init_point,
      const agent::Agent* const truck_agent,
      const speed::StGraphHelper* const st_graph_helper,
      std::shared_ptr<VariableCoordinateTimeOptimalTrajectory>& trajectory);

  bool IsSatisfiedOvertakeExitCondition(
      const double ego_speed_kmh,
      const speed::StGraphHelper* const st_graph_helper,
      const agent::Agent* const agent, const int64_t agent_id);

  bool IsSatisfiedYieldExitCondition(
      const double ego_speed_kmh,
      const speed::StGraphHelper* const st_graph_helper,
      const agent::Agent* const agent, const int64_t agent_id);

  bool IsSatisfiedLeadAndTargetIsTruckExitCondition(
      const agent::Agent* const agent);

  bool IsDynamicTruckOrBus(const agent::Agent* agent);
  bool ConstructNeighborLaneStGraph(const agent::Agent* const truck_agent);
  void SelectNearestNeighborAgent(const int64_t left_front_node_id,
                                  const int64_t left_node_id,
                                  const int64_t right_front_node_id,
                                  const int64_t right_node_id,
                                  int64_t* target_node_id,
                                  int64_t* alter_target_node_id);

  bool IsSpeedInRange(const agent::Agent* agent, const double min_speed_kmh,
                      const double max_speed_kmh);

  int64_t FindMatchingNodeId(const agent::Agent* agent,
                             const double target_agent_s);

 private:
  DeciderState current_state_ = DeciderState::IDLE;
  int32_t running_frame_count_ = 0;
  int32_t cooldown_frame_count_ = 0;
  int32_t exit_condition_frame_count_ = 0;

  double lateral_distance_ = -1.0;
  bool is_need_overtake_ = false;
  bool is_need_yield_ = false;
  bool is_lead_and_target_is_truck_ = false;
  bool is_running_longitudinal_avoid_ = false;
  bool is_right_agent_ = false;
  bool is_satisfied_speed_range_ = false;

  int64_t parallel_target_agent_id_ = -1;
  int64_t start_running_agent_id_ = -1;
  double front_edge_to_rear_axle_ = 0.0;
  double rear_edge_to_rear_axle_ = 0.0;

  framework::Session* session_ = nullptr;
  const EgoPlanningConfigBuilder* config_builder_ = nullptr;
  std::string name_ = "ParallelLongitudinalAvoidDecider";
};

}  // namespace planning
