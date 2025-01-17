#pragma once

#include <cstdint>
#include <map>
#include <utility>

#include "common/agent/agent_manager.h"
#include "src/modules/common/agent/agent.h"
#include "st_graph/st_graph_helper.h"
#include "task_basic_types.h"
#include "tasks/task.h"
#include "trajectory1d/second_order_time_optimal_trajectory.h"
#include "virtual_lane_manager.h"

namespace planning {

class LongitudinalDecisionDecider : public Task {
 public:
  LongitudinalDecisionDecider(const EgoPlanningConfigBuilder *config_builder,
                              framework::Session *session);
  ~LongitudinalDecisionDecider() override = default;

  bool Execute() override;

  void Reset();  // 后续考虑抽象到Task中

 private:
  void DetermineKinematicBoundForCruiseScenario();

  void UpdateInvadeNeighborResults();

  // only consider lane change execution stage
  void UpdateLaneChangeNeighborResults();

  bool ConstructNeighborLaneStGraph(const agent::Agent *const neighbor_agent);

  void MakeDebugMessage();

  double CalculateAgentsAverageSpeedAroundEgo() const;

  bool IsMaxAccCurvSafeInStGraph() const;

  SecondOrderTimeOptimalTrajectory GenerateMaxDecelerationCurve(
      const PlanningInitPoint &init_point) const;

  bool IgnoreLaneChangeGapRearAgent(const agent::Agent *gap_rear_agent,
                                    const std::shared_ptr<planning_math::KDPath>
                                        &target_lane_frenet_coord) const;

  void DetermineClosestInvadeNeighborGapInfo(
      const std::shared_ptr<VirtualLane> &ego_cur_lane,
      const double planning_init_x, const double planning_init_y,
      const std::unordered_map<uint32_t, LatObstacleDecisionType>
          &lat_obstacle_decision,
      const std::set<int32_t> &lane_borrow_blocked_obs_id_set,
      const std::shared_ptr<agent::AgentManager> &agent_manager,
      const speed::StGraphHelper *st_graph_helper);

 private:
  LongitudinalDecisionDeciderConfig config_;
  // <counter, flag>
  std::pair<int32_t, int32_t> cruise_accelerate_count_{0, 0};
  int32_t plan_points_num_ = 26;
  double plan_time_ = 5.0;
  double dt_ = 0.2;

  static constexpr double kCruiseSpeedMinThd = 60.0 / 3.6;
  static constexpr double kEgoSpeedWithCruiseSpeedDiffThd = 15.0 / 3.6;
  static constexpr double kEgoPreviewTimeThd = 6.0;
  static constexpr double kPreviewDistanceStep = 2.0;
  static constexpr double kMaxCurvThd = 0.01;
  static constexpr double kAgentsAverageSpeedRatioByCruiseThd = 0.7;
  static constexpr int32_t kIncreaseAccBoundCountThd = 3;
  static constexpr double kCruiseAccelerateThd = 1.0;

  static constexpr double kAroundEgoLateralDistanceThd = 5.4;
  static constexpr double kAroundEgoLongitudinalPreviewTimeThd = 3.0;
  static constexpr double kAroundEgoLongitudinalBackwardTimeThd = 1.0;

  // closet gap of neighbor invade agents
  // first: lower agent id, second: upper agent id
  std::pair<int32_t, int32_t> closest_neighbor_invade_gap_agents_id_{-1, -1};
  bool has_lon_decision_to_invade_agents_{false};
};

}  // namespace planning
