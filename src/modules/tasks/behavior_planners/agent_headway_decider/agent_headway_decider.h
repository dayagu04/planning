#pragma once

#include <cstdint>
#include <memory>

#include "agent/agent.h"
#include "agent/agent_manager.h"
#include "agent_headway_decider_output.h"
#include "ego_planning_config.h"
#include "ego_state_manager.h"
#include "st_graph/st_graph_helper.h"
#include "tasks/task.h"
#include "trajectory1d/piecewise_jerk_acceleration_trajectory1d.h"

namespace planning {

enum class DrivingStyle { AGGRESIVE, NORMAL, CONSERVATIVE };

class AgentHeadwayDecider : public Task {
 public:
  AgentHeadwayDecider(const EgoPlanningConfigBuilder* config_builder,
                      framework::Session* session);
  virtual ~AgentHeadwayDecider() = default;

  bool Execute() override;

  void Reset();

 private:
  bool UpdateAgentsHeadwayInfos();

  void MatchHeadwayWithGearTable(double& desired_headway) const;

  void CalculateTHWInLaneChange(const int32_t gap_front_agent_id,
                                const double thw_request);

  bool CalculateTHWInLaneChangeToLaneKeep(const double thw_request);

  bool IsNeighborTargetValid(const speed::StGraphHelper* st_graph_helper) const;

  double CalcAgentInitHeadway(
      const std::shared_ptr<EgoStateManager>& ego_state_manager,
      const agent::Agent* agent);

  double CalculateCutinHeadway(const agent::Agent* agent,
                               const double ego_velocity,
                               const double current_headway);

  int32_t GetOriginLaneFrontAgentId();

  std::unique_ptr<Trajectory1d> MakeVirtualZeroAccCurve();

 private:
  std::unordered_map<int32_t, AgentHeadwayInfo> agents_headway_map_;
  AgentHeadwayConfig config_;
  pnc::filters::SlopeFilter thw_lane_change_slope_filter_;
  int32_t plan_points_num_ = 0;
  double plan_time_ = 0.0;
  int32_t last_gap_front_agent_id_ = -1;
  double dt_ = 0.0;
  bool lc_to_lk_thw_is_init_ = false;
};

}  // namespace planning