#pragma once

#include <memory>

#include "agent/agent.h"
#include "agent/agent_manager.h"
#include "agent_headway_decider_output.h"
#include "ego_planning_config.h"
#include "ego_state_manager.h"
#include "st_graph/st_graph_helper.h"
#include "tasks/task.h"

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

  void MatchHeadwayWithGearTable(double* const desired_headway) const;

  bool IsNeighborTargetValid(const speed::StGraphHelper* st_graph_helper) const;

  double CalcAgentInitHeadway(
      const std::shared_ptr<EgoStateManager>& ego_state_manager,
      const agent::Agent* agent);

 private:
  std::unordered_map<int32_t, AgentHeadwayInfo> agents_headway_map_;
  AgentHeadwayConfig config_;
  int32_t plan_points_num_ = 0;
  double plan_time_ = 0.0;
  double dt_ = 0.0;
};

}  // namespace planning