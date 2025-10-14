#pragma once

#include <memory>

#include "common/debug_info_log.h"
#include "framework/session.h"
#include "modules/common/rss_model/rss_model.h"
#include "modules/common/task_basic_types.h"
#include "modules/context/virtual_lane_manager.h"
#include "modules/tasks/task.h"
#include "modules/tasks/task_interface/ego_lane_road_right_decider_output.h"
#include "modules/tasks/task_interface/potential_dangerous_agent_decider_output.h"
#include "basic_types.pb.h"
#include "potential_dangerous_agent_decider_info.pb.h"
namespace planning {

class PotentialDangerousAgentDecider : public Task {
 public:
  PotentialDangerousAgentDecider() = default;
  PotentialDangerousAgentDecider(const EgoPlanningConfigBuilder* config_builder,
                                 framework::Session* session);
  virtual ~PotentialDangerousAgentDecider() = default;

  bool Execute() override;

 private:
  bool EstimateRiskLevel(std::shared_ptr<FrenetObstacle> obstacle_ptr,
                         PotentialDangerousAgentDeciderOutput* output);

  void EstimateAgentPosType(std::shared_ptr<FrenetObstacle> agent_ptr,
                            AgentPosType* output, double* lateral_distance,
                            double* longitudinal_distance);
  void LogDebugInfo();

 private:
  PotentialDangerousAgentDeciderConfig config_;
  std::shared_ptr<VirtualLaneManager> virtual_lane_manager_ = nullptr;
  EgoLaneRoadRightDeciderOutput ego_road_right_decision_;
  FrenetBoundary ego_frenet_boundary_;
  std::vector<std::shared_ptr<FrenetObstacle>> frenet_obstacles_{};
  RssModel::FrenetState ego_state_;
  planning::common::PotentialDangerousAgentDeciderInfo potential_dangerous_agent_decider_info_;
};

}  // namespace planning