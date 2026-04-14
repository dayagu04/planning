#pragma once

#include <cstdint>
#include <memory>

#include "agent/agent.h"
#include "ego_planning_config.h"
#include "environmental_model_manager.h"
#include "common/constraint_check/collision_checker.h"
#include "memory"
#include "tasks/task.h"
#include "virtual_lane.h"

namespace planning {

class StopDestinationDecider : public Task {
 public:
  StopDestinationDecider(const EgoPlanningConfigBuilder *config_builder,
                         framework::Session *session);
  virtual ~StopDestinationDecider() = default;

  void StopDestinationProcess();

  bool AddVirtualObstacle();

  bool Execute() override;

  void SaveToSession();

 private:
  StopDestinationDeciderConfig config_;
  std::shared_ptr<VirtualLane> current_lane_;
  double stop_destination_virtual_agent_time_headway_ = 1.0;
  int32_t stop_destination_virtual_agent_id_ =
      agent::AgentDefaultInfo::kNoAgentId;
  std::shared_ptr<planning_math::CollisionChecker> lon_collision_checker_;
  std::vector<double> rads_bound_s_by_collision_check_;
};

}  // namespace planning
