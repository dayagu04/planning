#pragma once
#include "planning_context.h"
#include "session.h"
#include "tasks/task.h"
#include "tasks/task_interface/lateral_obstacle_decider_output.h"

namespace planning {

class BaseLateralObstacleDecider : public Task {
 public:
  BaseLateralObstacleDecider(const EgoPlanningConfigBuilder *config_builder,
                             framework::Session *session);
  virtual ~BaseLateralObstacleDecider() = default;

  virtual bool Execute();

 protected:
  void ConstructPlanHistoryTraj(
      const std::shared_ptr<ReferencePath> &reference_path_ptr);
  planning::framework::Session *session_;
  std::shared_ptr<ReferencePath> reference_path_ptr_;
  LateralObstacleDeciderConfig config_;
  std::unordered_map<uint32_t, LatObstacleDecisionType> output_;
};
}  // namespace planning