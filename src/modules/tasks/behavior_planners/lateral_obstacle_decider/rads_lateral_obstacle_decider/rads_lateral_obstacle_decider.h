#pragma once

#include "tasks/behavior_planners/lateral_obstacle_decider/base_lateral_obstacle_decider.h"
#include "tasks/task.h"

namespace planning {

class RADSLateralObstacleDecider : public BaseLateralObstacleDecider {
 public:
  RADSLateralObstacleDecider(const EgoPlanningConfigBuilder *config_builder,
                             framework::Session *session);
  virtual ~RADSLateralObstacleDecider() = default;

  bool Execute() override;

 private:
  void InitInfo();

  void UpdateLatDecision();

  void GenerateOutput();

  void Log();

 private:
  std::unordered_map<uint32_t, LatObstacleDecisionType> last_output_;
};

}  // namespace planning