#pragma once
#include "ego_planning_config.h"
#include "session.h"
#include "src/modules/common/status/status.h"
#include "tasks/behavior_planners/speed_planner_preprocessor/target_marker/target_maker.h"

namespace planning {

class WeightMaker {
 public:
  WeightMaker(const SpeedPlannerConfig& speed_planning_config,
              framework::Session *session, const TargetMaker &target_maker);
  ~WeightMaker() = default;

  common::Status Run();

 private:
};

}  // namespace planning
