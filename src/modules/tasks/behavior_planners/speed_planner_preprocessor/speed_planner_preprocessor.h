#pragma once

#include "tasks/behavior_planners/speed_planner_preprocessor/bound_maker/bound_maker.h"
#include "tasks/behavior_planners/speed_planner_preprocessor/target_marker/target_maker.h"
#include "tasks/behavior_planners/speed_planner_preprocessor/weight_maker/weight_maker.h"
#include "tasks/task.h"

namespace planning {

class SpeedPlannerPreProcessor : public Task {
 public:
  SpeedPlannerPreProcessor(const EgoPlanningConfigBuilder *config_builder,
                           framework::Session *session);
  ~SpeedPlannerPreProcessor() = default;

  bool Execute() override;
  

 private:
  std::unique_ptr<TargetMaker> target_maker_;
  std::unique_ptr<BoundMaker> bound_maker_;
  std::unique_ptr<WeightMaker> weight_maker_;
};

}  // namespace planning