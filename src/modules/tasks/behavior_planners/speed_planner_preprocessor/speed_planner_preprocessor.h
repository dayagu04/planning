#pragma once

#include "tasks/behavior_planners/speed_planner_preprocessor/bound_maker/bound_maker.h"
#include "tasks/behavior_planners/speed_planner_preprocessor/target_marker/target_maker.h"
#include "tasks/behavior_planners/speed_planner_preprocessor/weight_maker/weight_maker.h"
#include "tasks/task.h"
#include "tasks/task_interface/longitudinal_decider_output.h"

namespace planning {

class SpeedPlannerPreProcessor : public Task {
 public:
  SpeedPlannerPreProcessor(const EgoPlanningConfigBuilder *config_builder,
                           framework::Session *session);
  ~SpeedPlannerPreProcessor() = default;

  bool Execute() override;

 public:
  void UpdateLonRefPath();

 private:
  SpeedPlannerConfig speed_planning_config_;
  std::unique_ptr<TargetMaker> target_maker_;
  std::unique_ptr<BoundMaker> bound_maker_;
  std::unique_ptr<WeightMaker> weight_maker_;
  double dt_ = 0.0;
  double plan_time_ = 0.0;
  int32_t plan_points_num_ = 0.0;

  LongitudinalDeciderOutput lon_behav_output_;
};

}  // namespace planning