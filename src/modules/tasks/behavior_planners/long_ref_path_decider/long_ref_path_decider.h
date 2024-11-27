#pragma once

#include "tasks/behavior_planners/long_ref_path_decider/bound_maker/bound_maker.h"
#include "tasks/behavior_planners/long_ref_path_decider/target_marker/target_maker.h"
#include "tasks/behavior_planners/long_ref_path_decider/weight_maker/weight_maker.h"
#include "tasks/task.h"
#include "tasks/task_interface/longitudinal_decider_output.h"

namespace planning {

class LongRefPathDecider : public Task {
 public:
  LongRefPathDecider(const EgoPlanningConfigBuilder *config_builder,
                     framework::Session *session);
  ~LongRefPathDecider() = default;

  bool Execute() override;

 public:
  void UpdateLonRefPath();

  void Reset();

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