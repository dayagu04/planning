#pragma once

#include "lon_behavior_planner.pb.h"
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
  void Reset();

 private:
  void UpdateLonRefPath();

  void SaveToSession();

  void SaveToDebugInfo();

  void ClearOutput();

 private:
  SpeedPlannerConfig speed_planning_config_;
  std::unique_ptr<TargetMaker> target_maker_;
  std::unique_ptr<BoundMaker> bound_maker_;
  std::unique_ptr<WeightMaker> weight_maker_;
  double dt_ = 0.0;
  double plan_time_ = 0.0;
  int32_t plan_points_num_ = 0.0;

  LongitudinalDeciderOutput lon_behavior_output_;
  planning::common::LonRefPath lon_behavior_output_pb_;
};

}  // namespace planning