#pragma once
#include "ego_planning_config.h"
#include "session.h"
#include "src/modules/common/status/status.h"
#include "tasks/behavior_planners/long_ref_path_decider/target_marker/target_maker.h"

namespace planning {

class WeightMaker {
 public:
  WeightMaker(const SpeedPlannerConfig& speed_planning_config,
              framework::Session* session, const TargetMaker& target_maker);
  ~WeightMaker() = default;

  common::Status Run();

 private:
  SpeedPlannerConfig speed_planning_config_;
  framework::Session* session_;

  double dt_ = 0.0;
  double plan_time_ = 0.0;
  int32_t plan_points_num_ = 0.0;
  std::vector<double> s_weight_;
  std::vector<double> v_weight_;
  std::vector<double> acc_weight_;
  std::vector<double> jerk_weight_;
};

}  // namespace planning
