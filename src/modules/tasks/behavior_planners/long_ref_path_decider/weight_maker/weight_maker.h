#pragma once

#include "ego_planning_config.h"
#include "lon_target_maker.pb.h"
#include "session.h"
#include "src/modules/common/status/status.h"
#include "tasks/behavior_planners/long_ref_path_decider/target_marker/target_maker.h"
#include "trajectory1d/second_order_time_optimal_trajectory.h"

namespace planning {

class WeightMaker {
 public:
  WeightMaker(const SpeedPlannerConfig& speed_planning_config,
              framework::Session* session);
  ~WeightMaker() = default;

  common::Status Run(const TargetMaker& target_maker);

  void Reset();

  double s_weight(const double t) const;

  double v_weight(const double t) const;

  double a_weight(const double t) const;

  double jerk_weight(const double t) const;

 private:
  void MakeSWeight(const TargetMaker& target_maker);

  void MakeVWeight(const TargetMaker& target_maker);

  void MakeAccWeight();

  void MakeJerkWeight();

  std::unique_ptr<Trajectory1d> MakeVirtualZeroAccCurve();

  SecondOrderTimeOptimalTrajectory GenerateMaxDecelerationCurve();

  void CollectDataToProto(const TargetMaker& target_maker);

  SpeedPlannerConfig speed_planning_config_;
  std::array<double, 3> init_lon_state_;
  framework::Session* session_;
  double dt_ = 0.0;
  double plan_time_ = 0.0;
  int32_t plan_points_num_ = 0.0;
  std::vector<double> s_weight_;
  std::vector<double> v_weight_;
  std::vector<double> acc_weight_;
  std::vector<double> jerk_weight_;
  bool is_urgent_ = false;
  bool is_more_urgent_ = false;
  planning::common::WeightMakerReplayInfo weight_maker_replay_info_;
};

}  // namespace planning
