#pragma once
#include "ego_planning_config.h"
#include "session.h"
#include "src/modules/common/status/status.h"
#include "trajectory1d/second_order_time_optimal_trajectory.h"
#include "trajectory1d/trajectory1d.h"
#include "trajectory1d/piecewise_jerk_acceleration_trajectory1d.h"
#include "planning_context.h"
#include "environmental_model.h"

namespace planning {

class BoundMaker {
 public:
  BoundMaker(const SpeedPlannerConfig& speed_planning_config,
             framework::Session* session);
  ~BoundMaker() = default;

  common::Status Run();

  double s_lower_bound(const double t) const;

  double s_upper_bound(const double t) const;

  double v_lower_bound(const double t) const;

  double v_upper_bound(const double t) const;

  double a_lower_bound(const double t) const;

  double a_upper_bound(const double t) const;

  double jerk_lower_bound(const double t) const;

  double jerk_upper_bound(const double t) const;

 private:
  void MakeAccBound();

  void MakeSBound();

  void MakeVBound();

  void MakeJerkBound();

  SecondOrderTimeOptimalTrajectory GenerateMaxAccelerationCurve() const;
  SecondOrderTimeOptimalTrajectory GenerateMaxDecelerationCurve() const;
  std::unique_ptr<Trajectory1d> MakeVirtualZeroAccCurve();

 private:
  SpeedPlannerConfig speed_planning_config_;
  framework::Session* session_;

  double dt_ = 0.0;
  double plan_time_ = 0.0;
  int32_t plan_points_num_ = 0.0;
  std::array<double, 3> init_lon_state_;
  std::vector<double> s_upper_bound_;
  std::vector<double> s_lower_bound_;
  std::vector<double> v_upper_bound_;
  std::vector<double> v_lower_bound_;
  std::vector<double> acc_upper_bound_;
  std::vector<double> acc_lower_bound_;
  std::vector<double> jerk_upper_bound_;
  std::vector<double> jerk_lower_bound_;

  double acc_upper_bound_with_speed_ = 0.0;
};

}  // namespace planning
