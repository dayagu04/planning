#pragma once

#include "behavior_planners/long_ref_path_decider/target_marker/target.h"
#include "behavior_planners/long_ref_path_decider/target_marker/target_maker.h"
#include "ego_planning_config.h"
#include "environmental_model.h"
#include "planning_context.h"
#include "session.h"
#include "src/modules/common/status/status.h"
#include "trajectory1d/piecewise_jerk_acceleration_trajectory1d.h"
#include "trajectory1d/second_order_time_optimal_trajectory.h"
#include "trajectory1d/trajectory1d.h"

namespace planning {

class BoundMaker {
 public:
  BoundMaker(const SpeedPlannerConfig& speed_planning_config,
             framework::Session* session);
  ~BoundMaker() = default;

  common::Status Run(const TargetMaker& target_maker);

  double s_lower_bound(const double t) const;

  double s_upper_bound(const double t) const;

  double v_lower_bound(const double t) const;

  double v_upper_bound(const double t) const;

  double a_lower_bound(const double t) const;

  double a_upper_bound(const double t) const;

  double jerk_lower_bound(const double t) const;

  double jerk_upper_bound(const double t) const;

  struct UpperBoundInfo {
    double s = 0.0;
    double t = 0.0;
    double v = 0.0;
    double a = 0.0;
    double d_path = std::numeric_limits<double>::max();
    double d_rel = std::numeric_limits<double>::max();
    int32_t agent_id = -1;
  };

 private:
  void MakeAccBound();

  void MakeAccBound(const double& v_ego, const std::string& lc_request);

  void MakeSBound();

  void MakeVBound();

  void MakeJerkBound(const TargetMaker& target_maker);

  double GetCalibratedDistance(const double v_lead, const double v_ego,
                               const std::string& lc_request);

  double CalcDesiredVelocity(const double d_rel, const double d_des,
                             const double v_lead, const double v_ego);

  void CalcAccLimits(const UpperBoundInfo& upper_bound_info,
                     const double desired_distance, const double v_target,
                     const double v_ego, const double lead_one_a_processed,
                     std::pair<double, double>* acc_target);

  double CalcPositiveAccLimit(const double v_ego, const double v_rel,
                              const double a_max_const);

  double CalcCriticalDecel(const double d_lead, const double v_rel,
                           const double d_offset, const double v_offset);

  void GenerateUpperBoundInfo();

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

  std::vector<UpperBoundInfo> upper_bound_infos_;
  double min_follow_distance_m_ = 3.0;

  const double _A_MAX = 1.5;
  planning::common::MaxDecelTarget max_decel_target_pb_;

  const double _A_MIN = -4.0;
  const std::vector<double> _L_SLOPE_BP{0.0, 40.0};
  const std::vector<double> _L_SLOPE_V{0.35, 0.08};
  const std::vector<double> _P_SLOPE_BP{0., 40.0};
  const std::vector<double> _P_SLOPE_V{0.8, 0.2};
  const std::vector<double> _T_GAP_VEGO_BP{5.0, 15.0, 30.0};
  const std::vector<double> _T_GAP_VEGO_V{1.35, 1.55, 2.0};
  const std::vector<double> _A_CRUISE_MIN_BP{0.0, 5.0, 10.0, 20.0, 40.0};
  const std::vector<double> _A_CRUISE_MIN_V{-1.5, -1.5, -1.5, -1.0, -0.3};
  const std::vector<double> _A_CRUISE_MAX_BP{0.0, 5.0, 10.0, 20.0, 40.0};
  const std::vector<double> _A_CRUISE_MAX_V{1.0, 0.85, 0.6, 0.5, 0.3};
  const std::vector<double> _A_LEAD_LOW_SPEED_BP{0.0, 5.0};
  const std::vector<double> _A_LEAD_LOW_SPEED_V{0.0, 1.0};
  const std::vector<double> _A_LEAD_DISTANCE_BP{50.0, 100.0};
  const std::vector<double> _A_LEAD_DISTANCE_V{1.0, 0.5};
  const std::vector<double> _DECEL_OFFSET_BP{0.0, 4.0, 15.0, 30.0, 40.0};
  const std::vector<double> _DECEL_OFFSET_V{-0.3, -0.5, -0.5, -0.4, -0.3};
  const std::vector<double> _A_CORR_BY_SPEED_BP{0.0, 2.0, 10.0};
  const std::vector<double> _A_CORR_BY_SPEED_V{0.4, 0.4, 0.0};
};

}  // namespace planning
