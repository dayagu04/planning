#pragma once

#include "behavior_planners/speed_limit_decider/speed_limit_decider_output.h"
#include "common/trajectory1d/piecewise_jerk_acceleration_trajectory1d.h"
#include "lon_target_maker.pb.h"
#include "session.h"
#include "target.h"

namespace planning {

class CruiseTarget : public Target {
 public:
  CruiseTarget(const SpeedPlannerConfig& config, framework::Session* session);
  ~CruiseTarget() = default;

  struct KinematicsBound {
    double acc_positive_mps2 = 0.0;
    double acc_negative_mps2 = 0.0;
    double jerk_positive_mps3 = 0.0;
    double jerk_negative_mps3 = 0.0;
  };

  enum class SpeedUpperBoundType : int32_t {
    kCurvature = 0,
    kClosePass = 1,
    kMaxType = 2,
  };

  enum class DrivingStyle {
    AGGRESIVE = 0,
    NORMAL = 1,
    CONSERVATIVE = 2,
  };

  struct SpeedLimitBoundInfo {
    double max_kappa = 0.0;
  };

 private:
  bool MakeKinematicsBound(
      const double ego_speed, const SpeedLimitType& speed_limit_type,
      KinematicsBound* const comfort_kinematic_bound) const;

  void MakeObstacleDistanceSpeedLimitTables();

  std::unique_ptr<PiecewiseJerkAccelerationTrajectory1d> MakeTarget(
      const double ref_speed, const SpeedLimitType& ref_speed_limit_type);

  double CalculateAccelerationWithinBound(
      const double a_next, const double a_t, const double t_step_length,
      const KinematicsBound& kinematics_bound) const;

  double MakeSpeedUpperBound(const double s, const double t,
                             SpeedLimitBoundInfo* const speed_limit_bound_info);

  double CalculateSpeedUpperBoundWithPathCurvature(
      const double s, SpeedLimitBoundInfo* const speed_limit_bound_info) const;

  // This function is confusing!
  //   double CalculateSpeedUpperBoundWithClosePassAgent(
  //       const double s, const double relative_t,
  //       SpeedLimitBoundInfo* const speed_limit_bound_info) const;

  double MatchSpeedWithKappaSpeedLimitTable(
      const bool is_lane_change_execution, const double kappa,
      const DrivingStyle driving_style = DrivingStyle::NORMAL) const;
  void DetermineSideLaneType();

  bool MakeSpeedLimitKinematicTable(
      const double ego_speed,
      const SpeedLimitDeciderOutput& speed_limit_decider_output);

  void AddCruiseTargetDataToProto();

  bool CalcLowSpeedFollowAccAndJerk(double* acc, double* jerk);

 private:
  const std::vector<double> _LOW_SPEED_FOLLOW_ACC_BP{10.0, 21.0, 35.0};
  const std::vector<double> _LOW_SPEED_FOLLOW_ACC_V{0.50, 0.80, 1.35};

  const std::vector<double> _LOW_SPEED_FOLLOW_JERK_BP{15.0, 25.0, 35.0};
  const std::vector<double> _LOW_SPEED_FOLLOW_JERK_V{2.50, 2.20, 2.00};

  std::map<SpeedLimitType, KinematicsBound> speed_limit_kinematics_bound_table_;
  planning::common::CruiseTarget cruise_target_pb_;
};

}  // namespace planning