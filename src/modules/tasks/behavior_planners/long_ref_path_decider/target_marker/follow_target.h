#pragma once

#include <cstdint>
#include <memory>
#include "ego_planning_config.h"
#include "lon_target_maker.pb.h"
#include "session.h"
#include "target.h"
#include "trajectory1d/second_order_time_optimal_trajectory.h"
#include "trajectory1d/variable_coordinate_time_optimal_trajectory.h"

namespace planning {

class FollowTarget : public Target {
  struct UpperBoundInfo {
    double s = 0.0;
    double t = 0.0;
    double v = 0.0;
    TargetType target_type = TargetType::kNotSet;
    int32_t agent_id = -1;
    int64_t st_boundary_id = -1;
  };

 public:
  FollowTarget(const SpeedPlannerConfig config, framework::Session* session);
  ~FollowTarget() = default;

  void Update();

  double MakeSlowerFollowSTarget(const double speed, const double upper_bound_s,
                                 const double time_gap) const;

 private:
  void GenerateUpperBoundInfo();

  void GenerateFollowTarget();

  void MakeMinFollowDistance();

  // generate stable curve
  std::shared_ptr<VariableCoordinateTimeOptimalTrajectory>
  GenerateStableFollowSlowCurve(const double matched_desired_headway, const bool enable_stable_jlt) const;

  // generate far slow curve
  std::shared_ptr<VariableCoordinateTimeOptimalTrajectory>
  GenerateFarFollowSlowCurve(const bool enable_far_slow_jlt) const;

  VariableCoordinateTimeOptimalTrajectory MakeSafeFarSlowCurve(
      const CoordinateParam& relative_coordinate_param) const;

  bool IsSafeFarSlowCurve(
      const VariableCoordinateTimeOptimalTrajectory& far_slow_curve) const;

  bool GenerateRelativeCoordinate(
      const double follow_time_gap,
      CoordinateParam* const relative_coordinate_param) const;

  bool GenerateFarSlowCarRelativeCoordinate(
      const int64_t st_boundary_id, const double follow_time_gap,
      CoordinateParam* const relative_coordinate_param) const;

  bool MakeSValueWithTargetFollowCurve(const int32_t index,
                                       const bool has_valid_s_value,
                                       double* const target_s_value) const;

  bool JudgeStableCar(const double matched_desired_headway) const;

  bool JudgeFarSlowCar(const double matched_desired_headway) const;

  bool JudgeSrefValid(
      std::shared_ptr<VariableCoordinateTimeOptimalTrajectory> jlt_curve) const;

  void AddFollowTargetDataToProto();

 private:
  std::vector<UpperBoundInfo> upper_bound_infos_;
  planning::common::FollowTarget follow_target_pb_;
  bool enable_target_follow_curve_ = false;
  double min_follow_distance_m_ = 3.0;
};

}  // namespace planning