#pragma once

#include "apa_obstacle_manager.h"
#include "common/speed/apa_speed_decision.h"
#include "parking_task.h"
#include "point_cloud_obstacle.h"
#include "speed_limit_profile.h"

namespace planning {

// 限速来源：
// 1. path kappa change
// 2. gap too much bettwen car steering wheel kappa with path kappa.
// 3. obstacle distance
// 4. path kappa too large
class ParkSpeedLimitDecider : public ParkingTask {
 public:
  ParkSpeedLimitDecider();

  ~ParkSpeedLimitDecider() = default;

  void Process(const std::vector<pnc::geometry_lib::PathPoint>& path,
               SpeedDecisions* speed_decisions);

  // Get minimum speed limit decision
  const SpeedLimitDecision* GetSpeedLimitDecisionBySRange(
      const SpeedDecisions* speed_decisions, const double start_s,
      const double end_s) const;

  /**
   * [input] ego v: maybe negative
   * todo: temporary strategy
   * 直接给控制模块发送跳变的参考速度，控制难以处理.
   * 所以将限速决策增加一个速度平滑.
   */
  const double CalcRefSpeedBySpeedLimitDecision(
      const double ego_v, const double ego_s,
      const SpeedLimitDecision* decision);

  const SpeedLimitProfile& GetSpeedLimitProfile() const {
    return speed_limit_profile_;
  }

  const double GetCruiseSpeed() const { return default_cruise_speed_; }

 private:
  void AddSpeedLimitDecisions(
      const std::vector<pnc::geometry_lib::PathPoint>& path,
      SpeedDecisions* speed_decisions);

  StopDecision* GetCloseStopDecision(SpeedDecisions* speed_decisions);

  void PublishDebugInfo();

  void UpdateConfig();

 private:
  double default_cruise_speed_;
  double min_cruise_speed_;
  SpeedLimitProfile speed_limit_profile_;

  // todo: speed limit should designed to linear shape, not ladder shape.
  // parameter: if gap is big, ego need speed down.
  double kappa_gap_in_path_point_;
  double kappa_gap_speed_limit_;

  // kappa speed limit related
  double kappa_thresh_;
  double kappa_speed_limit_;

  double obs_dist_thresh_;
  double obs_dist_speed_limit_;

  // for keep a safe lon buffer with obstacles, stop decision need a lon buffer.
  double stop_decision_lon_buffer_ = 0.1;

  double acc_upper_;
  double acc_lower_;
  double jerk_upper_;
  double jerk_lower_;
};

}  // namespace planning