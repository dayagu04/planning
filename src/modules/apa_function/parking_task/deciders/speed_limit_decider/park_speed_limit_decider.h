#pragma once

#include "apa_obstacle_manager.h"
#include "common/speed/apa_speed_decision.h"
#include "park_speed_limit_config.h"
#include "parking_task.h"
#include "point_cloud_obstacle.h"
#include "speed_limit_profile.h"

namespace planning {
namespace apa_planner {

// 限速来源：
// 1. path kappa switch
// 2. gap too much bettwen car steering wheel kappa with path kappa.
// 3. obstacle distance
// 4. path kappa too large
// 5. vehicle distance to ref path;
// 6. speed bump;
class ParkSpeedLimitDecider : public ParkingTask {
 public:
  ParkSpeedLimitDecider(
      const std::shared_ptr<apa_planner::CollisionDetectorInterface>&
          col_det_interface_ptr,
      const std::shared_ptr<apa_planner::ApaMeasureDataManager>&
          measure_data_ptr,
      const std::shared_ptr<ApaObstacleManager>& obs_manager_ptr)
      : col_det_interface_ptr_(col_det_interface_ptr),
        measure_data_ptr_(measure_data_ptr),
        obs_manager_(obs_manager_ptr) {}

  ~ParkSpeedLimitDecider() = default;

  /**
   * [in]:
   * [out]: path,speed_decisions;
   */
  void Execute(std::vector<pnc::geometry_lib::PathPoint>& path,
               SpeedDecisions* speed_decisions);

  // Get minimum speed limit decision
  const ParkLonDecision* GetSpeedLimitDecisionBySRange(
      const SpeedDecisions* speed_decisions, const double start_s,
      const double end_s) const;

  /**
   * [input] ego v: maybe negative
   * todo: temporary strategy
   * 直接给控制模块发送跳变的参考速度，控制难以处理.
   * 所以将限速决策增加一个速度平滑.
   */
  const double CalcRefSpeedBySpeedLimitDecision(
      const double ego_v, const double ego_s, const ParkLonDecision* decision);

  const SpeedLimitProfile& GetSpeedLimitProfile() const {
    return speed_limit_profile_;
  }

  const double GetCruiseSpeed() const { return config_.default_cruise_speed_; }

  // TODO: compute caution decision by obstacle distance is not good. We need
  // use temporal-spatial information.
  void AddSpeedLimitDecisions(
      const std::vector<pnc::geometry_lib::PathPoint>& path,
      SpeedDecisions* speed_decisions);

  void PublishDebugInfo(const std::vector<pnc::geometry_lib::PathPoint>& path);

  void TaskDebug(const std::vector<pnc::geometry_lib::PathPoint>& path);

 private:
  std::shared_ptr<apa_planner::CollisionDetectorInterface>
      col_det_interface_ptr_;
  std::shared_ptr<apa_planner::ApaMeasureDataManager> measure_data_ptr_;
  std::shared_ptr<apa_planner::ApaObstacleManager> obs_manager_;

  SpeedLimitProfile speed_limit_profile_;

  ParkSpeedLimitConfig config_;
};
}  // namespace apa_planner
}  // namespace planning