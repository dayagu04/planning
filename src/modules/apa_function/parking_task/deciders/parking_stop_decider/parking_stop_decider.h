#pragma once

#include "parking_task.h"
#include "common/speed/apa_speed_decision.h"
#include "geometry_math.h"
#include "point_cloud_obstacle.h"
#include "apa_world.h"

namespace planning {

// If bycle/car/human is passing, need add a stop decision.
// If planning/prediction path is collided, need add a stop decision.
// If path point is a end point, need add a stop decision.
// If slot change too much, add a stop decision? Need a discuss.
class ParkingStopDecider : public ParkingTask {
 public:
  ParkingStopDecider() = default;

  /**
   * [out]:path, fill path distance info;
   * [out]:speed_decisions,fill stop decision info;
   */
  void Process(const ParkObstacleList& obstacles,
               const std::shared_ptr<apa_planner::ApaWorld> apa_world_ptr,
               const double tracking_path_collision_dist,
               std::vector<pnc::geometry_lib::PathPoint>& path,
               SpeedDecisions* speed_decisions);

  const double GetEgoPathProjectS() const { return ego_project_s_; }

 private:
  void AddStopDecisionByPlanningPath(
      std::vector<pnc::geometry_lib::PathPoint>& path,
      SpeedDecisions* speed_decisions);

  void AddDebugInfo(const std::vector<pnc::geometry_lib::PathPoint>& path);

  void AddStopDecisionByPredictedPath(
      const std::vector<pnc::geometry_lib::PathPoint>& path,
      const double tracking_path_collision_dist,
      SpeedDecisions* speed_decisions);

  void TaskDebug();

 private:
  double tracking_path_collision_dist_;
  double ego_project_s_;
};

}  // namespace planning