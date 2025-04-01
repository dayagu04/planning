#pragma once

#include <memory>

#include "apa_measure_data_manager.h"
#include "collision_detection/collision_detector_interface.h"
#include "common/speed/apa_speed_decision.h"
#include "geometry_math.h"
#include "parking_task.h"
#include "point_cloud_obstacle.h"

namespace planning {
namespace apa_planner {
// If bycle/car/human is passing, need add a stop decision.
// If planning/prediction path is collided, need add a stop decision.
// If path point is a end point, need add a stop decision.
// If slot change too much, add a stop decision? Need a discuss.
class ParkingStopDecider : public ParkingTask {
 public:
  ParkingStopDecider(
      const std::shared_ptr<apa_planner::CollisionDetectorInterface>&
          col_det_interface_ptr,
      const std::shared_ptr<apa_planner::ApaMeasureDataManager>&
          measure_data_ptr) {
    SetCollisionDetectorIntefacePtr(col_det_interface_ptr);
    SetMeasureDataManagerPtr(measure_data_ptr);
  }

  ~ParkingStopDecider() {}

  void SetCollisionDetectorIntefacePtr(
      const std::shared_ptr<apa_planner::CollisionDetectorInterface>&
          col_det_interface_ptr) {
    col_det_interface_ptr_ = col_det_interface_ptr;
  }

  void SetMeasureDataManagerPtr(
      const std::shared_ptr<apa_planner::ApaMeasureDataManager>&
          measure_data_ptr) {
    measure_data_ptr_ = measure_data_ptr;
  }

  /**
   * [out]:path, fill path distance info;
   * [out]:speed_decisions,fill stop decision info;
   */
  void Process(const double tracking_path_collision_dist,
               std::vector<pnc::geometry_lib::PathPoint>& path,
               SpeedDecisions* speed_decisions);

  const double GetEgoPathProjectS() const { return ego_project_s_; }

  void AddDebugInfo(const std::vector<pnc::geometry_lib::PathPoint>& path);

 private:
  void AddStopDecisionByPlanningPath(
      std::vector<pnc::geometry_lib::PathPoint>& path,
      SpeedDecisions* speed_decisions);

  void AddStopDecisionByPredictedPath(
      const std::vector<pnc::geometry_lib::PathPoint>& path,
      const double tracking_path_collision_dist,
      SpeedDecisions* speed_decisions);

  void TaskDebug();

 private:
  double tracking_path_collision_dist_;
  double ego_project_s_;

  std::shared_ptr<apa_planner::CollisionDetectorInterface>
      col_det_interface_ptr_;

  std::shared_ptr<apa_planner::ApaMeasureDataManager> measure_data_ptr_;
};
}  // namespace apa_planner
}  // namespace planning