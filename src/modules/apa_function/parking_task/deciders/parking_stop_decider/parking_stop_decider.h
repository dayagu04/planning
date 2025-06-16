#pragma once

#include <memory>

#include "apa_measure_data_manager.h"
#include "collision_detection/collision_detector_interface.h"
#include "common/speed/apa_speed_decision.h"
#include "dp_speed_common.h"
#include "geometry_math.h"
#include "parking_stop_config.h"
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
          measure_data_ptr,
      const std::shared_ptr<ApaObstacleManager>& obs_manager_ptr)
      : col_det_interface_ptr_(col_det_interface_ptr),
        measure_data_ptr_(measure_data_ptr),
        obs_manager_(obs_manager_ptr) {}

  ~ParkingStopDecider() {}

  /**
   * [out]:path, fill path distance info;
   * [out]:speed_decisions,fill stop decision info;
   * This API will compute all spatial-temporal interaction relationship bettwen
   * ego and other agents.
   * But for these agents which have not st interaction
   * relationship, ego need add caution decision in speed_limit_decider.
   */
  void Execute(const SVPoint& init_point,
               const std::vector<pnc::geometry_lib::PathPoint>& lateral_path,
               const std::vector<pnc::geometry_lib::PathPoint>& control_path,
               const pnc::geometry_lib::PathSegGear gear);

  void AddStopDecisionByDistance(
      const double stop_s, const LonDecisionReason decision_reason,
      const std::vector<pnc::geometry_lib::PathPoint>& lateral_path);

  const ParkLonDecision& GetStopDecision() const { return stop_decision_; }

  const double GetStopDecisionS();

 private:
  void AddStopDecisionByControlPath(
      const std::vector<pnc::geometry_lib::PathPoint>& path,
      const double stop_fence_s, SpeedDecisions* speed_decisions);

  void AddDecisionByPathTargetPoint(
      const std::vector<pnc::geometry_lib::PathPoint>& path);

  void AddDecisionByObstacle(
      const std::vector<pnc::geometry_lib::PathPoint>& path,
      const bool check_extend_path);

  void TaskDebug();

  bool IsVehComponentCollision(const Polygon2D* polygon,
                               const ApaObstacle& obs);

  bool IsVehCollision(const PolygonFootPrint& foot_print,
                      const ApaObstacle& obs);

  void ExtendPath(const double extend_length,
                  std::vector<pnc::geometry_lib::PathPoint>& path);

  void GeneratePathFootPrint(
      const std::vector<pnc::geometry_lib::PathPoint>& path);

  bool GetPathCollisionByObstacle(int* collision_index, const ApaObstacle& obs);

  bool GetOverlapBoundaryPoints(
      const std::vector<pnc::geometry_lib::PathPoint>& path,
      const ApaObstacle& obstacle, std::vector<STPoint>* upper_points,
      std::vector<STPoint>* lower_points);

  void ComputeSTBoundary(std::vector<pnc::geometry_lib::PathPoint>& path,
                         ApaObstacle& obstacle);

  void PathDebug(const std::vector<pnc::geometry_lib::PathPoint>& path);

  void RecordDebugInfo(
      const std::vector<pnc::geometry_lib::PathPoint>& lateral_path);

 private:
  std::shared_ptr<apa_planner::CollisionDetectorInterface>
      col_det_interface_ptr_;

  std::shared_ptr<apa_planner::ApaMeasureDataManager> measure_data_ptr_;

  GJK2DInterface gjk_interface_;

  std::shared_ptr<apa_planner::ApaObstacleManager> obs_manager_;

  std::shared_ptr<PathSafeChecker> path_safe_check_;

  std::vector<PolygonFootPrint> little_buffer_path_polygons_;
  std::vector<PolygonFootPrint> big_buffer_path_polygons_;

  const ApaObstacle* stop_obstacle_ = nullptr;
  ParkLonDecision stop_decision_;
  SVPoint init_point_;

  ParkStopConfig config_;

  pnc::geometry_lib::PathSegGear gear_;
};
}  // namespace apa_planner
}  // namespace planning