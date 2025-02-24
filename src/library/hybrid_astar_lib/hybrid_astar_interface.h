#pragma once

#include <memory>
#include <vector>

#include "src/library/convex_collision_detection/gjk2d_interface.h"
#include "src/library/occupancy_grid_map/point_cloud_obstacle.h"
#include "Eigen/Core"
#include "hybrid_a_star.h"
#include "node3d.h"
#include "polygon_base.h"
#include "pose2d.h"
#include "target_pose_regulator.h"

namespace planning {

// todo: use float to replace double.
class HybridAStarInterface {
 public:
  HybridAStarInterface();

  ~HybridAStarInterface();

  // todo: need to unify same vehicle chassis params for on lane driving and
  // parking
  int Init(const double back_edge_to_rear_axis, const double car_length,
           const double car_width, const double steer_ratio,
           const double wheel_base, const double min_turn_radius,
           const double mirror_width);

  // for now, use slot coordinate. you can call this API in one thread.
  void GeneratePath(const Eigen::Vector3d& start, const Eigen::Vector3d& end,
                    const ParkObstacleList& obs_list,
                    const AstarRequest& request);

  const AstarSearchState GetFullLengthPath(HybridAStarResult* result);

  AstarRequest* GetMutableRequest() { return &request_; }

  const AstarRequest& GetConstRequest() const { return request_; }

  static void GetRSPathInFullPath(std::vector<double>& x,
                                  std::vector<double>& y,
                                  std::vector<double>& phi,
                                  const HybridAStarResult& result);

  const HybridAStarResult& GetConstFullLengthPath() const;

  const bool GetFirstSegmentPath(std::vector<AStarPathPoint>& result);

  static const AstarSearchState TransformFirstSegmentPath(
      std::vector<AStarPathPoint>& result, const HybridAStarResult& full_path,
      const Pose2D& start);

  const int GetFallBackPath(std::vector<AStarPathPoint>& result);

  const int GetFallBackPath(HybridAStarResult* result);

  int ExtendPathToRealTargetPose(const Pose2D& real_end);

  const ParkObstacleList& GetConstObstacles() const;

  ParkObstacleList& GetMutableObstacleList();

  const Pose2D GetAstarTargetPose() const { return target_regulator_goal_; }

  // multi-thread, input
  void UpdateInput(const ParkObstacleList& obs_list,
                  const AstarRequest& request);

  // multi-thread, output
  void UpdateOutput();

  const EulerDistanceTransform* GetEulerDistanceTransform() const {
    return &edt_;
  }

  EulerDistanceTransform* GetMutableEDT() { return &edt_; }

 public:
  // for debug
  void GetRSPathHeuristic(
      std::vector<std::vector<ad_common::math::Vec2d>>& path_list);

  // for debug
  const std::vector<DebugAstarSearchPoint>& GetChildNodeForDebug();

  // for debug
  const std::vector<ad_common::math::Vec2d>& GetPriorQueueNode();
  const std::vector<ad_common::math::Vec2d>& GetDelNodeQueueNode();
  // for debug,retired
  void GetNodeListMessage(planning::common::AstarNodeList* list);

  // for debug
  void GetNodeListMessage(std::vector<std::vector<Eigen::Vector2d>>& list);

  // for debug
  void GetRSPathForDebug(std::vector<double>& x, std::vector<double>& y,
                         std::vector<double>& phi);

  const ParkReferenceLine& GetConstRefLine() const;

  // for debug
  void GetPolynomialPathForDebug(std::vector<double>& x, std::vector<double>& y,
                                 std::vector<double>& phi);

  void UpdateEDTByObs(const ParkObstacleList& obs_list);

  FootPrintCircleModel* GetSlotOutsideCircleFootPrint();

 private:
  // if ego pose is good, seleted real end is ok
  const bool IsSelectedRealTargetPose() const;

  int UpdateEDT();

  void ExtendPathToRealParkSpacePoint(HybridAStarResult* result,
                                      const Pose2D& real_end);

  void PathClear(HybridAStarResult* path);

  void UpdateSearchBoundary();

  void UpdateEDTBasePose(Pose2D& ogm_base_pose);

  const Pose2D& GetStartPoint();

  const Pose2D& GetGoalPoint();

  const bool IsEgoOverlapWithSlot();

  // 基于采样的揉库API
  void PathSamplingForScenarioRunning();

  void PathSearchForScenarioTry(const TargetPoseRegulator& regulator);

  // 基于搜索的路径生成API
  void PathSearchForScenarioRunning(const TargetPoseRegulator& regulator,
                                    const double ego_obs_dist,
                                    const bool is_ego_overlap_with_slot);

 private:
  // read vehicle param from file
  VehicleParam vehicle_param_;
  PlannerOpenSpaceConfig config_;
  // xmin, xmax, ymin, ymax
  MapBound map_bounds_;

  std::shared_ptr<HybridAStar> hybrid_astar_;
  // path = astar node path + rs path.
  HybridAStarResult coarse_traj_;

  Pose2D ego_state_;
  // 对于垂直、平行车位，goal_state位于车位中心线上.
  // 位姿调节器依赖这个pose重新计算搜索目标点.
  Pose2D goal_state_;
  // 目标调节器会计算一个合适的目标
  Pose2D target_regulator_goal_;

  AstarSearchState search_state_;

  AstarRequest request_;

  ParkObstacleList obs_;

  ObstacleClearZone clear_zone_;

  OccupancyGridMap ogm_;
  EulerDistanceTransform edt_;
  ParkReferenceLine ref_line_;
};

}  // namespace planning