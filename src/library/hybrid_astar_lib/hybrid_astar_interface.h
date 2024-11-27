#pragma once

#include <memory>
#include <vector>

#include "./../collision_detection/gjk2d_interface.h"
#include "./../occupancy_grid_map/point_cloud_obstacle.h"
#include "Eigen/Core"
#include "ad_common/math/line_segment2d.h"
#include "hybrid_a_star.h"
#include "log_glog.h"
#include "node3d.h"
#include "polygon_base.h"
#include "pose2d.h"

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
  int GeneratePath(const Eigen::Vector3d& start, const Eigen::Vector3d& end,
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

  const Pose2D GetAstarTargetPose() const { return goal_state_; }

  // multi-thread, input
  int UpdateInput(const ParkObstacleList& obs_list,
                  const AstarRequest& request);

  // multi-thread, output
  int UpdateOutput();

  const EulerDistanceTransform* GetEulerDistanceTransform() const {
    return &edt_;
  }

  void SwapStartGoal() {
    Pose2D tmp = initial_state_;
    initial_state_ = goal_state_;
    goal_state_ = tmp;

    return;
  }

  void UpdateReqeustBySwapStartGoal();

 public:
  // for debug
  void GetRSPathHeuristic(
      std::vector<std::vector<ad_common::math::Vec2d>>& path_list);

  // for debug
  const std::vector<DebugAstarSearchPoint>& GetChildNodeForDebug();

  // for debug
  const std::vector<ad_common::math::Vec2d>& GetPriorQueueNode();

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

 private:
  // if ego pose is good, seleted real end is ok
  const bool IsSelectedRealTargetPose() const;

  int UpdateEDT();

  int UpdateEDTByObs(const ParkObstacleList& obs_list);

  int ExtendPathToRealParkSpacePoint(HybridAStarResult* result,
                                     const Pose2D& real_end);

  void PathClear(HybridAStarResult* path);

  // if in vertial parking, check goal safe, if not safe, move goal to a safe
  // position.
  void AdjustGoalBySafeCheck(Pose2D* adjust_goal, const Pose2D& request_goal);

  void UpdateSearchBoundary();

  void UpdateEDTBasePose(Pose2D& ogm_base_pose);

 private:
  // read vehicle param from file
  VehicleParam vehicle_param_;

  PlannerOpenSpaceConfig config_;

  // xmin, xmax, ymin, ymax
  MapBound map_bounds_;

  std::shared_ptr<HybridAStar> hybrid_astar_;
  // path = astar node path + rs path.
  HybridAStarResult coarse_traj_;

  Pose2D ego_pose_;
  Pose2D initial_state_;
  // astar goal
  Pose2D goal_state_;

  AstarSearchState search_state_;

  AstarRequest request_;

  ParkObstacleList obs_;

  ObstacleClearZone clear_zone_;

  OccupancyGridMap ogm_;
  EulerDistanceTransform edt_;
  ParkReferenceLine ref_line_;
};

}  // namespace planning