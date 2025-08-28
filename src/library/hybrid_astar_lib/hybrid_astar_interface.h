#pragma once

#include <array>
#include <cstdint>
#include <memory>
#include <vector>

#include "Eigen/Core"
#include "hybrid_a_star.h"
#include "node3d.h"
#include "polygon_base.h"
#include "pose2d.h"
#include "src/library/convex_collision_detection/gjk2d_interface.h"
#include "src/library/occupancy_grid_map/point_cloud_obstacle.h"
#include "target_pose_regulator.h"

namespace planning {

class HybridAStarInterface {
 public:
  HybridAStarInterface();

  ~HybridAStarInterface();

  // todo: need to unify same vehicle chassis params for on lane driving and
  // parking
  int Init(const float back_edge_to_rear_axis, const float car_length,
           const float car_width, const float steer_ratio,
           const float wheel_base, const float min_turn_radius,
           const float mirror_width);

  // for now, use slot coordinate. you can call this API in one thread.
  void GeneratePath(const Eigen::Vector3d& start, const Eigen::Vector3d& end,
                    const AstarRequest& request);

  const AstarSearchState GetFullLengthPath(HybridAStarResult* result);

  AstarRequest* GetMutableRequest() { return &request_; }

  const AstarRequest& GetConstRequest() const { return request_; }

  static void GetRSPathInFullPath(std::vector<float>& x, std::vector<float>& y,
                                  std::vector<float>& phi,
                                  const HybridAStarResult& result);

  const HybridAStarResult* GetConstFullLengthPath() const;

  const bool GetFirstSegmentPath(std::vector<AStarPathPoint>& result);

  static const AstarSearchState TransformFirstSegmentPath(
      std::vector<AStarPathPoint>& result, const HybridAStarResult& full_path,
      const Pose2f& start);

  const int GetFallBackPath(std::vector<AStarPathPoint>& result);

  const int GetFallBackPath(HybridAStarResult* result);

  const std::array<bool, 6> GetFeasibleDirections() {
    return feasible_directions_;
  };

  const ParkObstacleList& GetConstObstacles() const;

  ParkObstacleList& GetMutableObstacleList();

  const Pose2f GetAstarTargetPose() const { return target_regulator_goal_; }

  // multi-thread, input
  void UpdateInput(const ParkObstacleList& obs_list,
                   const AstarRequest& request);

  // multi-thread, output
  void UpdateOutput();

  const EulerDistanceTransform* GetEulerDistanceTransform() const {
    return &edt_;
  }

  EulerDistanceTransform* GetMutableEDT() { return &edt_; }

  // for debug
  void GetRSPathHeuristic(std::vector<std::vector<Vec2f>>& path_list);

  // for debug
  const std::vector<DebugAstarSearchPoint>& GetChildNodeForDebug();

  // for debug
  const std::vector<Vec2f>& GetPriorQueueNode();
  const std::vector<Vec2f>& GetDelNodeQueueNode();
  // for debug,retired
  void GetNodeListMessage(planning::common::AstarNodeList* list);

  // for debug
  void GetNodeListMessage(std::vector<std::vector<Eigen::Vector2d>>& list);

  // for debug
  void GetRSPathForDebug(std::vector<float>& x, std::vector<float>& y,
                         std::vector<float>& phi);

  const ParkReferenceLine& GetConstRefLine() const;

  // for debug
  void GetPolynomialPathForDebug(std::vector<float>& x, std::vector<float>& y,
                                 std::vector<float>& phi);

  void UpdateEDTByObs(const ParkObstacleList& obs_list);

  FootPrintCircleModel* GetSlotOutsideCircleFootPrint();

 private:
  int UpdateEDT();

  void PathClear();

  void UpdateGridMapBound();

  void UpdateEDTBasePose(Pose2f& ogm_base_pose);

  const Pose2f& GetStartPoint();

  const Pose2f& GetGoalPoint();

  const bool IsEgoOverlapWithSlot();

  // 基于采样的揉库API, will be retired.
  void PathSamplingForScenarioRunning();

  void PathSearchForScenarioTry(const TargetPoseRegulator& regulator);

  // 基于搜索的路径生成API
  void PathSearchForScenarioRunning(const TargetPoseRegulator& regulator,
                                    const float ego_obs_dist,
                                    const bool is_ego_overlap_with_slot);

  // todo: move it to safe buffer decider
  // return safe buffer when path is insidet slot.
  // Need to consider:
  // 1. distance from ego to obstacle;
  // 2. distance from target pose to obstacle;
  const float GetLatBufferForInsideSlot(const float target_obs_dist,
                                        const float ego_obs_dist,
                                        const bool is_ego_overlap_with_slot);

  void PathCandidateCompare();

 private:
  // read vehicle param from file
  VehicleParam vehicle_param_;
  PlannerOpenSpaceConfig config_;
  // xmin, xmax, ymin, ymax
  MapBound map_bounds_;

  std::shared_ptr<HybridAStar> hybrid_astar_;
  std::shared_ptr<GridSearch> dp_heuristic_generator_;
  // path = astar node path + rs path.
  HybridAStarResult* best_traj_;
  std::array<HybridAStarResult, 3> traj_candidates_;

  Pose2f ego_state_;
  // 对于垂直、平行车位，goal_state位于车位中心线上.
  // 位姿调节器依赖这个pose重新计算搜索目标点.
  Pose2f goal_state_;
  // 目标调节器会计算一个合适的目标
  Pose2f target_regulator_goal_;

  AstarSearchState search_state_;

  AstarRequest request_;

  ParkObstacleList obs_;

  ObstacleClearZone clear_zone_;

  OccupancyGridMap ogm_;
  EulerDistanceTransform edt_;
  ParkReferenceLine ref_line_;

  int gear_switch_number_scenario_try_;

  std::array<bool, 6> feasible_directions_;
};

}  // namespace planning